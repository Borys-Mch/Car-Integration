#include "elm327_uart.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace elm327_uart {

static const char *const TAG = "elm327_uart";

void ELM327UARTComponent::setup() {
  this->flush_input_();
}

void ELM327UARTComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ELM327 UART OBD-II");
  LOG_UPDATE_INTERVAL(this);
}

bool ELM327UARTComponent::initialize_next_command_() {
  static const char *const commands[] = {
      "ATZ",   // reset
      "ATE0",  // echo off
      "ATL0",  // linefeeds off
      "ATS0",  // spaces off
      "ATH0",  // headers off
      "ATAT1", // adaptive timing
      "ATST64", // 400 ms OBD response timeout inside ELM327
      "ATSP0", // automatic protocol
  };
  static const uint8_t command_count = sizeof(commands) / sizeof(commands[0]);

  if (this->init_index_ >= command_count) {
    if (this->init_responses_ > 0) {
      this->initialized_ = true;
      ESP_LOGI(TAG, "ELM327 initialization finished");
      return true;
    }

    ESP_LOGW(TAG, "ELM327 did not answer initialization commands; check power, UART pins, baud rate, and GND");
    this->init_index_ = 0;
    this->init_responses_ = 0;
    return false;
  }

  const char *command = commands[this->init_index_];
  std::string response = this->transact_(command, this->init_index_ == 0 ? 1200 : 900);
  ESP_LOGD(TAG, "%s -> %s", command, response.c_str());
  if (this->is_init_response_valid_(response)) {
    this->init_responses_++;
  } else if (!response.empty()) {
    ESP_LOGW(TAG, "Ignoring invalid ELM327 init response for %s: %s", command, response.c_str());
  }
  this->init_index_++;
  return false;
}

void ELM327UARTComponent::update() {
  if (!this->initialized_) {
    this->initialize_next_command_();
    return;
  }

  struct PollItem {
    const char *command;
    const char *header;
  };

  static const PollItem items[] = {
      {"010C", "410C"},
      {"010D", "410D"},
      {"0105", "4105"},
      {"0142", "4142"},
      {"0111", "4111"},
      {"010F", "410F"},
  };

  const PollItem &item = items[this->poll_index_];
  this->poll_index_ = (this->poll_index_ + 1) % (sizeof(items) / sizeof(items[0]));

  std::string response = this->transact_(item.command);
  std::vector<uint8_t> bytes;

  if (!this->extract_bytes_(response, item.header, &bytes)) {
    ESP_LOGW(TAG, "No usable response for %s: %s", item.command, response.c_str());
    return;
  }

  if (strcmp(item.command, "010C") == 0 && bytes.size() >= 2 && this->rpm_sensor_ != nullptr) {
    float rpm = ((bytes[0] * 256.0f) + bytes[1]) / 4.0f;
    this->rpm_sensor_->publish_state(rpm);
  } else if (strcmp(item.command, "010D") == 0 && bytes.size() >= 1 && this->speed_sensor_ != nullptr) {
    this->speed_sensor_->publish_state(bytes[0]);
  } else if (strcmp(item.command, "0105") == 0 && bytes.size() >= 1 && this->coolant_temperature_sensor_ != nullptr) {
    this->coolant_temperature_sensor_->publish_state(bytes[0] - 40);
  } else if (strcmp(item.command, "0142") == 0 && bytes.size() >= 2 && this->control_module_voltage_sensor_ != nullptr) {
    float voltage = ((bytes[0] * 256.0f) + bytes[1]) / 1000.0f;
    this->control_module_voltage_sensor_->publish_state(voltage);
  } else if (strcmp(item.command, "0111") == 0 && bytes.size() >= 1 && this->throttle_position_sensor_ != nullptr) {
    this->throttle_position_sensor_->publish_state(bytes[0] * 100.0f / 255.0f);
  } else if (strcmp(item.command, "010F") == 0 && bytes.size() >= 1 && this->intake_air_temperature_sensor_ != nullptr) {
    this->intake_air_temperature_sensor_->publish_state(bytes[0] - 40);
  }
}

void ELM327UARTComponent::flush_input_() {
  while (this->available()) {
    this->read();
  }
}

std::string ELM327UARTComponent::transact_(const char *command, uint32_t timeout_ms) {
  this->flush_input_();
  this->write_str(command);
  this->write_byte('\r');
  this->flush();

  std::string response;
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    while (this->available()) {
      char c = static_cast<char>(this->read());
      if (c == '>') {
        return response;
      }
      if (c != '\r' && c != '\n') {
        response.push_back(c);
      }
    }
    delay(5);
  }
  return response;
}

bool ELM327UARTComponent::is_init_response_valid_(const std::string &response) {
  std::string normalized;
  normalized.reserve(response.size());
  for (char c : response) {
    if (c >= 'a' && c <= 'z') {
      normalized.push_back(static_cast<char>(c - 'a' + 'A'));
    } else if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '?' || c == '.') {
      normalized.push_back(c);
    }
  }

  return normalized.find("OK") != std::string::npos || normalized.find("ELM") != std::string::npos;
}

bool ELM327UARTComponent::extract_bytes_(const std::string &response, const char *header, std::vector<uint8_t> *bytes) {
  bytes->clear();
  std::string normalized = normalize_hex_(response);
  size_t pos = normalized.find(header);
  if (pos == std::string::npos) {
    return false;
  }

  pos += strlen(header);
  while (pos + 1 < normalized.size()) {
    int high = hex_value_(normalized[pos]);
    int low = hex_value_(normalized[pos + 1]);
    if (high < 0 || low < 0) {
      break;
    }
    bytes->push_back(static_cast<uint8_t>((high << 4) | low));
    pos += 2;
  }

  return !bytes->empty();
}

std::string ELM327UARTComponent::normalize_hex_(const std::string &input) {
  std::string output;
  output.reserve(input.size());
  for (char c : input) {
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) {
      output.push_back(c);
    } else if (c >= 'a' && c <= 'f') {
      output.push_back(static_cast<char>(c - 'a' + 'A'));
    }
  }
  return output;
}

int ELM327UARTComponent::hex_value_(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

}  // namespace elm327_uart
}  // namespace esphome
