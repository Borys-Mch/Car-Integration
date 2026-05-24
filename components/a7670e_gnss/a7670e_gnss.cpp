#include "a7670e_gnss.h"

#include <cstdlib>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace a7670e_gnss {

static const char *const TAG = "a7670e_gnss";

void A7670EGNSSComponent::setup() {
  this->flush_input_();
}

void A7670EGNSSComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "A7670E GNSS");
  LOG_UPDATE_INTERVAL(this);
}

void A7670EGNSSComponent::update() {
  if (!this->initialized_) {
    this->initialize_next_command_();
    return;
  }

  std::string response = this->transact_("AT+CGNSSINFO", 9000);
  if (!this->parse_cgnssinfo_(response)) {
    ESP_LOGW(TAG, "No valid GNSS fix: %s", response.c_str());
  }
}

bool A7670EGNSSComponent::initialize_next_command_() {
  static const char *const commands[] = {
      "ATE0",
      "AT+CGNSSPWR=1",
      "AT+CGNSSINFO=0",
  };
  static const uint8_t command_count = sizeof(commands) / sizeof(commands[0]);

  if (this->init_index_ >= command_count) {
    this->initialized_ = true;
    ESP_LOGI(TAG, "A7670E GNSS initialization finished");
    return true;
  }

  const char *command = commands[this->init_index_];
  std::string response = this->transact_(command, 9000);
  ESP_LOGD(TAG, "%s -> %s", command, response.c_str());

  if (response.find("ERROR") != std::string::npos) {
    ESP_LOGW(TAG, "A7670E command failed: %s -> %s", command, response.c_str());
  }

  this->init_index_++;
  return false;
}

void A7670EGNSSComponent::flush_input_() {
  while (this->available()) {
    this->read();
  }
}

std::string A7670EGNSSComponent::transact_(const char *command, uint32_t timeout_ms) {
  this->flush_input_();
  this->write_str(command);
  this->write_byte('\r');
  this->flush();

  std::string response;
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    while (this->available()) {
      char c = static_cast<char>(this->read());
      if (c != '\r' && c != '\n') {
        response.push_back(c);
      }
    }

    if (response.find("OK") != std::string::npos || response.find("ERROR") != std::string::npos) {
      return response;
    }
    delay(5);
  }
  return response;
}

bool A7670EGNSSComponent::parse_cgnssinfo_(const std::string &response) {
  const std::string prefix = "+CGNSSINFO:";
  size_t start = response.find(prefix);
  if (start == std::string::npos) {
    return false;
  }

  start += prefix.size();
  size_t end = response.find("OK", start);
  std::string payload = response.substr(start, end == std::string::npos ? std::string::npos : end - start);
  std::vector<std::string> fields = split_csv_(payload);
  if (fields.size() < 13) {
    return false;
  }

  int mode = 0;
  int satellites = 0;
  float latitude = 0.0f;
  float longitude = 0.0f;
  float altitude = 0.0f;
  float speed_knots = 0.0f;
  float course = 0.0f;
  float hdop = 0.0f;

  if (!parse_int_(fields[0], &mode) || mode < 2) {
    return false;
  }

  size_t lat_hemi_index = std::string::npos;
  for (size_t i = 1; i < fields.size(); i++) {
    if (fields[i] == "N" || fields[i] == "S") {
      lat_hemi_index = i;
      break;
    }
  }

  if (lat_hemi_index == std::string::npos || lat_hemi_index == 0 || lat_hemi_index + 2 >= fields.size()) {
    return false;
  }

  size_t lon_hemi_index = lat_hemi_index + 2;
  if (fields[lon_hemi_index] != "E" && fields[lon_hemi_index] != "W") {
    return false;
  }

  if (!parse_coordinate_(fields[lat_hemi_index - 1], fields[lat_hemi_index], true, &latitude)) {
    return false;
  }
  if (!parse_coordinate_(fields[lat_hemi_index + 1], fields[lon_hemi_index], false, &longitude)) {
    return false;
  }

  for (size_t i = 1; i + 1 < lat_hemi_index; i++) {
    int value = 0;
    if (parse_int_(fields[i], &value)) {
      satellites += value;
    }
  }

  size_t altitude_index = lon_hemi_index + 3;
  size_t speed_index = lon_hemi_index + 4;
  size_t course_index = lon_hemi_index + 5;
  size_t hdop_index = lon_hemi_index + 7;

  if (altitude_index < fields.size()) {
    parse_float_(fields[altitude_index], &altitude);
  }
  if (speed_index < fields.size()) {
    parse_float_(fields[speed_index], &speed_knots);
  }
  if (course_index < fields.size()) {
    parse_float_(fields[course_index], &course);
  }
  if (hdop_index < fields.size()) {
    parse_float_(fields[hdop_index], &hdop);
  }

  if (this->fix_mode_sensor_ != nullptr) {
    this->fix_mode_sensor_->publish_state(mode);
  }
  if (this->satellites_sensor_ != nullptr) {
    this->satellites_sensor_->publish_state(satellites);
  }
  if (this->latitude_sensor_ != nullptr) {
    this->latitude_sensor_->publish_state(latitude);
  }
  if (this->longitude_sensor_ != nullptr) {
    this->longitude_sensor_->publish_state(longitude);
  }
  if (this->altitude_sensor_ != nullptr) {
    this->altitude_sensor_->publish_state(altitude);
  }
  if (this->speed_sensor_ != nullptr) {
    this->speed_sensor_->publish_state(speed_knots * 1.852f);
  }
  if (this->course_sensor_ != nullptr) {
    this->course_sensor_->publish_state(course);
  }
  if (this->hdop_sensor_ != nullptr) {
    this->hdop_sensor_->publish_state(hdop);
  }

  ESP_LOGD(TAG, "GNSS fix mode=%d lat=%.6f lon=%.6f sats=%d", mode, latitude, longitude, satellites);
  return true;
}

std::vector<std::string> A7670EGNSSComponent::split_csv_(const std::string &line) {
  std::vector<std::string> fields;
  std::string field;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(field);
      field.clear();
    } else if (c != ' ') {
      field.push_back(c);
    }
  }
  fields.push_back(field);
  return fields;
}

bool A7670EGNSSComponent::parse_float_(const std::string &value, float *out) {
  if (value.empty()) {
    return false;
  }
  char *end = nullptr;
  float parsed = strtof(value.c_str(), &end);
  if (end == value.c_str()) {
    return false;
  }
  *out = parsed;
  return true;
}

bool A7670EGNSSComponent::parse_int_(const std::string &value, int *out) {
  if (value.empty()) {
    return false;
  }
  char *end = nullptr;
  long parsed = strtol(value.c_str(), &end, 10);
  if (end == value.c_str()) {
    return false;
  }
  *out = static_cast<int>(parsed);
  return true;
}

bool A7670EGNSSComponent::parse_coordinate_(const std::string &value, const std::string &hemisphere, bool latitude,
                                            float *out) {
  float raw = 0.0f;
  if (!parse_float_(value, &raw) || hemisphere.empty()) {
    return false;
  }

  float coordinate = raw;
  if ((latitude && raw > 90.0f) || (!latitude && raw > 180.0f)) {
    int degrees = static_cast<int>(raw / 100.0f);
    float minutes = raw - degrees * 100.0f;
    coordinate = degrees + minutes / 60.0f;
  }

  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    coordinate = -coordinate;
  }
  *out = coordinate;
  return true;
}

}  // namespace a7670e_gnss
}  // namespace esphome
