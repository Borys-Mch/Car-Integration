#pragma once

#include <string>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace elm327_uart {

class ELM327UARTComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_rpm_sensor(sensor::Sensor *sensor) { rpm_sensor_ = sensor; }
  void set_speed_sensor(sensor::Sensor *sensor) { speed_sensor_ = sensor; }
  void set_coolant_temperature_sensor(sensor::Sensor *sensor) { coolant_temperature_sensor_ = sensor; }
  void set_control_module_voltage_sensor(sensor::Sensor *sensor) { control_module_voltage_sensor_ = sensor; }
  void set_throttle_position_sensor(sensor::Sensor *sensor) { throttle_position_sensor_ = sensor; }
  void set_intake_air_temperature_sensor(sensor::Sensor *sensor) { intake_air_temperature_sensor_ = sensor; }

 protected:
  sensor::Sensor *rpm_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *coolant_temperature_sensor_{nullptr};
  sensor::Sensor *control_module_voltage_sensor_{nullptr};
  sensor::Sensor *throttle_position_sensor_{nullptr};
  sensor::Sensor *intake_air_temperature_sensor_{nullptr};

  uint8_t poll_index_{0};
  uint8_t init_index_{0};
  uint8_t init_responses_{0};
  bool initialized_{false};

  bool initialize_next_command_();
  void flush_input_();
  std::string transact_(const char *command, uint32_t timeout_ms = 1800);
  bool is_init_response_valid_(const std::string &response);
  bool extract_bytes_(const std::string &response, const char *header, std::vector<uint8_t> *bytes);
  static std::string normalize_hex_(const std::string &input);
  static int hex_value_(char c);
};

}  // namespace elm327_uart
}  // namespace esphome
