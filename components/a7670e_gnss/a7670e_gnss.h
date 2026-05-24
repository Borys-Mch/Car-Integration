#pragma once

#include <string>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace a7670e_gnss {

class A7670EGNSSComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_latitude_sensor(sensor::Sensor *sensor) { latitude_sensor_ = sensor; }
  void set_longitude_sensor(sensor::Sensor *sensor) { longitude_sensor_ = sensor; }
  void set_altitude_sensor(sensor::Sensor *sensor) { altitude_sensor_ = sensor; }
  void set_speed_sensor(sensor::Sensor *sensor) { speed_sensor_ = sensor; }
  void set_course_sensor(sensor::Sensor *sensor) { course_sensor_ = sensor; }
  void set_hdop_sensor(sensor::Sensor *sensor) { hdop_sensor_ = sensor; }
  void set_satellites_sensor(sensor::Sensor *sensor) { satellites_sensor_ = sensor; }
  void set_fix_mode_sensor(sensor::Sensor *sensor) { fix_mode_sensor_ = sensor; }

 protected:
  sensor::Sensor *latitude_sensor_{nullptr};
  sensor::Sensor *longitude_sensor_{nullptr};
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *course_sensor_{nullptr};
  sensor::Sensor *hdop_sensor_{nullptr};
  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *fix_mode_sensor_{nullptr};

  bool initialized_{false};
  uint8_t init_index_{0};

  bool initialize_next_command_();
  void flush_input_();
  std::string transact_(const char *command, uint32_t timeout_ms = 5000);
  bool parse_cgnssinfo_(const std::string &response);
  static std::vector<std::string> split_csv_(const std::string &line);
  static bool parse_float_(const std::string &value, float *out);
  static bool parse_int_(const std::string &value, int *out);
  static bool parse_coordinate_(const std::string &value, const std::string &hemisphere, bool latitude, float *out);
};

}  // namespace a7670e_gnss
}  // namespace esphome
