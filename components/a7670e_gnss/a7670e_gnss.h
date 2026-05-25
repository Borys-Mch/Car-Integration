#pragma once

#include <string>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome
{
  namespace a7670e_gnss
  {

    class A7670EGNSSComponent : public PollingComponent, public uart::UARTDevice
    {
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
      void set_mqtt_apn(const std::string &apn) { mqtt_apn_ = apn; }
      void set_mqtt_host(const std::string &host) { mqtt_host_ = host; }
      void set_mqtt_port(uint16_t port) { mqtt_port_ = port; }
      void set_mqtt_username(const std::string &username) { mqtt_username_ = username; }
      void set_mqtt_password(const std::string &password) { mqtt_password_ = password; }
      void set_mqtt_topic(const std::string &topic) { mqtt_topic_ = topic; }
      void set_mqtt_client_id(const std::string &client_id) { mqtt_client_id_ = client_id; }
      void set_mqtt_use_tls(bool use_tls) { mqtt_use_tls_ = use_tls; }

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
      bool mqtt_enabled_{false};
      bool mqtt_started_{false};
      bool mqtt_connected_{false};
      bool discovery_published_{false};
      uint8_t init_index_{0};
      uint8_t mqtt_failures_{0};

      std::string mqtt_apn_;
      std::string mqtt_host_;
      uint16_t mqtt_port_{1883};
      std::string mqtt_username_;
      std::string mqtt_password_;
      std::string mqtt_topic_{"car/forester/state"};
      std::string mqtt_client_id_{"car-integration"};
      bool mqtt_use_tls_{true};
      uint32_t last_mqtt_retry_{0};

      bool initialize_next_command_();
      bool ensure_mqtt_connected_();
      bool mqtt_command_ok_(const char *command, uint32_t timeout_ms = 12000);
      bool mqtt_publish_(const std::string &payload);
      bool mqtt_publish_raw_(const std::string &topic, const std::string &payload, bool retain);
      void publish_discovery_();
      void flush_input_();
      std::string transact_(const char *command, uint32_t timeout_ms = 5000);
      bool write_prompt_data_(const std::string &data, uint32_t timeout_ms = 5000);
      bool parse_cgnssinfo_(const std::string &response);
      std::string build_payload_(float latitude, float longitude, float altitude, float speed_kmh, float course, float hdop,
                                 int satellites, int mode);
      static std::vector<std::string> split_csv_(const std::string &line);
      static bool parse_float_(const std::string &value, float *out);
      static bool parse_int_(const std::string &value, int *out);
      static bool parse_coordinate_(const std::string &value, const std::string &hemisphere, bool latitude, float *out);
    };

  } // namespace a7670e_gnss
} // namespace esphome
