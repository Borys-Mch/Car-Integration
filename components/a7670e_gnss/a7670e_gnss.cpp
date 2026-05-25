#include "a7670e_gnss.h"

#include <cstdlib>
#include <cstdio>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace a7670e_gnss
  {

    static const char *const TAG = "a7670e_gnss";

    void A7670EGNSSComponent::setup()
    {
      this->flush_input_();
      this->mqtt_enabled_ = !this->mqtt_apn_.empty() && !this->mqtt_host_.empty() && !this->mqtt_username_.empty();
    }

    void A7670EGNSSComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "A7670E GNSS");
      LOG_UPDATE_INTERVAL(this);
    }

    void A7670EGNSSComponent::update()
    {
      if (!this->initialized_)
      {
        this->initialize_next_command_();
        return;
      }

      if (this->mqtt_enabled_)
      {
        uint32_t now = millis();
        if (!this->mqtt_connected_ && now - this->last_mqtt_retry_ > 15000)
        {
          this->last_mqtt_retry_ = now;
          this->ensure_mqtt_connected_();
        }
      }

      if (this->mqtt_connected_ && !this->discovery_published_)
      {
        this->publish_discovery_next_();
      }

      std::string response = this->transact_("AT+CGNSSINFO", 9000);
      if (!this->parse_cgnssinfo_(response))
      {
        ESP_LOGW(TAG, "No valid GNSS fix: %s", response.c_str());
      }
    }

    bool A7670EGNSSComponent::initialize_next_command_()
    {
      static const char *const commands[] = {
          "ATE0",
          "AT+CGNSSPWR=1",
          "AT+CGNSSINFO=0",
      };
      static const uint8_t command_count = sizeof(commands) / sizeof(commands[0]);

      if (this->init_index_ >= command_count)
      {
        this->initialized_ = true;
        ESP_LOGI(TAG, "A7670E GNSS initialization finished");
        return true;
      }

      const char *command = commands[this->init_index_];
      std::string response = this->transact_(command, 9000);
      ESP_LOGD(TAG, "%s -> %s", command, response.c_str());

      if (response.find("ERROR") != std::string::npos)
      {
        ESP_LOGW(TAG, "A7670E command failed: %s -> %s", command, response.c_str());
      }

      this->init_index_++;
      return false;
    }

    bool A7670EGNSSComponent::ensure_mqtt_connected_()
    {
      if (!this->mqtt_enabled_)
      {
        return false;
      }
      if (this->mqtt_connected_)
      {
        return true;
      }

      ESP_LOGI(TAG, "Connecting A7670E MQTT to %s:%u", this->mqtt_host_.c_str(), this->mqtt_port_);

      this->mqtt_command_ok_("AT+CPIN?", 5000);
      this->mqtt_command_ok_("AT+CREG?", 5000);
      this->mqtt_command_ok_("AT+CEREG?", 5000);

      std::string command = "AT+CGDCONT=1,\"IP\",\"" + this->mqtt_apn_ + "\"";
      if (!this->mqtt_command_ok_(command.c_str(), 8000))
      {
        return false;
      }

      this->mqtt_command_ok_("AT+CGACT=1,1", 30000);

      if (!this->mqtt_started_)
      {
        this->transact_("AT+CMQTTDISC=0,10", 5000);
        this->transact_("AT+CMQTTREL=0", 5000);
        this->transact_("AT+CMQTTSTOP", 10000);
        delay(2000);

        std::string response = this->transact_("AT+CMQTTSTART", 30000);
        ESP_LOGD(TAG, "AT+CMQTTSTART -> %s", response.c_str());
        if (response.find("OK") == std::string::npos && response.find("+CMQTTSTART: 0") == std::string::npos)
        {
          delay(1000);
          response = this->transact_("AT+CMQTTSTART", 30000);
          ESP_LOGD(TAG, "AT+CMQTTSTART retry -> %s", response.c_str());
          if (response.find("OK") == std::string::npos && response.find("+CMQTTSTART: 0") == std::string::npos)
          {
            this->mqtt_failures_++;
            this->mqtt_started_ = false;
            return false;
          }
        }
        this->mqtt_started_ = true;
      }

      this->transact_("AT+CMQTTREL=0", 5000);

      command = "AT+CMQTTACCQ=0,\"" + this->mqtt_client_id_ + "\"," + std::string(this->mqtt_use_tls_ ? "1" : "0");
      if (!this->mqtt_command_ok_(command.c_str(), 8000))
      {
        this->mqtt_failures_++;
        return false;
      }

      if (this->mqtt_use_tls_)
      {
        this->mqtt_command_ok_("AT+CSSLCFG=\"sslversion\",0,3", 8000);
        this->mqtt_command_ok_("AT+CSSLCFG=\"authmode\",0,0", 8000);
        this->mqtt_command_ok_("AT+CSSLCFG=\"ignorlocaltime\",0,1", 8000);
        if (!this->mqtt_command_ok_("AT+CMQTTSSLCFG=0,0", 8000))
        {
          this->mqtt_failures_++;
          return false;
        }
      }

      command = "AT+CMQTTCONNECT=0,\"tcp://" + this->mqtt_host_ + ":" + std::to_string(this->mqtt_port_) +
                "\",60,1,\"" + this->mqtt_username_ + "\",\"" + this->mqtt_password_ + "\"";
      std::string response = this->transact_(command.c_str(), 60000);
      ESP_LOGD(TAG, "AT+CMQTTCONNECT -> %s", response.c_str());
      if (response.find("+CMQTTCONNECT: 0,0") == std::string::npos && response.find("OK") == std::string::npos)
      {
        this->mqtt_failures_++;
        this->mqtt_connected_ = false;
        this->mqtt_started_ = false;
        return false;
      }

      this->mqtt_failures_ = 0;
      this->mqtt_connected_ = true;
      ESP_LOGI(TAG, "A7670E MQTT connected");
      return true;
    }

    bool A7670EGNSSComponent::mqtt_command_ok_(const char *command, uint32_t timeout_ms)
    {
      std::string response = this->transact_(command, timeout_ms);
      ESP_LOGD(TAG, "%s -> %s", command, response.c_str());
      return response.find("OK") != std::string::npos || response.find(command) != std::string::npos;
    }

    bool A7670EGNSSComponent::mqtt_publish_(const std::string &payload)
    {
      if (!this->ensure_mqtt_connected_())
      {
        ESP_LOGW(TAG, "MQTT is not connected; skipping publish");
        return false;
      }

      return this->mqtt_publish_raw_(this->mqtt_topic_, payload, false);
    }

    bool A7670EGNSSComponent::mqtt_publish_raw_(const std::string &topic, const std::string &payload, bool retain)
    {
      std::string command = "AT+CMQTTTOPIC=0," + std::to_string(topic.size());
      std::string response = this->transact_(command.c_str(), 8000);
      ESP_LOGD(TAG, "%s -> %s", command.c_str(), response.c_str());
      if (response.find(">") == std::string::npos && response.find("OK") == std::string::npos)
      {
        this->mqtt_connected_ = false;
        this->mqtt_started_ = false;
        return false;
      }
      if (!this->write_prompt_data_(topic, 8000))
      {
        this->mqtt_connected_ = false;
        this->mqtt_started_ = false;
        return false;
      }

      command = "AT+CMQTTPAYLOAD=0," + std::to_string(payload.size());
      response = this->transact_(command.c_str(), 8000);
      ESP_LOGD(TAG, "%s -> %s", command.c_str(), response.c_str());
      if (response.find(">") == std::string::npos && response.find("OK") == std::string::npos)
      {
        this->mqtt_connected_ = false;
        this->mqtt_started_ = false;
        return false;
      }
      if (!this->write_prompt_data_(payload, 8000))
      {
        this->mqtt_connected_ = false;
        this->mqtt_started_ = false;
        return false;
      }

      command = "AT+CMQTTPUB=0,1,60," + std::string(retain ? "1" : "0");
      response = this->transact_(command.c_str(), 30000);
      ESP_LOGD(TAG, "AT+CMQTTPUB -> %s", response.c_str());
      if (response.find("+CMQTTPUB: 0,0") == std::string::npos && response.find("OK") == std::string::npos)
      {
        this->mqtt_connected_ = false;
        this->mqtt_started_ = false;
        return false;
      }

      ESP_LOGD(TAG, "Published MQTT payload: %s", payload.c_str());
      return true;
    }

    void A7670EGNSSComponent::publish_discovery_()
    {
      if (this->discovery_published_)
      {
        return;
      }

      ESP_LOGI(TAG, "Publishing Home Assistant MQTT discovery");

      const std::string device =
          "\"dev\":{\"ids\":[\"car_forester\"],\"name\":\"Car Forester\","
          "\"mf\":\"DIY\",\"mdl\":\"ESP32-C6 + A7670E + ELM327\"}";
      const std::string availability =
          ",\"avty_t\":\"car/forester/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\"";
      const std::string state = "\"stat_t\":\"" + this->mqtt_topic_ + "\"";

      struct DiscoverySensor
      {
        const char *uid;
        const char *name;
        const char *value_template;
        const char *unit;
        const char *icon;
        const char *device_class;
      };

      static const DiscoverySensor sensors[] = {
          {"car_forester_lat", "Car Latitude", "{{ value_json.lat }}", "°", "mdi:crosshairs-gps", ""},
          {"car_forester_lon", "Car Longitude", "{{ value_json.lon }}", "°", "mdi:crosshairs-gps", ""},
          {"car_forester_altitude", "Car Altitude", "{{ value_json.altitude }}", "m", "mdi:map-marker-distance",
           "distance"},
          {"car_forester_gnss_speed", "Car GNSS Speed", "{{ value_json.speed }}", "km/h", "mdi:speedometer", "speed"},
          {"car_forester_course", "Car Course", "{{ value_json.course }}", "°", "mdi:compass", ""},
          {"car_forester_hdop", "Car GNSS HDOP", "{{ value_json.hdop }}", "", "mdi:map-marker-check", ""},
          {"car_forester_satellites", "Car GNSS Satellites", "{{ value_json.satellites }}", "", "mdi:satellite-variant",
           ""},
          {"car_forester_fix_mode", "Car GNSS Fix Mode", "{{ value_json.fix_mode }}", "", "mdi:crosshairs-question", ""},
      };

      bool ok = true;
      for (const auto &sensor : sensors)
      {
        std::string topic = "homeassistant/sensor/" + std::string(sensor.uid) + "/config";
        std::string payload = "{\"name\":\"" + std::string(sensor.name) + "\",\"uniq_id\":\"" +
                              std::string(sensor.uid) + "\"," + state + ",\"val_tpl\":\"" +
                              std::string(sensor.value_template) + "\"";
        if (sensor.unit[0] != '\0')
        {
          payload += ",\"unit_of_meas\":\"" + std::string(sensor.unit) + "\"";
        }
        if (sensor.icon[0] != '\0')
        {
          payload += ",\"icon\":\"" + std::string(sensor.icon) + "\"";
        }
        if (sensor.device_class[0] != '\0')
        {
          payload += ",\"dev_cla\":\"" + std::string(sensor.device_class) + "\"";
        }
        payload += availability + "," + device + "}";

        ok = this->mqtt_publish_raw_(topic, payload, true) && ok;
        delay(50);
      }

      ok = this->mqtt_publish_raw_("car/forester/status", "online", true) && ok;
      this->discovery_published_ = ok;
      if (ok)
      {
        ESP_LOGI(TAG, "Home Assistant MQTT discovery published");
      }
      else
      {
        ESP_LOGW(TAG, "Home Assistant MQTT discovery publish failed");
      }
    }

    void A7670EGNSSComponent::publish_discovery_next_()
    {
      const std::string device =
          "\"dev\":{\"ids\":[\"car_forester\"],\"name\":\"Car Forester\","
          "\"mf\":\"DIY\",\"mdl\":\"ESP32-C6 + A7670E + ELM327\"}";
      const std::string availability =
          ",\"avty_t\":\"car/forester/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\"";
      const std::string state = "\"stat_t\":\"" + this->mqtt_topic_ + "\"";

      struct DiscoverySensor
      {
        const char *uid;
        const char *name;
        const char *value_template;
        const char *unit;
        const char *icon;
        const char *device_class;
      };

      static const DiscoverySensor sensors[] = {
          {"car_forester_lat", "Car Latitude", "{{ value_json.lat }}", "°", "mdi:crosshairs-gps", ""},
          {"car_forester_lon", "Car Longitude", "{{ value_json.lon }}", "°", "mdi:crosshairs-gps", ""},
          {"car_forester_altitude", "Car Altitude", "{{ value_json.altitude }}", "m", "mdi:map-marker-distance",
           "distance"},
          {"car_forester_gnss_speed", "Car GNSS Speed", "{{ value_json.speed }}", "km/h", "mdi:speedometer", "speed"},
          {"car_forester_course", "Car Course", "{{ value_json.course }}", "°", "mdi:compass", ""},
          {"car_forester_hdop", "Car GNSS HDOP", "{{ value_json.hdop }}", "", "mdi:map-marker-check", ""},
          {"car_forester_satellites", "Car GNSS Satellites", "{{ value_json.satellites }}", "", "mdi:satellite-variant",
           ""},
          {"car_forester_fix_mode", "Car GNSS Fix Mode", "{{ value_json.fix_mode }}", "", "mdi:crosshairs-question", ""},
          {"car_rpm", "Car RPM", "{{ value_json.rpm }}", "rpm", "mdi:engine", ""},
          {"car_speed_obd", "Car Speed OBD", "{{ value_json.speed_obd }}", "km/h", "mdi:speedometer", "speed"},
          {"car_coolant_temp", "Car Coolant Temperature", "{{ value_json.coolant_temp }}", "°C", "mdi:thermometer", "temperature"},
          {"car_throttle", "Car Throttle Position", "{{ value_json.throttle }}", "%", "mdi:car-turbocharger", ""},
          {"car_voltage", "Car ECU Voltage", "{{ value_json.voltage }}", "V", "mdi:car-battery", "voltage"},
          {"car_intake_temp", "Car Intake Air Temperature", "{{ value_json.intake_temp }}", "°C", "mdi:thermometer", "temperature"},
      };
      static const uint8_t sensor_count = sizeof(sensors) / sizeof(sensors[0]);

      if (this->discovery_index_ < sensor_count)
      {
        const auto &sensor = sensors[this->discovery_index_];
        std::string topic = "homeassistant/sensor/" + std::string(sensor.uid) + "/config";
        std::string payload = "{\"name\":\"" + std::string(sensor.name) + "\",\"uniq_id\":\"" +
                              std::string(sensor.uid) + "\"," + state + ",\"val_tpl\":\"" +
                              std::string(sensor.value_template) + "\"";
        if (sensor.unit[0] != '\0')
        {
          payload += ",\"unit_of_meas\":\"" + std::string(sensor.unit) + "\"";
        }
        if (sensor.icon[0] != '\0')
        {
          payload += ",\"icon\":\"" + std::string(sensor.icon) + "\"";
        }
        if (sensor.device_class[0] != '\0')
        {
          payload += ",\"dev_cla\":\"" + std::string(sensor.device_class) + "\"";
        }
        payload += availability + "," + device + "}";

        this->mqtt_publish_raw_(topic, payload, true);
        this->discovery_index_++;
      }
      else if (this->discovery_index_ == sensor_count)
      {
        std::string button_payload = "{\"name\":\"Open Gate\",\"uniq_id\":\"car_barrier_btn\",\"cmd_t\":\"car/forester/cmd\","
                                     "\"pl_press\":\"OPEN\",\"icon\":\"mdi:boom-gate-up\"," + availability + "," + device + "}";
        this->mqtt_publish_raw_("homeassistant/button/car_barrier_btn/config", button_payload, true);
        this->discovery_index_++;
      }
      else if (this->discovery_index_ == sensor_count + 1)
      {
        this->mqtt_publish_raw_("car/forester/status", "online", true);
        this->discovery_published_ = true;
        ESP_LOGI(TAG, "Home Assistant MQTT discovery published");
      }
    }

    void A7670EGNSSComponent::flush_input_()
    {
      while (this->available())
      {
        this->read();
      }
    }

    std::string A7670EGNSSComponent::transact_(const char *command, uint32_t timeout_ms)
    {
      this->flush_input_();
      this->write_str(command);
      this->write_byte('\r');
      this->flush();

      std::string response;
      uint32_t start = millis();
      while (millis() - start < timeout_ms)
      {
        while (this->available())
        {
          char c = static_cast<char>(this->read());
          if (c == '>')
          {
            response.push_back(c);
            return response;
          }
          if (c != '\r' && c != '\n')
          {
            response.push_back(c);
          }
        }

        if (response.find("OK") != std::string::npos || response.find("ERROR") != std::string::npos)
        {
          return response;
        }
        delay(5);
      }
      return response;
    }

    bool A7670EGNSSComponent::write_prompt_data_(const std::string &data, uint32_t timeout_ms)
    {
      this->write_array(reinterpret_cast<const uint8_t *>(data.data()), data.size());
      this->flush();

      std::string response;
      uint32_t start = millis();
      while (millis() - start < timeout_ms)
      {
        while (this->available())
        {
          char c = static_cast<char>(this->read());
          if (c != '\r' && c != '\n')
          {
            response.push_back(c);
          }
        }
        if (response.find("OK") != std::string::npos || response.find("ERROR") != std::string::npos)
        {
          ESP_LOGD(TAG, "prompt data -> %s", response.c_str());
          return response.find("OK") != std::string::npos;
        }
        delay(5);
      }
      ESP_LOGW(TAG, "No response after prompt data: %s", response.c_str());
      return false;
    }

    bool A7670EGNSSComponent::parse_cgnssinfo_(const std::string &response)
    {
      const std::string prefix = "+CGNSSINFO:";
      size_t start = response.find(prefix);
      if (start == std::string::npos)
      {
        return false;
      }

      start += prefix.size();
      size_t end = response.find("OK", start);
      std::string payload = response.substr(start, end == std::string::npos ? std::string::npos : end - start);
      std::vector<std::string> fields = split_csv_(payload);
      if (fields.size() < 13)
      {
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

      if (!parse_int_(fields[0], &mode) || mode < 2)
      {
        return false;
      }

      size_t lat_hemi_index = std::string::npos;
      for (size_t i = 1; i < fields.size(); i++)
      {
        if (fields[i] == "N" || fields[i] == "S")
        {
          lat_hemi_index = i;
          break;
        }
      }

      if (lat_hemi_index == std::string::npos || lat_hemi_index == 0 || lat_hemi_index + 2 >= fields.size())
      {
        return false;
      }

      size_t lon_hemi_index = lat_hemi_index + 2;
      if (fields[lon_hemi_index] != "E" && fields[lon_hemi_index] != "W")
      {
        return false;
      }

      if (!parse_coordinate_(fields[lat_hemi_index - 1], fields[lat_hemi_index], true, &latitude))
      {
        return false;
      }
      if (!parse_coordinate_(fields[lat_hemi_index + 1], fields[lon_hemi_index], false, &longitude))
      {
        return false;
      }

      for (size_t i = 1; i + 1 < lat_hemi_index; i++)
      {
        int value = 0;
        if (parse_int_(fields[i], &value))
        {
          satellites += value;
        }
      }

      int reported_satellites = 0;
      if (parse_int_(fields.back(), &reported_satellites) && reported_satellites > 0)
      {
        satellites = reported_satellites;
      }

      size_t altitude_index = lon_hemi_index + 3;
      size_t speed_index = lon_hemi_index + 4;
      size_t course_index = lon_hemi_index + 5;
      size_t hdop_index = lon_hemi_index + 7;

      if (altitude_index < fields.size())
      {
        parse_float_(fields[altitude_index], &altitude);
      }
      if (speed_index < fields.size())
      {
        parse_float_(fields[speed_index], &speed_knots);
      }
      if (course_index < fields.size())
      {
        parse_float_(fields[course_index], &course);
      }
      if (hdop_index < fields.size())
      {
        parse_float_(fields[hdop_index], &hdop);
      }

      if (this->fix_mode_sensor_ != nullptr)
      {
        this->fix_mode_sensor_->publish_state(mode);
      }
      if (this->satellites_sensor_ != nullptr)
      {
        this->satellites_sensor_->publish_state(satellites);
      }
      if (this->latitude_sensor_ != nullptr)
      {
        this->latitude_sensor_->publish_state(latitude);
      }
      if (this->longitude_sensor_ != nullptr)
      {
        this->longitude_sensor_->publish_state(longitude);
      }
      if (this->altitude_sensor_ != nullptr)
      {
        this->altitude_sensor_->publish_state(altitude);
      }
      if (this->speed_sensor_ != nullptr)
      {
        this->speed_sensor_->publish_state(speed_knots * 1.852f);
      }
      if (this->course_sensor_ != nullptr)
      {
        this->course_sensor_->publish_state(course);
      }
      if (this->hdop_sensor_ != nullptr)
      {
        this->hdop_sensor_->publish_state(hdop);
      }

      this->mqtt_publish_(this->build_payload_(latitude, longitude, altitude, speed_knots * 1.852f, course, hdop, satellites,
                                               mode));

      ESP_LOGD(TAG, "GNSS fix mode=%d lat=%.6f lon=%.6f sats=%d", mode, latitude, longitude, satellites);
      return true;
    }

    std::string A7670EGNSSComponent::build_payload_(float latitude, float longitude, float altitude, float speed_kmh,
                                                    float course, float hdop, int satellites, int mode)
    {
      char buffer[256];
      snprintf(buffer, sizeof(buffer),
               "{\"lat\":%.6f,\"lon\":%.6f,\"altitude\":%.1f,\"speed\":%.1f,\"course\":%.1f,\"hdop\":%.1f,"
               "\"satellites\":%d,\"fix_mode\":%d}",
               latitude, longitude, altitude, speed_kmh, course, hdop, satellites, mode);
      return std::string(buffer);
    }

    std::vector<std::string> A7670EGNSSComponent::split_csv_(const std::string &line)
    {
      std::vector<std::string> fields;
      std::string field;
      for (char c : line)
      {
        if (c == ',')
        {
          fields.push_back(field);
          field.clear();
        }
        else if (c != ' ')
        {
          field.push_back(c);
        }
      }
      fields.push_back(field);
      return fields;
    }

    bool A7670EGNSSComponent::parse_float_(const std::string &value, float *out)
    {
      if (value.empty())
      {
        return false;
      }
      char *end = nullptr;
      float parsed = strtof(value.c_str(), &end);
      if (end == value.c_str())
      {
        return false;
      }
      *out = parsed;
      return true;
    }

    bool A7670EGNSSComponent::parse_int_(const std::string &value, int *out)
    {
      if (value.empty())
      {
        return false;
      }
      char *end = nullptr;
      long parsed = strtol(value.c_str(), &end, 10);
      if (end == value.c_str())
      {
        return false;
      }
      *out = static_cast<int>(parsed);
      return true;
    }

    bool A7670EGNSSComponent::parse_coordinate_(const std::string &value, const std::string &hemisphere, bool latitude,
                                                float *out)
    {
      float raw = 0.0f;
      if (!parse_float_(value, &raw) || hemisphere.empty())
      {
        return false;
      }

      float coordinate = raw;
      if ((latitude && raw > 90.0f) || (!latitude && raw > 180.0f))
      {
        int degrees = static_cast<int>(raw / 100.0f);
        float minutes = raw - degrees * 100.0f;
        coordinate = degrees + minutes / 60.0f;
      }

      if (hemisphere[0] == 'S' || hemisphere[0] == 'W')
      {
        coordinate = -coordinate;
      }
      *out = coordinate;
      return true;
    }

  } // namespace a7670e_gnss
} // namespace esphome
