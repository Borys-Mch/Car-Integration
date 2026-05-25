/*
 * Car Telemetry + Barrier Call
 * ESP32-C6 + ELM327 (UART0) + A7670E (UART1)
 *
 * Бібліотеки: ArduinoJson by Benoit Blanchon
 *
 * Arduino IDE → Tools → USB CDC On Boot: Enabled
 */

#if ARDUINO_USB_CDC_ON_BOOT
  #define DBG Serial
#else
  #define DBG Serial0
#endif

#include <ArduinoJson.h>
#include "secrets.h"

// ===== СТРУКТУРИ =============================================
struct GNSSData {
    float lat, lon, alt, speed, course, hdop;
    int satellites, fixMode;
    bool valid;
};


// ===== СТРУКТУРИ =============================================
// ===== ПІНИ ==================================================
#define ELM_RX_PIN      17
#define ELM_TX_PIN      16
#define ELM_BAUD        38400

#define GSM_RX_PIN      20    // з YAML uart_modem
#define GSM_TX_PIN      21
#define GSM_BAUD        115200

#define BARRIER_BTN_PIN 10    // кнопка на GND

// ===== НАЛАШТУВАННЯ ==========================================
#define BARRIER_CALL_MS  10000   // з YAML: delay 10s

#define POLL_FAST_MS     2000
#define POLL_SLOW_MS     10000
#define GNSS_MS          30000   // з YAML: update_interval 30s
#define MQTT_RETRY_MS    15000

#define MQTT_TOPIC_STATE    "car/forester/state"
#define MQTT_TOPIC_CMD      "car/forester/cmd"
#define MQTT_CLIENT_ID      "car-integration"   // з YAML

// ===== UART ==================================================
HardwareSerial SerialELM(0);
HardwareSerial SerialGSM(1);

// ===== СТАН ==================================================
bool modemReady    = false;
bool mqttStarted   = false;
bool mqttConnected = false;
bool gnssReady     = false;
uint8_t mqttFailures = 0;

uint32_t lastFast     = 0;
uint32_t lastSlow     = 0;
uint32_t lastGNSS     = 0;
uint32_t lastMqttRetry = 0;

bool lastBtnState = HIGH;

// ===== AT КОМАНДИ ============================================

void gsmFlush() {
    while (SerialGSM.available()) SerialGSM.read();
}

// Відправити AT команду, повернути відповідь
String gsmCmd(const char* cmd, uint32_t timeout = 5000) {
    gsmFlush();
    SerialGSM.print(cmd);
    SerialGSM.print('\r');

    String resp = "";
    uint32_t t = millis();
    while (millis() - t < timeout) {
        while (SerialGSM.available()) {
            char c = (char)SerialGSM.read();
            if (c == '>') { resp += c; return resp; }
            if (c != '\r' && c != '\n') resp += c;
        }
        if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) break;
        delay(5);
    }
    return resp;
}

bool gsmOK(const char* cmd, uint32_t timeout = 5000) {
    String r = gsmCmd(cmd, timeout);
    DBG.printf("  %s -> %s\n", cmd, r.c_str());
    return r.indexOf("OK") >= 0;
}

// Відправити дані після prompt ">"
bool gsmPrompt(const String& data, uint32_t timeout = 8000) {
    SerialGSM.print(data);
    String resp = "";
    uint32_t t = millis();
    while (millis() - t < timeout) {
        while (SerialGSM.available()) {
            char c = (char)SerialGSM.read();
            if (c != '\r' && c != '\n') resp += c;
        }
        if (resp.indexOf("OK") >= 0) return true;
        if (resp.indexOf("ERROR") >= 0) return false;
        delay(5);
    }
    return false;
}

// ===== ІНІЦІАЛІЗАЦІЯ МОДЕМА ==================================

bool modemInit() {
    DBG.println("[GSM] Init...");

    // Echo off
    gsmCmd("ATE0", 3000);

    // Перевірити SIM
    String sim = gsmCmd("AT+CPIN?", 5000);
    DBG.printf("[GSM] SIM: %s\n", sim.c_str());
    if (sim.indexOf("READY") < 0) {
        DBG.println("[GSM] SIM not ready!");
        return false;
    }

    // Сигнал
    String csq = gsmCmd("AT+CSQ", 3000);
    DBG.printf("[GSM] Signal: %s\n", csq.c_str());

    // Реєстрація в мережі
    DBG.print("[GSM] Waiting network");
    uint32_t t = millis();
    while (millis() - t < 60000) {
        String creg = gsmCmd("AT+CREG?", 3000);
        // +CREG: 0,1 або +CREG: 0,5 = зареєстровано
        if (creg.indexOf(",1") >= 0 || creg.indexOf(",5") >= 0) {
            DBG.println(" OK");
            break;
        }
        DBG.print(".");
        delay(2000);
    }

    // Налаштувати APN і активувати PDP
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN);
    if (!gsmOK(cmd, 8000)) return false;
    gsmOK("AT+CGACT=1,1", 30000);

    modemReady = true;
    DBG.println("[GSM] Ready");
    return true;
}

// ===== GNSS ==================================================

bool gnssInit() {
    DBG.println("[GNSS] Init...");
    gsmOK("AT+CGNSSPWR=1", 9000);
    gsmOK("AT+CGNSSINFO=0", 9000);
    gnssReady = true;
    DBG.println("[GNSS] Ready");
    return true;
}

GNSSData getGNSS() {
    GNSSData d = {};
    String resp = gsmCmd("AT+CGNSSINFO", 9000);
    DBG.printf("[GNSS] %s\n", resp.c_str());

    // Парсинг +CGNSSINFO: mode,sats_gps,sats_glonass,sats_beidou,lat,N/S,lon,E/W,date,time,alt,speed,course,,,hdop,...
    int idx = resp.indexOf("+CGNSSINFO:");
    if (idx < 0) return d;

    String data = resp.substring(idx + 11);
    // Розбити на поля
    String fields[20];
    int count = 0, start = 0;
    for (int i = 0; i < data.length() && count < 20; i++) {
        if (data[i] == ',') {
            fields[count++] = data.substring(start, i);
            fields[count-1].trim();
            start = i + 1;
        }
    }
    fields[count++] = data.substring(start);

    if (count < 13) return d;

    int mode = fields[0].toInt();
    if (mode < 2) return d;  // немає фіксу

    d.fixMode = mode;

    // Знайти індекс N/S (latitude hemisphere)
    int latHemi = -1;
    for (int i = 1; i < count; i++) {
        if (fields[i] == "N" || fields[i] == "S") { latHemi = i; break; }
    }
    if (latHemi < 1 || latHemi + 2 >= count) return d;

    // Парсинг координат (формат DDMM.MMMM)
    auto parseCoord = [](const String& val, const String& hemi, bool isLat, float& out) {
        float raw = val.toFloat();
        if (raw == 0) return false;
        int deg = (int)(raw / 100);
        float min = raw - deg * 100;
        out = deg + min / 60.0f;
        if (hemi == "S" || hemi == "W") out = -out;
        return true;
    };

    if (!parseCoord(fields[latHemi-1], fields[latHemi], true, d.lat)) return d;
    if (!parseCoord(fields[latHemi+1], fields[latHemi+2], false, d.lon)) return d;

    // Сателіти (сума GPS+GLONASS+BEIDOU)
    for (int i = 1; i < latHemi - 1; i++) d.satellites += fields[i].toInt();

    int altIdx   = latHemi + 3;
    int spdIdx   = latHemi + 4;
    int crsIdx   = latHemi + 5;
    int hdopIdx  = latHemi + 7;

    if (altIdx  < count) d.alt    = fields[altIdx].toFloat();
    if (spdIdx  < count) d.speed  = fields[spdIdx].toFloat() * 1.852f; // вузли → км/год
    if (crsIdx  < count) d.course = fields[crsIdx].toFloat();
    if (hdopIdx < count) d.hdop   = fields[hdopIdx].toFloat();

    d.valid = true;
    return d;
}

// ===== MQTT через AT+CMQTT ==================================

bool mqttConnect() {
    if (!modemReady) return false;
    DBG.printf("[MQTT] Connecting to %s:%d...\n", MQTT_HOST, MQTT_PORT);

    // Перевірки мережі
    gsmOK("AT+CREG?", 5000);
    gsmOK("AT+CEREG?", 5000);

    // Повний скид MQTT стеку перед кожним підключенням
    gsmCmd("AT+CMQTTDISC=0,10", 5000);
    gsmCmd("AT+CMQTTREL=0", 5000);
    gsmCmd("AT+CMQTTSTOP", 10000);
    mqttStarted = false;
    mqttConnected = false;
    delay(2000);

    // Запустити стек чисто
    {
        String r = gsmCmd("AT+CMQTTSTART", 30000);
        DBG.printf("  CMQTTSTART -> %s\n", r.c_str());
        if (r.indexOf("OK") < 0 && r.indexOf("+CMQTTSTART: 0") < 0) {
            // Retry
            delay(1000);
            r = gsmCmd("AT+CMQTTSTART", 30000);
            DBG.printf("  CMQTTSTART retry -> %s\n", r.c_str());
            if (r.indexOf("OK") < 0 && r.indexOf("+CMQTTSTART: 0") < 0) {
                mqttFailures++;
                return false;
            }
        }
        mqttStarted = true;
        delay(500);
    }

    // Звільнити клієнт якщо зайнятий
    gsmCmd("AT+CMQTTREL=0", 5000);

    // Створити клієнта
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\",%d",
             MQTT_CLIENT_ID, MQTT_USE_TLS ? 1 : 0);
    if (!gsmOK(cmd, 8000)) { mqttFailures++; return false; }

    // TLS налаштування
    if (MQTT_USE_TLS) {
        gsmOK("AT+CSSLCFG=\"sslversion\",0,3", 8000);
        gsmOK("AT+CSSLCFG=\"authmode\",0,0", 8000);
        gsmOK("AT+CSSLCFG=\"ignorlocaltime\",0,1", 8000);
        if (!gsmOK("AT+CMQTTSSLCFG=0,0", 8000)) { mqttFailures++; return false; }
    }

    // Підключитись
    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,1,\"%s\",\"%s\"",
             MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
    String r = gsmCmd(cmd, 60000);
    DBG.printf("  CMQTTCONNECT -> %s\n", r.c_str());

    if (r.indexOf("+CMQTTCONNECT: 0,0") < 0 && r.indexOf("OK") < 0) {
        mqttFailures++;
        mqttConnected = false;
        return false;
    }

    mqttFailures = 0;
    mqttConnected = true;
    DBG.println("[MQTT] Connected!");

    // Підписатись на команди
    String subResp = gsmCmd("AT+CMQTTSUB=0,\"car/forester/cmd\",1", 10000);
    DBG.printf("  CMQTTSUB -> %s\n", subResp.c_str());

    delay(2000);
    publishDiscovery();
    return true;
}

bool mqttPublish(const String& payload) {
    if (!mqttConnected) {
        if (millis() - lastMqttRetry > MQTT_RETRY_MS) {
            lastMqttRetry = millis();
            mqttConnect();
        }
        return false;
    }

    // Встановити топік
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", strlen(MQTT_TOPIC_STATE));
    String r = gsmCmd(cmd, 8000);
    if (r.indexOf(">") < 0 && r.indexOf("OK") < 0) { mqttConnected = false; return false; }
    if (!gsmPrompt(MQTT_TOPIC_STATE)) { mqttConnected = false; return false; }

    // Встановити payload
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", payload.length());
    r = gsmCmd(cmd, 8000);
    if (r.indexOf(">") < 0 && r.indexOf("OK") < 0) { mqttConnected = false; return false; }
    if (!gsmPrompt(payload)) { mqttConnected = false; return false; }

    // Публікувати
    r = gsmCmd("AT+CMQTTPUB=0,1,60", 30000);
    if (r.indexOf("+CMQTTPUB: 0,0") < 0 && r.indexOf("OK") < 0) {
        mqttConnected = false;
        return false;
    }

    DBG.printf("[MQTT] Published: %s\n", payload.c_str());
    return true;
}

// ===== ELM327 ================================================

String elmCmd(const String& cmd, uint16_t timeout = 1500) {
    while (SerialELM.available()) SerialELM.read();
    SerialELM.print(cmd);
    SerialELM.print('\r');
    String resp = "";
    uint32_t t = millis();
    while (millis() - t < timeout) {
        while (SerialELM.available()) {
            char c = SerialELM.read();
            if (c == '>') goto done;
            if (c != '\r') resp += c;
        }
    }
    done:
    resp.trim();
    return resp;
}

bool elmInit() {
    DBG.println("[ELM] Init...");
    elmCmd("ATZ",  2000);
    elmCmd("ATE0"); elmCmd("ATL0"); elmCmd("ATS0"); elmCmd("ATH0");
    elmCmd("ATSP0");
    String id = elmCmd("ATI");
    DBG.printf("[ELM] ID: %s\n", id.c_str());
    return id.length() > 0;
}

int elmPID(const String& pid, int bytes = 1) {
    String resp = elmCmd("01" + pid);
    if (resp.length() == 0 || resp.indexOf("NO DATA") >= 0 ||
        resp.indexOf("STOPPED") >= 0 || resp.indexOf("ERROR") >= 0) return -1;
    String search = "41" + pid; search.toUpperCase(); resp.toUpperCase();
    int idx = resp.indexOf(search);
    if (idx < 0) return -1;
    String data = resp.substring(idx + search.length());
    data.trim(); data.replace(" ", "");
    if (data.length() < (unsigned)(2 * bytes)) return -1;
    if (bytes == 1) return strtol(data.substring(0, 2).c_str(), nullptr, 16);
    int A = strtol(data.substring(0, 2).c_str(), nullptr, 16);
    int B = strtol(data.substring(2, 4).c_str(), nullptr, 16);
    return A * 256 + B;
}

int   getRPM()      { int v = elmPID("0C", 2); return v < 0 ? -1 : v / 4; }
int   getSpeed()    { return elmPID("0D"); }
int   getCoolant()  { int v = elmPID("05"); return v < 0 ? -999 : v - 40; }
float getThrottle() { int v = elmPID("11"); return v < 0 ? -1 : v * 100.0f / 255.0f; }
float getFuel()     { int v = elmPID("2F"); return v < 0 ? -1 : v * 100.0f / 255.0f; }
float getVoltage()  { String r = elmCmd("ATRV"); r.replace("V",""); r.trim(); return r.toFloat(); }
int   getIntakeTemp(){ int v = elmPID("0F"); return v < 0 ? -999 : v - 40; }

// ===== ШЛАГБАУМ ==============================================

void callBarrier() {
    DBG.println("[BARRIER] Calling...");

    // Відключити MQTT на час дзвінка
    gsmCmd("AT+CMQTTDISC=0,60", 5000);
    mqttConnected = false;
    delay(500);

    // Дзвонимо — шлагбаум сам скине дзвінок
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "ATD%s;", BARRIER_NUMBER);
    String r = gsmCmd(cmd, 3000);
    DBG.printf("[BARRIER] %s -> %s\n", cmd, r.c_str());

    // Чекаємо відповідь модема (NO CARRIER = скинули, або чекаємо 30с макс)
    uint32_t t = millis();
    while (millis() - t < 30000) {
        if (SerialGSM.available()) {
            String resp = SerialGSM.readStringUntil('\n');
            resp.trim();
            DBG.printf("[BARRIER] modem: %s\n", resp.c_str());
            if (resp.indexOf("NO CARRIER") >= 0 ||
                resp.indexOf("BUSY") >= 0 ||
                resp.indexOf("NO ANSWER") >= 0) {
                DBG.println("[BARRIER] Call ended by remote");
                break;
            }
        }
        delay(100);
    }

    // На всяк випадок покласти трубку
    gsmOK("ATH", 3000);
    DBG.println("[BARRIER] Done");

    // Відновити MQTT
    delay(1000);
    mqttConnect();
}


// ===== HA MQTT DISCOVERY =====================================

bool mqttPublishRaw(const String& topic, const String& payload, bool retain = true) {
    DBG.printf("  [RAW] topic=%s len=%d\n", topic.c_str(), payload.length());

    // Встановити топік
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", topic.length());
    String r = gsmCmd(cmd, 8000);
    DBG.printf("  CMQTTTOPIC -> %s\n", r.c_str());
    if (r.indexOf(">") < 0 && r.indexOf("OK") < 0) { mqttConnected = false; return false; }
    if (!gsmPrompt(topic)) {
        DBG.println("  gsmPrompt(topic) FAILED");
        mqttConnected = false; return false;
    }

    // Встановити payload
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", payload.length());
    r = gsmCmd(cmd, 8000);
    DBG.printf("  CMQTTPAYLOAD -> %s\n", r.c_str());
    if (r.indexOf(">") < 0 && r.indexOf("OK") < 0) { mqttConnected = false; return false; }
    if (!gsmPrompt(payload)) {
        DBG.println("  gsmPrompt(payload) FAILED");
        mqttConnected = false; return false;
    }

    // Публікувати QoS 1
    // retain передається як окремий параметр тільки якщо підтримується
    String pubCmd = retain ? "AT+CMQTTPUB=0,1,60,1" : "AT+CMQTTPUB=0,1,60,0";
    r = gsmCmd(pubCmd.c_str(), 30000);
    DBG.printf("  CMQTTPUB -> %s\n", r.c_str());
    // Якщо ERROR — спробувати без retain параметра
    if (r.indexOf("ERROR") >= 0) {
        r = gsmCmd("AT+CMQTTPUB=0,1,60", 30000);
        DBG.printf("  CMQTTPUB(no retain) -> %s\n", r.c_str());
    }
    bool ok = r.indexOf("+CMQTTPUB: 0,0") >= 0 || r.indexOf("OK") >= 0;
    if (!ok) mqttConnected = false;
    return ok;
}

void publishDiscovery() {
    if (!mqttConnected) return;
    DBG.println("[HA] Publishing discovery...");

    // Генеруємо кожен discovery конфіг через ArduinoJson
    auto sendSensor = [&](const char* uid, const char* name, const char* valTpl,
                          const char* unit, const char* icon, const char* devClass = "") -> bool {
        JsonDocument doc;
        doc["name"]                  = name;
        doc["unique_id"]             = uid;
        doc["state_topic"]           = "car/forester/state";
        doc["value_template"]        = valTpl;
        doc["unit_of_measurement"]   = unit;
        doc["icon"]                  = icon;
        if (strlen(devClass) > 0) doc["device_class"] = devClass;
        doc["availability_topic"]    = "car/forester/status";
        doc["payload_available"]     = "online";
        doc["payload_not_available"] = "offline";
        JsonObject dev = doc["device"].to<JsonObject>();
        dev["identifiers"][0]        = "car_obd_01";
        dev["name"]                  = "Car Forester";
        dev["model"]                 = "ESP32-C6+ELM327+A7670E";
        dev["manufacturer"]          = "DIY";

        String topic = String("homeassistant/sensor/") + uid + "/config";
        String payload;
        serializeJson(doc, payload);
        DBG.printf("  [DISC] %s\n", topic.c_str());
        delay(300);
        return mqttPublishRaw(topic, payload, true);
    };

    sendSensor("car_rpm",      "Car RPM",        "{{ value_json.rpm }}",          "rpm",  "mdi:engine");
    sendSensor("car_speed",    "Car Speed",       "{{ value_json.speed_obd }}",    "km/h", "mdi:speedometer",     "speed");
    sendSensor("car_coolant",  "Car Coolant",     "{{ value_json.coolant_temp }}", "°C",   "mdi:thermometer",     "temperature");
    sendSensor("car_throttle", "Car Throttle",    "{{ value_json.throttle }}",     "%",    "mdi:car-turbocharger");
    sendSensor("car_fuel",     "Car Fuel",        "{{ value_json.fuel }}",         "%",    "mdi:fuel");
    sendSensor("car_voltage",  "Car Voltage",     "{{ value_json.voltage }}",      "V",    "mdi:car-battery",     "voltage");
    sendSensor("car_intake",   "Car Intake Temp", "{{ value_json.intake_temp }}",  "°C",   "mdi:thermometer",     "temperature");
    sendSensor("car_sats",     "Car Satellites",  "{{ value_json.satellites }}",   "",     "mdi:satellite-variant");
    sendSensor("car_gspeed",   "Car GPS Speed",   "{{ value_json.speed }}",        "km/h", "mdi:speedometer",     "speed");

    // Кнопка шлагбауму
    {
        JsonDocument doc;
        doc["name"]           = "Open Gate";
        doc["unique_id"]      = "car_barrier_btn";
        doc["command_topic"]  = "car/forester/cmd";
        doc["payload_press"]  = "OPEN";
        doc["icon"]           = "mdi:boom-gate-up";
        doc["availability_topic"]    = "car/forester/status";
        doc["payload_available"]     = "online";
        doc["payload_not_available"] = "offline";
        JsonObject dev = doc["device"].to<JsonObject>();
        dev["identifiers"][0] = "car_obd_01";
        dev["name"]           = "Car Forester";
        String payload;
        serializeJson(doc, payload);
        delay(300);
        mqttPublishRaw("homeassistant/button/car_barrier_btn/config", payload, true);
        DBG.println("  [DISC] barrier button");
    }

    // Статус online з retain
    delay(500);
    mqttPublishRaw("car/forester/status", "online", true);

    DBG.println("[HA] Discovery done");
}

// ===== ПОБУДОВА JSON PAYLOAD =================================

String buildPayload(int rpm, int spd, int coolant, float throttle,
                    float fuel, float voltage, int intakeTemp,
                    const GNSSData& gnss) {
    JsonDocument doc;

    // Завжди включати timestamp щоб payload не був null
    doc["ts"] = millis() / 1000;

    // OBD дані (null якщо недоступні — авто не підключено)
    doc["rpm"]          = (rpm      >= 0)    ? rpm      : (int)-1;
    doc["speed_obd"]    = (spd      >= 0)    ? spd      : (int)-1;
    doc["coolant_temp"] = (coolant  > -999)  ? coolant  : (int)-999;
    doc["throttle"]     = (throttle >= 0)    ? throttle : -1.0f;
    doc["fuel"]         = (fuel     >= 0)    ? fuel     : -1.0f;
    doc["voltage"]      = (voltage  > 0)     ? voltage  :  0.0f;
    doc["intake_temp"]  = (intakeTemp > -999)? intakeTemp: (int)-999;

    // GPS
    if (gnss.valid) {
        doc["lat"]        = gnss.lat;
        doc["lon"]        = gnss.lon;
        doc["altitude"]   = gnss.alt;
        doc["speed"]      = gnss.speed;
        doc["course"]     = gnss.course;
        doc["hdop"]       = gnss.hdop;
        doc["satellites"] = gnss.satellites;
        doc["fix_mode"]   = gnss.fixMode;
    } else {
        doc["fix_mode"]   = 0;
    }

    String out;
    serializeJson(doc, out);
    DBG.printf("[PAYLOAD] %s\n", out.c_str());
    return out;
}

// ===== КЕШ ДАНИХ =============================================
int   c_rpm = -1, c_spd = -1, c_coolant = -999, c_intakeTemp = -999;
float c_throttle = -1, c_fuel = -1, c_voltage = 0;
GNSSData c_gnss = {};

// ===== SETUP =================================================

void setup() {
    DBG.begin(115200);
    delay(1000);
    DBG.println("\n=== Car Telemetry + Barrier ===");

    pinMode(BARRIER_BTN_PIN, INPUT_PULLUP);

    // ELM327
    SerialELM.begin(ELM_BAUD, SERIAL_8N1, ELM_RX_PIN, ELM_TX_PIN);
    delay(500);
    elmInit();

    // A7670E
    SerialGSM.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(1000);

    while (!modemInit()) {
        DBG.println("[GSM] Retry in 10s...");
        delay(10000);
    }

    gnssInit();
    mqttConnect();

    DBG.println("[INIT] Ready!");
}

// ===== LOOP ==================================================

void loop() {
    // Читати URC від модема посимвольно в буфер
    static String urcBuf = "";
    while (SerialGSM.available()) {
        char c = (char)SerialGSM.read();
        if (c == '\n') {
            urcBuf.trim();
            if (urcBuf.length() > 0) {
                DBG.printf("[URC] %s\n", urcBuf.c_str());
                // Перевірити команду відкриття шлагбауму
                if (urcBuf == "OPEN" || urcBuf.indexOf("OPEN") >= 0) {
                    DBG.println("[CMD] OPEN received!");
                    callBarrier();
                }
            }
            urcBuf = "";
        } else if (c != '\r') {
            urcBuf += c;
        }
    }

    // Фізична кнопка
    bool btn = digitalRead(BARRIER_BTN_PIN);
    if (btn == LOW && lastBtnState == HIGH) {
        delay(50);
        if (digitalRead(BARRIER_BTN_PIN) == LOW) {
            DBG.println("[BTN] Pressed!");
            callBarrier();
        }
    }
    lastBtnState = btn;

    uint32_t now = millis();

    // Швидкі OBD дані
    if (now - lastFast >= POLL_FAST_MS) {
        lastFast = now;
        c_rpm      = getRPM();
        c_spd      = getSpeed();
        c_throttle = getThrottle();
        if (c_rpm >= 0) DBG.printf("[OBD] RPM:%d Speed:%d\n", c_rpm, c_spd);
    }

    // Повільні OBD дані
    if (now - lastSlow >= POLL_SLOW_MS) {
        lastSlow = now;
        c_coolant   = getCoolant();
        c_fuel      = getFuel();
        c_voltage   = getVoltage();
        c_intakeTemp = getIntakeTemp();
        DBG.printf("[OBD] Coolant:%d Fuel:%.1f Volt:%.2f\n", c_coolant, c_fuel, c_voltage);
    }

    // GNSS + публікація
    if (now - lastGNSS >= GNSS_MS) {
        lastGNSS = now;
        c_gnss = getGNSS();
        if (c_gnss.valid)
            DBG.printf("[GNSS] %.6f,%.6f sats:%d\n", c_gnss.lat, c_gnss.lon, c_gnss.satellites);

        // Публікуємо всі дані разом одним JSON
        String payload = buildPayload(c_rpm, c_spd, c_coolant, c_throttle,
                                      c_fuel, c_voltage, c_intakeTemp, c_gnss);
        if (!mqttPublish(payload)) {
            DBG.println("[MQTT] Publish failed, reconnecting...");
            if (millis() - lastMqttRetry > MQTT_RETRY_MS) {
                lastMqttRetry = millis();
                mqttConnect();
            }
        }
    }
}
