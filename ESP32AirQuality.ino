#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "Adafruit_SHT4x.h"

// ====== CONFIG ======
const char* WIFI_SSID     = "SSIDHere";
const char* WIFI_PASSWORD = "passwordhere";

// InfluxDB v2 settings
const char* INFLUX_HOST   = "192.168.1.100";  // IP address of InfluxDB host
const char* INFLUX_ORG    = "YourOrg";         // InfluxDB organization
const char* INFLUX_BUCKET = "YourBucket";      // InfluxDB bucket name
const char* INFLUX_TOKEN  = "YourTokenHere";   // InfluxDB API token
// ====================

// PMS5003 on UART2
HardwareSerial pmsSerial(2);

// SenseAir S8 on UART1
HardwareSerial s8Serial(1);

// SHT41 on I2C
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// ========== PMS5003 Struct ==========
struct __attribute__((packed)) pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um;
  uint16_t particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

pms5003data data;

// ========== Utility ==========
void sendToInflux(String lineProtocol) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String("http://") + INFLUX_HOST + ":8086/api/v2/write?org=" + INFLUX_ORG + "&bucket=" + INFLUX_BUCKET + "&precision=s";
    http.begin(url);
    http.addHeader("Authorization", String("Token ") + INFLUX_TOKEN);
    http.addHeader("Content-Type", "text/plain");

    int code = http.POST(lineProtocol);
    if (code > 0) {
      Serial.printf("InfluxDB response: %d\n", code);
    } else {
      Serial.printf("POST failed: %s\n", http.errorToString(code).c_str());
    }
    http.end();
  }
}

// ========== PMS5003 Parser ==========
boolean readPMSdata(Stream *s) {
  if (!s->available()) return false;

  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
  if (s->available() < 32) return false;

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  for (uint8_t i=0; i<30; i++) sum += buffer[i];

  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  return true;
}

// ========== SenseAir S8 Parser ==========
int readCO2() {
  byte cmd_get_sensor[] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};
  byte response[7];

  while (s8Serial.available()) s8Serial.read(); // flush buffer

  s8Serial.write(cmd_get_sensor, 7);
  delay(50);

  if (s8Serial.available() >= 7) {
    s8Serial.readBytes(response, 7);
    if (response[0] == 0xFE && response[1] == 0x44) {
      int high = response[3];
      int low  = response[4];
      return (high << 8) + low;
    }
  }
  return -1; // error
}

// ========== FreeRTOS Tasks ==========
void pmsTask(void *pvParameters) {
  for (;;) {
    if (readPMSdata(&pmsSerial)) {
      String payload = String("air_quality ") +
        "pm1=" + data.pm10_standard + "," +
        "pm25=" + data.pm25_standard + "," +
        "pm10=" + data.pm100_standard + "," +
        "p_03um=" + data.particles_03um + "," +
        "p_05um=" + data.particles_05um + "," +
        "p_10um=" + data.particles_10um + "," +
        "p_25um=" + data.particles_25um + "," +
        "p_50um=" + data.particles_50um + "," +
        "p_100um=" + data.particles_100um;
      sendToInflux(payload);
      Serial.println("PMS data sent");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void shtTask(void *pvParameters) {
  sensors_event_t humidity, temp;
  for (;;) {
    sht4.getEvent(&humidity, &temp);
    float tempF = temp.temperature * 1.8 + 32;
    String payload = String("environment ") +
      "temperature=" + String(tempF, 1) + "," +
      "humidity=" + String(humidity.relative_humidity, 1);
    sendToInflux(payload);
    Serial.println("SHT41 data sent");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void co2Task(void *pvParameters) {
  for (;;) {
    int co2ppm = readCO2();
    if (co2ppm > 0) {
      String payload = String("air_quality ") + "co2=" + String(co2ppm);
      sendToInflux(payload);
      Serial.printf("CO2: %d ppm\n", co2ppm);
    } else {
      Serial.println("Failed to read CO2");
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // every 2s
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  pmsSerial.begin(9600, SERIAL_8N1, 16, 17);   // PMS5003: RX=16, TX=17
  s8Serial.begin(9600, SERIAL_8N1, 19, 18);    // S8: RX=19, TX=18

  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());

  Wire.begin(21, 22); // SHT41 I2C
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x sensor!");
    while (1) delay(1);
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(pmsTask, "PMSTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(shtTask, "SHTTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(co2Task, "CO2Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Empty
}
