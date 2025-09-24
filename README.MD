# ESP32 Air Quality Monitor

An ESP32-based air quality monitoring system that collects data from multiple sensors and sends it to InfluxDB for visualization in Grafana.

![Air Quality Dashboard](dashboard.png)

## Hardware Components
- ESP32 microcontroller
- PMS5003 particulate matter sensor
- SenseAir S8 CO2 sensor  
- SHT41 temperature/humidity sensor

## Software Stack
- Arduino IDE/ESP32 framework
- InfluxDB v2 (time-series database)
- Grafana (data visualization)
- Docker containers for database and visualization

## Quick Start Guide

### 1. Hardware Wiring

**ESP32 Pinout:**
```
PMS5003 (Particulate Matter Sensor):
- VCC → 5V
- GND → GND
- TX → GPIO 16 (ESP32 RX)
- RX → GPIO 17 (ESP32 TX)

SenseAir S8 (CO2 Sensor):
- VCC → 5V
- GND → GND
- TX → GPIO 19 (ESP32 RX)
- RX → GPIO 18 (ESP32 TX)

SHT41 (Temperature/Humidity Sensor):
- VCC → 3.3V
- GND → GND
- SDA → GPIO 21
- SCL → GPIO 22
```

### 2. Server Setup

**Install Docker:**
- **Docker Desktop:** https://www.docker.com/products/docker-desktop/
- **OR**
- **Rancher Desktop:** https://rancherdesktop.io/

**Start Containers:**
1. Navigate to project directory
These scripts are built for MacOS/Linux. You can modify for windows if you like. If you got this far, I believe in you.
2. Start InfluxDB:
   ```bash
   cd InfluxDB
   ./buildrun.sh
   ```
3. Start Grafana:
   ```bash
   cd ../Grafana
   ./startrun.sh
   ```

### 3. Configure InfluxDB
1. Open http://localhost:8086
2. Create initial user and organization
3. Create bucket named "AQData"
4. Generate API token
5. Note your organization name

### 4. Configure ESP32
1. Install Adafruit SHT4x library in Arduino IDE
2. Update configuration in `ESP32AirQuality.ino`:
   ```cpp
   const char* WIFI_SSID     = "YourWiFiName";
   const char* WIFI_PASSWORD = "YourWiFiPassword";
   const char* INFLUX_HOST   = "192.168.1.100";  // Your server IP
   const char* INFLUX_ORG    = "YourOrg";
   const char* INFLUX_BUCKET = "AQData";
   const char* INFLUX_TOKEN  = "YourTokenHere";
   ```
3. Upload code to ESP32

### 5. Setup Grafana Dashboard
1. Open http://localhost:3000
2. Login (admin/admin)
3. Add InfluxDB data source:
   - URL: http://localhost:8086
   - Organization: Your org name
   - Token: Your API token
   - Default bucket: AQData
4. Import dashboard: `Grafana/Enviro+ (InfluxDB)-1758748677774.json`

## Third-Party Code and Libraries

### Arduino Libraries
- **Adafruit SHT4x Library** - Used for SHT41 temperature/humidity sensor communication
  - Repository: https://github.com/adafruit/Adafruit_SHT4X
  - License: MIT License
  - Copyright: Adafruit Industries

### ESP32 Core Libraries (Built-in)
- **WiFi.h** - ESP32 WiFi connectivity
- **HTTPClient.h** - HTTP client for API requests
- **HardwareSerial.h** - UART communication
- **Wire.h** - I2C communication

### Docker Images
- **InfluxDB Official Image** - `influxdb:2.7`
  - Source: https://hub.docker.com/_/influxdb
  - License: MIT License
  - Maintained by: InfluxData

- **Grafana Official Image** - `grafana/grafana:latest`
  - Source: https://hub.docker.com/r/grafana/grafana
  - License: AGPL v3
  - Maintained by: Grafana Labs

### Sensor Communication Protocols
- **PMS5003 Protocol Implementation** - Custom implementation based on https://instesre.org/ParticulateSensorKit/ParticulateSensorKit.htm#:~:text=You%20can%20download%20the%20Arduino,stores%20PMS5003%20and%20DHT22%20data. This tutorial I was able to get the PMS5003 to work adapting for ESP32.
- **SenseAir S8 Protocol Implementation** - ChatGPT lol
- **SHT4x Implementation** - ChatGPT and following documentation https://cdn-learn.adafruit.com/downloads/pdf/adafruit-sht40-temperature-humidity-sensor.pdf

### Credits
Thanks to KindOfOK for publishing Enviro + grafana dashboard I used as a template for my project. My dashboard removes the heatmaps and pressure sensing, and adds CO2 monitoring.
https://grafana.com/grafana/dashboards/13885-enviro/

