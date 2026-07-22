#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <DHT.h>

// ---- WiFi: join your phone's hotspot so the phone keeps internet for map tiles ----
const char* WIFI_SSID = "YOUR_PHONE_HOTSPOT_NAME";
const char* WIFI_PASS = "YOUR_PHONE_HOTSPOT_PASSWORD";

WebServer server(80);

// ---- Pixhawk TELEM1 (UART1) ----
#define TELEM_RX_PIN  13   // Pixhawk TELEM1 TX -> here
#define TELEM_TX_PIN  14   // Pixhawk TELEM1 RX -> here (heartbeat/stream requests)
#define TELEM_BAUD    57600
HardwareSerial mavSerial(1);

// ---- PMS5003 (UART2, RX only) ----
#define PMS_RX_PIN 16
HardwareSerial pmsSerial(2);

// ---- MQ-3 (analog, ADC1) ----
#define MQ3_PIN 36

// ---- DHT11 ----
#define DHT_PIN  25
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// ---- MAVLink definitions (same approach as your confirmed-working reference) ----
#define MAVLINK_STX        0xFE
#define MAVLINK_HEADER     6
#define MSGID_HEARTBEAT    0
#define MSGID_GPS_RAW_INT  24
#define MSGID_GLOBAL_POSITION_INT 33
#define MSGID_REQUEST_DATA_STREAM 66
#define REQUEST_DATA_STREAM_CRC_EXTRA 148

#define MAV_DATA_STREAM_RAW_SENSORS     1
#define MAV_DATA_STREAM_EXTENDED_STATUS 2
#define MAV_DATA_STREAM_POSITION        6

uint8_t mavFrame[300];
uint8_t mavIdx = 0;
bool    mavReceiving = false;
uint8_t mavPayloadLen = 0;
uint8_t mavSeq = 0;
unsigned long lastStreamRequest = 0;

// ---- Live GPS state ----
double  currentLat = 0.0;
double  currentLon = 0.0;
float   currentAlt = 0.0;
uint8_t gpsFix     = 0;
uint8_t satCount   = 0;

// ---- Live sensor state ----
uint16_t pm25_val = 0;
uint16_t pm10_val = 0;
bool     pmsDataValid = false;

int   mq3_raw = 0;
float mq3_voltage = 0.0;

float temp_c = 0.0;
float humidity = 0.0;
bool  dhtValid = false;

// ---------------------------------------------------------------
// MAVLink CRC + stream request (identical approach to your working reference)
// ---------------------------------------------------------------
uint16_t crcAccumulate(uint8_t data, uint16_t crcAccum) {
  uint8_t tmp = data ^ (uint8_t)(crcAccum & 0xFF);
  tmp ^= (tmp << 4);
  crcAccum = (crcAccum >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
  return crcAccum;
}

void requestDataStream(uint8_t streamId, uint16_t rateHz) {
  uint8_t targetSystem = 1;
  uint8_t targetComponent = 1;
  uint8_t mySysId = 255;
  uint8_t myCompId = 0;

  uint8_t payload[6];
  payload[0] = rateHz & 0xFF;
  payload[1] = (rateHz >> 8) & 0xFF;
  payload[2] = targetSystem;
  payload[3] = targetComponent;
  payload[4] = streamId;
  payload[5] = 1; // start

  uint8_t len = 6;
  uint8_t header[5] = { len, mavSeq++, mySysId, myCompId, MSGID_REQUEST_DATA_STREAM };

  uint16_t crc = 0xFFFF;
  for (int i = 0; i < 5; i++) crc = crcAccumulate(header[i], crc);
  for (int i = 0; i < len; i++) crc = crcAccumulate(payload[i], crc);
  crc = crcAccumulate(REQUEST_DATA_STREAM_CRC_EXTRA, crc);

  mavSerial.write(MAVLINK_STX);
  mavSerial.write(header, 5);
  mavSerial.write(payload, len);
  mavSerial.write((uint8_t)(crc & 0xFF));
  mavSerial.write((uint8_t)((crc >> 8) & 0xFF));
}

void requestAllStreams() {
  requestDataStream(MAV_DATA_STREAM_RAW_SENSORS, 10);
  requestDataStream(MAV_DATA_STREAM_EXTENDED_STATUS, 5);
  requestDataStream(MAV_DATA_STREAM_POSITION, 5);
}

// ---------------------------------------------------------------
// MAVLink frame parsing (identical approach to your working reference)
// ---------------------------------------------------------------
void handleMavFrame() {
  uint8_t msgId = mavFrame[5];
  uint8_t *p = &mavFrame[6];

  if (msgId == MSGID_GPS_RAW_INT) {
    int32_t lat, lon;
    memcpy(&lat, p + 8, 4);
    memcpy(&lon, p + 12, 4);
    gpsFix   = p[28];
    satCount = p[29];

    // Use raw GPS as a fallback before EKF position (GLOBAL_POSITION_INT) is available
    if (currentLat == 0.0 && currentLon == 0.0) {
      currentLat = lat / 1e7;
      currentLon = lon / 1e7;
    }
  }
  else if (msgId == MSGID_GLOBAL_POSITION_INT) {
    int32_t lat, lon, relAlt;
    memcpy(&lat,    p + 4,  4);
    memcpy(&lon,    p + 8,  4);
    memcpy(&relAlt, p + 16, 4);

    currentLat = lat / 1e7;
    currentLon = lon / 1e7;
    currentAlt = relAlt / 1000.0;
  }
  else if (msgId == MSGID_HEARTBEAT) {
    // link alive
  }
}

void processMavByte(uint8_t b) {
  if (!mavReceiving) {
    if (b == MAVLINK_STX) {
      mavReceiving = true;
      mavIdx = 0;
      mavFrame[mavIdx++] = b;
    }
    return;
  }

  mavFrame[mavIdx++] = b;
  if (mavIdx == 2) mavPayloadLen = b;

  uint16_t totalLen = MAVLINK_HEADER + mavPayloadLen + 2;
  if (mavIdx >= MAVLINK_HEADER && mavIdx >= totalLen) {
    handleMavFrame();
    mavReceiving = false;
    mavIdx = 0;
  }

  if (mavIdx >= sizeof(mavFrame)) {
    mavReceiving = false;
    mavIdx = 0;
  }
}

// ---------------------------------------------------------------
// PMS5003 (PM2.5 / PM10)
// ---------------------------------------------------------------
void readPMS5003() {
  static uint8_t buf[32];
  static uint8_t idx = 0;

  while (pmsSerial.available()) {
    uint8_t b = pmsSerial.read();

    if (idx == 0 && b != 0x42) continue;
    if (idx == 1 && b != 0x4D) { idx = 0; continue; }

    buf[idx++] = b;

    if (idx >= 32) {
      uint16_t sum = 0;
      for (int i = 0; i < 30; i++) sum += buf[i];
      uint16_t checksum = (buf[30] << 8) | buf[31];

      if (sum == checksum) {
        pm25_val = (buf[12] << 8) | buf[13];
        pm10_val = (buf[14] << 8) | buf[15];
        pmsDataValid = true;
      }
      idx = 0;
    }
  }
}

// ---------------------------------------------------------------
// MQ-3 (analog)
// ---------------------------------------------------------------
void readMQ3() {
  mq3_raw = analogRead(MQ3_PIN);
  mq3_voltage = (mq3_raw / 4095.0) * 3.3;
}

// ---------------------------------------------------------------
// DHT11
// ---------------------------------------------------------------
void readDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h)) {
    temp_c   = t;
    humidity = h;
    dhtValid = true;
  } else {
    dhtValid = false;
  }
}

// ---------------------------------------------------------------
// Custom AQI
// ---------------------------------------------------------------
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float pm25Subindex(float pm25) {
  if (pm25 <= 12.0)   return fmap(pm25, 0, 12, 0, 50);
  if (pm25 <= 35.4)   return fmap(pm25, 12.1, 35.4, 51, 100);
  if (pm25 <= 55.4)   return fmap(pm25, 35.5, 55.4, 101, 150);
  if (pm25 <= 150.4)  return fmap(pm25, 55.5, 150.4, 151, 200);
  if (pm25 <= 250.4)  return fmap(pm25, 150.5, 250.4, 201, 300);
  return fmap(min(pm25, 500.0f), 250.5, 500, 301, 500);
}

float pm10Subindex(float pm10) {
  if (pm10 <= 54)    return fmap(pm10, 0, 54, 0, 50);
  if (pm10 <= 154)   return fmap(pm10, 55, 154, 51, 100);
  if (pm10 <= 254)   return fmap(pm10, 155, 254, 101, 150);
  if (pm10 <= 354)   return fmap(pm10, 255, 354, 151, 200);
  if (pm10 <= 424)   return fmap(pm10, 355, 424, 201, 300);
  return fmap(min(pm10, 604.0f), 425, 604, 301, 500);
}

float gasSubindex(float voltage) {
  const float GAS_BASELINE = 0.3;
  const float GAS_SPAN     = 2.5;
  float v = voltage - GAS_BASELINE;
  if (v < 0) v = 0;
  float idx = (v / GAS_SPAN) * 300.0;
  return constrain(idx, 0, 300);
}

float comfortPenalty(float t, float h) {
  float penalty = 0;
  if (t > 35) penalty += (t - 35) * 2;
  if (h > 80) penalty += (h - 80) * 1;
  return constrain(penalty, 0, 30);
}

float calculateAQI() {
  float pm25Sub = pmsDataValid ? pm25Subindex(pm25_val) : 0;
  float pm10Sub = pmsDataValid ? pm10Subindex(pm10_val) : 0;
  float gasSub  = gasSubindex(mq3_voltage);
  float comfort = dhtValid ? comfortPenalty(temp_c, humidity) : 0;

  float aqi = (0.55 * pm25Sub) + (0.25 * pm10Sub) + (0.15 * gasSub) + (0.05 * comfort * 10);
  return constrain(aqi, 0, 500);
}

// ---------------------------------------------------------------
// Web page (structure copied directly from your confirmed-working reference)
// ---------------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AQI Drone Survey</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<style>
  html, body, #map { height:100%; margin:0; background:#0c1210; }
  #info {
    position:absolute; top:10px; left:50px; z-index:1000;
    background:#121b18; color:#e9efe9; padding:10px 14px; border-radius:6px;
    font-family:monospace; font-size:13px; box-shadow:0 1px 4px rgba(0,0,0,0.5);
    line-height:1.6;
  }
  #aqiVal { font-weight:bold; font-size:16px; }
  #markBtn {
    position:absolute; bottom:20px; left:50%; transform:translateX(-50%); z-index:1000;
    font-family:monospace; font-size:14px; text-transform:uppercase;
    background:#ffb454; color:#1a1306; border:none; padding:12px 20px;
    border-radius:6px; font-weight:bold;
  }
</style>
</head>
<body>
<div id="info">Waiting for data...</div>
<div id="map"></div>
<button id="markBtn">Mark this point</button>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
  var map = L.map('map').setView([0,0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors'
  }).addTo(map);

  var marker = L.marker([0,0]).addTo(map);
  var firstFix = false;
  var lastData = null;
  var lockedMarkers = [];

  function aqiColor(aqi) {
    if (aqi <= 50)  return "#6fcf6f";
    if (aqi <= 100) return "#1f8f3d";
    if (aqi <= 200) return "#e2734f";
    return "#c1272d";
  }

  function update() {
    fetch('/data').then(function(r) { return r.json(); }).then(function(d) {
      lastData = d;
      var color = aqiColor(d.aqi);

      document.getElementById('info').innerHTML =
        '<span id="aqiVal" style="color:' + color + '">AQI: ' + d.aqi.toFixed(0) + '</span><br>' +
        'PM2.5: ' + d.pm25 + ' | PM10: ' + d.pm10 + '<br>' +
        'Temp: ' + d.temp.toFixed(1) + 'C | Hum: ' + d.hum.toFixed(1) + '%<br>' +
        'Fix: ' + d.fix + ' | Sats: ' + d.sats + '<br>' +
        'Lat: ' + d.lat.toFixed(7) + '<br>' +
        'Lon: ' + d.lon.toFixed(7);

      if (d.lat !== 0 || d.lon !== 0) {
        marker.setLatLng([d.lat, d.lon]);
        marker.setStyle && marker.setStyle({ color: color });
        if (!firstFix) {
          map.setView([d.lat, d.lon], 17);
          firstFix = true;
        }
      }
    }).catch(function(e) { console.log("fetch error:", e); });
  }

  document.getElementById('markBtn').addEventListener('click', function() {
    if (!lastData) return;
    var color = aqiColor(lastData.aqi);
    var circle = L.circle([lastData.lat, lastData.lon], {
      radius: 3, color: color, fillColor: color, fillOpacity: 0.5
    }).addTo(map);
    circle.bindPopup('AQI ' + lastData.aqi.toFixed(0) + '<br>' + lastData.lat.toFixed(6) + ', ' + lastData.lon.toFixed(6));
  });

  setInterval(update, 1000);
  update();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleData() {
  float aqi = calculateAQI();

  String json = "{";
  json += "\"lat\":" + String(currentLat, 7) + ",";
  json += "\"lon\":" + String(currentLon, 7) + ",";
  json += "\"alt\":" + String(currentAlt, 2) + ",";
  json += "\"fix\":" + String(gpsFix) + ",";
  json += "\"sats\":" + String(satCount) + ",";
  json += "\"aqi\":" + String(aqi, 1) + ",";
  json += "\"pm25\":" + String(pm25_val) + ",";
  json += "\"pm10\":" + String(pm10_val) + ",";
  json += "\"temp\":" + String(temp_c, 1) + ",";
  json += "\"hum\":" + String(humidity, 1);
  json += "}";

  server.send(200, "application/json", json);
}

// ---------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------
unsigned long lastSensorRead = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Drone AQI Mapping System booting ===");

  mavSerial.begin(TELEM_BAUD, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, 27);
  dht.begin();

  analogReadResolution(12);
  analogSetPinAttenuation(MQ3_PIN, ADC_11db);

  delay(2000);
  requestAllStreams();
  lastStreamRequest = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi hotspot");

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! Browse to http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi FAILED to connect - check WIFI_SSID/WIFI_PASS, or hotspot range.");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  server.handleClient();

  while (mavSerial.available()) {
    processMavByte(mavSerial.read());
  }

  readPMS5003();

  if (millis() - lastStreamRequest > 5000) {
    requestAllStreams();
    lastStreamRequest = millis();
  }

  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();
    readMQ3();
    readDHT();

    Serial.printf("Fix:%d Sats:%d Lat:%.7f Lon:%.7f | PM2.5:%d PM10:%d | GasV:%.2f | T:%.1f H:%.1f | AQI:%.1f\n",
      gpsFix, satCount, currentLat, currentLon, pm25_val, pm10_val, mq3_voltage,
      temp_c, humidity, calculateAQI());
  }
}
