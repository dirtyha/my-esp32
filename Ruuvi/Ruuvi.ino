#include <BLEDevice.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "xCredentials.h"

#define OUTPUT_BUFFER_LENGTH 300
#define DEBUG 1

String publishTopic = String("events/Ruuvi/");               // publish events here
const char server[] = "myhomeat.cloud";
const char authMethod[] = "use-token-auth";
const char token[] = TOKEN;
const char clientId[] = "d:Ruuvi:RuuviGW1";
char output_buffer[OUTPUT_BUFFER_LENGTH];

int scanTime = 10; //In seconds
BLEScan* pBLEScan;
double temp, hum, pressure, ax, ay, az, voltage;
unsigned long lastScan = 0L;
unsigned long lastPublish = 0L;
const int wdtTimeout = 30000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

WiFiClient wifiClient;
PubSubClient client(server, 1883, wifiClient);

//Converts hexadecimal values to decimal values
int hex2Dec(String hexVal)
{
  int len = hexVal.length();
  int base = 1;

  int dec_val = 0;

  for (int i = len - 1; i >= 0; i--)
  {
    if (hexVal[i] >= '0' && hexVal[i] <= '9')
    {
      dec_val += (hexVal[i] - 48) * base;

      base = base * 16;
    }
    else if (hexVal[i] >= 'A' && hexVal[i] <= 'F')
    {
      dec_val += (hexVal[i] - 55) * base;

      base = base * 16;
    }
  }
  return dec_val;
}

boolean decodeRuuvi(String hex) {
  hum = hex2Dec(hex.substring(6, 8)) * 0.5;
  if (hum < 0.0 || hum > 100.0) {
    return false;
  }

  temp = (hex2Dec(hex.substring(8, 10)) & 0b01111111) + (hex2Dec(hex.substring(10, 12)) / 100.0);
  temp = (hex2Dec(hex.substring(8, 10)) & 0b10000000) == 128 ? temp * -1 : temp;
  if (temp < -50.0 || temp > 100.0) {
    return false;
  }

  pressure = hex2Dec(hex.substring(12, 16)) * 1.0 + 50000;
  pressure = pressure / 100.0;
  if (pressure < 900.0 || pressure > 1100.0) {
    return false;
  }

  ax = hex2Dec(hex.substring(16, 20));
  ay = hex2Dec(hex.substring(20, 24));
  az = hex2Dec(hex.substring(24, 28));
  voltage = hex2Dec(hex.substring(28, 32));

  return true;
}

//Class that scans for BLE devices
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      //Scans for specific BLE MAC addresses
      String raw = String(BLEUtils::buildHexData(nullptr, (uint8_t*)advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length()));
      if (raw.length() == 32) {
        raw.toUpperCase();
        if (decodeRuuvi(raw)) {
          publish(advertisedDevice.getAddress().toString().c_str());
        } else {
          Serial.println("Unable to decode raw data.");
        }
      }
    }
};

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

void setup() {
  Serial.begin(115200);

  wifiConnect();
  mqttConnect();

  //BLE scanning
  Serial.println("Setting up BLE scanning...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt

  if (DEBUG) {
    Serial.println("Setup done");
  }
}

void loop() {

  timerWrite(timer, 0); //reset timer (feed watchdog)

  // check that we are connected
  if (!client.loop()) {
    mqttConnect();
  }

  unsigned long now = millis();
  if (now - lastScan > 120000 || lastScan == 0L) {
    lastScan = now;
    Serial.println("BLE scanning...");
    pBLEScan->start(scanTime);
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  }

  if (now - lastPublish > 300000) {
    resetModule();
  }
}

void wifiConnect() {
  if (DEBUG) {
    Serial.print("Connecting to "); Serial.print(ssid);
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (DEBUG) {
      Serial.print(".");
    }
  }
  WiFi.mode(WIFI_STA);
  if (DEBUG) {
    Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
  }
}

void mqttConnect() {
  if (!!!client.connected()) {
    if (DEBUG) {
      Serial.print("Reconnecting MQTT client to "); Serial.println(server);
    }
    int count = 20;
    while (count-- > 0 && !!!client.connect(clientId, authMethod, token)) {
      if (DEBUG) {
        Serial.print(".");
      }
      delay(500);
    }
    if (DEBUG) {
      Serial.println();
    }
  }
}

boolean publish(String deviceId) {
  boolean ret = true;

  StaticJsonBuffer<OUTPUT_BUFFER_LENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");

  d["TA"] = temp;
  d["RH"] = hum;
  d["PA"] = pressure;
  d["battery"] = voltage;
  d["Td"] = calculateTd(hum, temp);

  deviceId.replace(":", "");
  String topic = publishTopic + deviceId;
  if (DEBUG) {
    Serial.print("Topic: "); Serial.println(topic);
    Serial.println("Publish payload:"); root.prettyPrintTo(Serial); Serial.println();
  }

  root.printTo(output_buffer, OUTPUT_BUFFER_LENGTH);
  if (client.publish(topic.c_str(), output_buffer)) {
    lastPublish = millis();
    if (DEBUG) {
      Serial.println("Publish OK");
    }
  } else {
    if (DEBUG) {
      Serial.println("Publish FAILED");
    }
    ret = false;
  }

  return ret;
}

float calculateTd(float rh, float ta) {
  return 243.04 * (log(rh / 100.0) + ((17.625 * ta) / (243.04 + ta))) / (17.625 - log(rh / 100.0) - ((17.625 * ta) / (243.04 + ta)));
}
