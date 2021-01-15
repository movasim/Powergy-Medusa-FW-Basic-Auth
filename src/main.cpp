/*
  Powergy Medusa. Basic-Authentication Firmware flavor.
  Power and Energy Meter for 1 or 3 Phases and with WiFi connectivity.

  MIT License

  Copyright (C) 2021  MOVASIM (https://movasim.com/)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

 =================== 
  = PZEM-004T V3.0 =
  ==================
  PZEM-004T V3.0 module measures AC voltage, current, active power, frequency,
  power factor and active energy. Data is read by the microcontroller through
  TTL interface.
  connections:
  - GND -> GND
  - RX -> TX (NodeMCU depends on PZEM #)
  - TX -> RX (NodeMCU depends on PZEM #)
  - VCC -> 5V

  ==========
  = Jumper =
  ==========
  The Juper (JP1) is used to switch Powergy into Debug/Production mode.
  connections:
  - D1 -> GND: Debug Mode
  - D1 -> N/C: Production Mode 

  Debug Mode: Useful information will be published in the Serial Port.
  Production Mode. Few infromation will be published to the Serial Port.

*/

#include <Arduino.h>                // https://github.com/arduino/ArduinoCore-avr
#include <ESP8266WiFi.h>            // https://github.com/esp8266/Arduino
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager/tree/development
#include <PubSubClient.h>           // https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>            // https://github.com/bblanchon/ArduinoJson
#include <PZEM004Tv30.h>            // https://github.com/mandulaj/PZEM-004T-v30
#include "mqtt.configuration.h"     // MQTT file configuration.

// ========== Start Device User Parametrization ============================================================================

#define Alias "Distribution Panel";             // Friendly name for the Device location.
unsigned long mqttSensorsReportPeriod = 10000;  // Sensors Report Period (Miliseconds).
unsigned int mqttDeviceReportPeriod = 3;        // Device Report Period (times) based on MQTTSensorsReportPeriod.
int resetPortal = 180;                          // Number of seconds until the WiFiManager resests ESP8266.
#define AP_Password "powergy.movasim"           // AP password.
#define Phases 1                                // Define the number of PZEM-004T devices installed in the main board [1,3].

// ========== Start Device Development Parametrization (ONLY MODIFY WHEN NEW HW/FW VERSION IS RELASED) =====================

#define DeviceType "Powergy" 
#define DeviceModel "Medusa"
#define DeviceVersion "1.0.0"
#define FirmwareFlavor "Basic-Auth"
#define FirmwareVersion "1.0.0"
#define JUMPER 10
#define PZEM_1_RX D1 
#define PZEM_1_TX D2
#define PZEM_2_RX D5
#define PZEM_2_TX D6
#define PZEM_3_RX D7
#define PZEM_3_TX D8
#define LED D0

// ========== End Device Parametrization ===================================================================================

const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
IPAddress mqtt_server_ip(MQTT_SERVER_IP);
int mqtt_server_port = MQTT_SERVER_PORT;
const char* mqtt_clientId;
const char* mqtt_publish_topic = MQTT_PUBLISH_TOPIC;
const char* mqtt_subscribe_topic = MQTT_SUBSCRIBE_TOPIC;
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned int counter = 0;
String strClientId; 
bool debug;

// Helper functions declarations
String GetDeviceName(void);
String GetMyMACAddress(void);
void reconnectMqttBroker(void);
void messageReceived(char* p_topic, byte* p_payload, unsigned int p_length);
void publishSensorsData1(float voltage1, float current1, float power1, float energy1, float frequency1, float powerfactor1);
void publishSensorsData3(float voltage1, float current1, float power1, float energy1, float frequency1, float powerfactor1, float voltage2, float current2, float power2, float energy2, float frequency2, float powerfactor2, float voltage3, float current3, float power3, float energy3, float frequency3, float powerfactor3);
void publishDeviceData(String dev_t, String dev_m, String dev_v, String fw_f, String fw_v, String mac, String ip, byte s_qty, unsigned long up, String rst_r, unsigned int free_heap, byte heap_frg);

// Create  object of the class.

PZEM004Tv30 pzem1(PZEM_1_RX, PZEM_1_TX);  // (RX,TX) connect to TX,RX of PZEM-1
PZEM004Tv30 pzem2(PZEM_2_RX, PZEM_2_TX);  // (RX,TX) connect to TX,RX of PZEM-2
PZEM004Tv30 pzem3(PZEM_3_RX, PZEM_3_TX);  // (RX,TX) connect to TX,RX of PZEM-3
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {

  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial);      // Wait for hardware serial to appear.

  // Check JUMPER Status.
  pinMode(JUMPER, INPUT_PULLUP);
  Serial.println();
  if (digitalRead(JUMPER) == LOW)
  {
    debug = true;
    Serial.println(F("DEBUG Mode ON"));
  } else
  {
    debug = false;
    Serial.println(F("DEBUG Mode OFF"));
  }

  // Generate ClientID with Device name and MAC.
  strClientId = GetDeviceName();
  mqtt_clientId = strClientId.c_str();

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  if (debug == true)
  {
    wifiManager.setDebugOutput(true); // Send debugging info to serial port.
  } else
  {
    wifiManager.setDebugOutput(false);
  }

  wifiManager.setConfigPortalTimeout(resetPortal); // auto close configportal after n seconds

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "ES-XXXX with the last 4 digits of the MAC Address"),
  // then goes into a blocking loop awaiting configuration and will return success result
  String APName = GetDeviceName();
  bool res;
  res = wifiManager.autoConnect((const char*)APName.c_str(),AP_Password); // password protected ap
  if(!res)
  {
    Serial.println(F("Failed to connect"));
    ESP.restart();
  } 
  else
  {
    // If you get here you have connected to the WiFi    
    Serial.println(F("Connected to the WiFi Network!"));
  }

  // Setup the MQTT Client
  client.setServer(mqtt_server_ip, mqtt_server_port); // Connect to the MQTT Broker using IP
  //client.setServer(mqtt_server, mqtt_server_port); // Connect to the MQTT Broker using URL
  client.setBufferSize(2048);
  client.setCallback(messageReceived);

  pinMode(LED, OUTPUT);     // Initialize LED pin as output.
  digitalWrite(LED, HIGH);  // Initialize the LED off by making the voltage HIGH.

  Serial.println(F("End setup"));
}

void loop() 
{
  currentTime = millis();
  
  // Check if MQTT connection is active. Otherwise reconnect.
  if (!client.connected())
    {
      reconnectMqttBroker();
    }
  client.loop();

  if (currentTime - previousTime >= mqttSensorsReportPeriod)
  {
    previousTime = currentTime;
    counter++;
    
    // Read PZEM-004T-1
    float voltage1 = pzem1.voltage();
    float current1 = pzem1.current();
    float power1 = pzem1.power();
    float energy1 = pzem1.energy();
    float frequency1 = pzem1.frequency();
    float powerfactor1 = pzem1.pf();

    if (debug == true)
    {
      if( !isnan(voltage1) )
      {
        Serial.print(F("Voltage: ")); Serial.print(voltage1); Serial.println(F("V"));
      } else 
      {
        Serial.println(F("Error reading voltage"));
      }

      if( !isnan(current1) )
      {
          Serial.print("Current: "); Serial.print(current1); Serial.println("A");
      } else 
      {
          Serial.println(F("Error reading current"));
      }

      if( !isnan(power1) )
      {
          Serial.print(F("Power: ")); Serial.print(power1); Serial.println(F("W"));
      } else 
      {
          Serial.println(F("Error reading power"));
      }

      if( !isnan(energy1) )
      {
          Serial.print(F("Energy: ")); Serial.print(energy1,3); Serial.println(F("kWh"));
      } else 
      {
          Serial.println(F("Error reading energy"));
      }

      if( !isnan(frequency1) )
      {
          Serial.print(F("Frequency: ")); Serial.print(frequency1, 1); Serial.println(F("Hz"));
      } else 
      {
          Serial.println(F("Error reading frequency"));
      }

      if( !isnan(powerfactor1) )
      {
          Serial.print(F("PF: ")); Serial.println(powerfactor1);
      } else 
      {
          Serial.println(F("Error reading power factor"));
      }
    }

    if (Phases > 1) // Phases could be eiher 1 or 3.
    {
      // Read PZEM-004T-2
      float voltage2 = pzem2.voltage();
      float current2 = pzem2.current();
      float power2 = pzem2.power();
      float energy2 = pzem2.energy();
      float frequency2 = pzem2.frequency();
      float powerfactor2 = pzem2.pf();
      // Read PZEM-004T-3
      float voltage3 = pzem3.voltage();
      float current3 = pzem3.current();
      float power3 = pzem3.power();
      float energy3 = pzem3.energy();
      float frequency3 = pzem3.frequency();
      float powerfactor3 = pzem3.pf();

      if (debug == true)
      {
        if( !isnan(voltage2) )
        {
          Serial.print(F("Voltage: ")); Serial.print(voltage2); Serial.println(F("V"));
        } else 
        {
          Serial.println(F("Error reading voltage"));
        }

        if( !isnan(current2) )
        {
            Serial.print(F("Current: ")); Serial.print(current2); Serial.println(F("A"));
        } else 
        {
            Serial.println(F("Error reading current"));
        }

        if( !isnan(power2) )
        {
            Serial.print(F("Power: ")); Serial.print(power2); Serial.println(F("W"));
        } else 
        {
            Serial.println(F("Error reading power"));
        }

        if( !isnan(energy2) )
        {
            Serial.print(F("Energy: ")); Serial.print(energy2,3); Serial.println(F("kWh"));
        } else 
        {
            Serial.println(F("Error reading energy"));
        }

        if( !isnan(frequency2) )
        {
            Serial.print(F("Frequency: ")); Serial.print(frequency2, 1); Serial.println(F("Hz"));
        } else 
        {
            Serial.println(F("Error reading frequency"));
        }

        if( !isnan(powerfactor2) )
        {
            Serial.print(F("PF: ")); Serial.println(powerfactor2);
        } else 
        {
            Serial.println(F("Error reading power factor"));
        }

        if( !isnan(voltage3) )
        {
          Serial.print("Voltage: "); Serial.print(voltage3); Serial.println("V");
        } else 
        {
          Serial.println(F("Error reading voltage"));
        }

        if( !isnan(current3) )
        {
            Serial.print(F("Current: ")); Serial.print(current3); Serial.println(F("A"));
        } else 
        {
            Serial.println(F("Error reading current"));
        }

        if( !isnan(power3) )
        {
            Serial.print(F("Power: ")); Serial.print(power3); Serial.println(F("W"));
        } else 
        {
            Serial.println(F("Error reading power"));
        }

        if( !isnan(energy3) )
        {
            Serial.print(F("Energy: ")); Serial.print(energy3,3); Serial.println(F("kWh"));
        } else 
        {
            Serial.println(F("Error reading energy"));
        }

        if( !isnan(frequency3) )
        {
            Serial.print(F("Frequency: ")); Serial.print(frequency3, 1); Serial.println(F("Hz"));
        } else 
        {
            Serial.println(F("Error reading frequency"));
        }

        if( !isnan(powerfactor3) )
        {
            Serial.print(F("PF: ")); Serial.println(powerfactor3);
        } else 
        {
            Serial.println(F("Error reading power factor"));
        }
      }

      publishSensorsData3(voltage1, current1, power1, energy1, frequency1, powerfactor1, voltage2, current2, power2, energy2, frequency2, powerfactor2, voltage3, current3, power3, energy3, frequency3, powerfactor3);

    } else
    {
      publishSensorsData1(voltage1, current1, power1, energy1, frequency1, powerfactor1);
    }
    
  }

  if ((counter == mqttDeviceReportPeriod) && (currentTime - previousTime >= (mqttSensorsReportPeriod/2)))
  {
    counter = 0;

    String deviceType = DeviceType;
    String deviceModel = DeviceModel;
    String deviceVersion = DeviceVersion;
    String firmwareFlavor = FirmwareFlavor;
    String firmwareVersion = FirmwareVersion;
    String deviceMAC = GetMyMACAddress();
    String deviceIP = WiFi.localIP().toString();

    // Device WiFi Signal Quality
    byte signalQuality = 0; 
    long rssi = WiFi.RSSI();
    // dBm to Signal Quality [%]:
      if(rssi <= -100)
          signalQuality = 0;
      else if(rssi >= -50)
          signalQuality = 100;
      else
          signalQuality = 2 * (rssi + 100);

    unsigned long deviceUptime = millis(); // Device Uptime
    String deviceResetReason = ESP.getResetReason(); // Returns a String containing the last reset reason in human readable format.
    unsigned int deviceFreeHeap = ESP.getFreeHeap(); // Returns the free heap size.
    byte deviceHeapFragmentation = ESP.getHeapFragmentation(); // Returns the fragmentation metric (0% is clean, more than ~50% is not harmless)

    publishDeviceData(deviceType, deviceModel, deviceVersion, firmwareFlavor, firmwareVersion, deviceMAC, deviceIP, signalQuality, deviceUptime, deviceResetReason, deviceFreeHeap, deviceHeapFragmentation);
  }
}

// Helper function definitions

// Function to get the Device Name. Device name is DeviceType + the last 4 MAC characters.
// This Device Name is used for WiFi SSID and MQTT clientId.
String GetDeviceName()
{
  String ssid1 = DeviceType;
  uint8_t mac[6];
  char macStr[6] = {0};
  String ssidDeviceName;
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X%02X", mac[4], mac[5]);
  ssidDeviceName = ssid1 + String(macStr);
  return  ssidDeviceName;  
}

// Function to get the Device MAC Address.
String GetMyMACAddress()
{
  uint8_t mac[6];
  char macStr[18] = {0};
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],  mac[1], mac[2], mac[3], mac[4], mac[5]); 
  return  String(macStr);
}

//Function to connect to MQTT Broker.
void reconnectMqttBroker()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection"));
    // Attempt to connect
      if (client.connect(mqtt_clientId, mqtt_user, mqtt_password))
      {
      if (debug == true)
      {
        Serial.print(F(" with clientID ")); Serial.print(mqtt_clientId);
      } 
      Serial.println(F("...Connected!"));
      // Subscribe to topics
      client.subscribe(mqtt_subscribe_topic);
      if (debug == true)
      {
        Serial.print(F("Subscribed to topic: ")); Serial.println(mqtt_subscribe_topic);
      }
    } else
    {
      Serial.print(F("...failed, rc=")); Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// function called when a MQTT message arrived.
void messageReceived(char* topic, byte* payload, unsigned int length)
{
  if (debug == true)
  {
    Serial.print(F("Message arrived ["));
    Serial.print(topic);
    Serial.print(F("] "));
    for (unsigned int i=0;i<length;i++) 
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }

  // Deserialize the JSON document
  DynamicJsonDocument jsonReceivedCommand(2048);
  DeserializationError error = deserializeJson(jsonReceivedCommand, payload);

  // Test if parsing succeeds.
  if (error)
  {
    if (debug == true)
    {
      Serial.print(F("deserializeJson() failed: ")); Serial.println(error.f_str());
    }
    return;
  }
  
  // Fetch values.
  // Command: Reset Accumulated Energy Counter.
  if(jsonReceivedCommand["value"].as<String>() == "ResetEnergy")
  {
    if (debug == true)
    {
      Serial.println(F("Accumulated Energy Counter is going to be Reset"));
    }
    pzem1.resetEnergy();
  }

  // Command: Restart Device.
  if(jsonReceivedCommand["value"].as<String>() == "restart")
  {
    if (debug == true)
    {
      Serial.print(F("Disconnecting from ")); Serial.println(mqtt_server);
    }
    client.disconnect();
    if (debug == true)
    {
      Serial.println(F("ESP8266 is going to be restarted"));
    }
    delay (1000);
    ESP.restart();
  }

// Command: Reset WiFi Settings - wipe credentials.
  if(jsonReceivedCommand["value"].as<String>() == "ResetWiFiSettings")
  {
    if (debug == true)
    {
      Serial.print(F("Disconnecting from ")); Serial.println(mqtt_server);
    }
    client.disconnect();
    if (debug == true)
    {
      Serial.println(F("WiFi settings are cleared. ESP8266 is going to be restarted"));
    }
    WiFiManager wifiManager;
    wifiManager.resetSettings(); 
    delay (1000);
    ESP.restart();
  }
}

// function called to publish Sensors data (Temperature, Pressure, Humidity, IAQ and Lux).
void publishSensorsData1(float voltage1, float current1, float power1, float energy1, float frequency1, float powerfactor1)

{
  StaticJsonDocument<256> jsonSensorsData;
  jsonSensorsData["msg_type"] = "srs";
  // PZEM-004T-1
  jsonSensorsData["v1"] = voltage1;
  jsonSensorsData["i1"] = current1;
  jsonSensorsData["p1"] = power1;
  jsonSensorsData["e1"] = energy1;
  jsonSensorsData["f1"] = frequency1;
  jsonSensorsData["pf1"] = powerfactor1;
  // Physical location of the device.
  jsonSensorsData["alias"] = Alias;
  
  char buffer[256];
  serializeJson(jsonSensorsData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}


// function called to publish Sensors data (Temperature, Pressure, Humidity, IAQ and Lux).
void publishSensorsData3(float voltage1, float current1, float power1, float energy1, float frequency1, float powerfactor1, float voltage2, float current2, float power2, float energy2, float frequency2, float powerfactor2, float voltage3, float current3, float power3, float energy3, float frequency3, float powerfactor3)

{
  StaticJsonDocument<512> jsonSensorsData;
  jsonSensorsData["msg_type"] = "srs";
  // PZEM-004T-1
  jsonSensorsData["v1"] = voltage1;
  jsonSensorsData["i1"] = current1;
  jsonSensorsData["p1"] = power1;
  jsonSensorsData["e1"] = energy1;
  jsonSensorsData["f1"] = frequency1;
  jsonSensorsData["pf1"] = powerfactor1;
  // PZEM-004T-2
  jsonSensorsData["v2"] = voltage2;
  jsonSensorsData["i2"] = current2;
  jsonSensorsData["p2"] = power2;
  jsonSensorsData["e2"] = energy2;
  jsonSensorsData["f2"] = frequency2;
  jsonSensorsData["pf2"] = powerfactor2;
  // PZEM-004T-3
  jsonSensorsData["v3"] = voltage3;
  jsonSensorsData["i3"] = current3;
  jsonSensorsData["p3"] = power3;
  jsonSensorsData["e3"] = energy3;
  jsonSensorsData["f3"] = frequency3;
  jsonSensorsData["pf3"] = powerfactor3;
  // Physical location of the device.
  jsonSensorsData["alias"] = Alias;
  
  char buffer[512];
  serializeJson(jsonSensorsData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}

// function called to publish Device information (Type, Model, Version, Firmware Flavor, Firmware version, MAC, IP, WiFi Signal Quality, Uptime, etc.).
void publishDeviceData(String dev_t, String dev_m, String dev_v, String fw_f, String fw_v, String mac, String ip, byte s_qty, unsigned long up, String rst_r, unsigned int free_heap, byte heap_frg)
{
  StaticJsonDocument<512> jsonDeviceData;
  jsonDeviceData["msg_type"] = "dev";
  // Device
  jsonDeviceData["dev_t"] = dev_t;
  jsonDeviceData["dev_m"] = dev_m;
  jsonDeviceData["dev_v"] = dev_v;
  jsonDeviceData["phases"] = Phases;
  jsonDeviceData["fw_f"] = fw_f;
  jsonDeviceData["fw_v"] = fw_v;
  jsonDeviceData["mac"] = mac;
  jsonDeviceData["ip"] = ip;
  jsonDeviceData["s_qty"] = s_qty;
  jsonDeviceData["up"] = up;
  jsonDeviceData["rst_r"] = rst_r;
  jsonDeviceData["free_heap"] = free_heap;
  jsonDeviceData["heap_frg"] = heap_frg;
  jsonDeviceData["debug"] = debug;

  char buffer[512];
  serializeJson(jsonDeviceData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}