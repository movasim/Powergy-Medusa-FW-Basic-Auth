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
  */

// Replace with your credentials

#define MQTT_USER "user"
#define MQTT_PASSWORD "password"
#define MQTT_SERVER_IP 192, 168, 1, 1
#define MQTT_SERVER "mqtt.server.io"
#define MQTT_SERVER_PORT 1883
#define MQTT_PUBLISH_TOPIC  "t"
#define MQTT_SUBSCRIBE_TOPIC "command///req/#"
