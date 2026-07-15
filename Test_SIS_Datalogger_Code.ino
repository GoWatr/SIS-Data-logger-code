/*
 * Bluetooth is enabled for wireless serial monitor
 * Download Bluetooth Serial Terminal app from Playstore
 * Before compiling, set: Tools > Partition Scheme > "Huge App (3MB No OTA)"
 */

float CurrentVersion = 1.0;
#include <BluetoothSerial.h>
#include <SoftwareSerial.h>
#include <ESPping.h>
#include <ESP32-ENC28J60.h>
#include <WiFi.h>
#include <Ticker.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <ArduinoHttpClient.h>
#include <WiFiClientSecure.h>
#include <EEPROM.h>

BluetoothSerial SerialBT;

#define LED 2

// Ethernet pins
#define SPI_HOST 1
#define SPI_CLOCK_MHZ 8
#define INT_GPIO 14
#define MISO_GPIO 19
#define MOSI_GPIO 23
#define SCLK_GPIO 18
#define CS_GPIO 5

const char* ssid = "test";
const char* password = "12345678";

#define EEPROM_SIZE 64
#define EEPROM_ADDR_PUSH_INTERVAL 0
#define EEPROM_ADDR_NETWORK_MODE 4  // Store network mode (0 = Ethernet, 1 = WiFi)

#define NETWORK_MODE_ETHERNET 0
#define NETWORK_MODE_WIFI 1

SoftwareSerial mod(16, 17);  // RX=16, TX=17
//SoftwareSerial mod(26, 27);  // RX=26, TX=27

const char* msg = "A_Block_S1";        // Bluetooth name without newline
unsigned long pushData_interval = 60;  // hr*min*sec
const int NUM_FLOWMETERS = 26;

// Modbus request buffers
const byte flowmeterSlaveID[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24, 25, 26
};

// Flow meter IDs
const char* flowmeter_id[] = {
  "GFSFA11", "GFSFA21", "GFSFA31", "GFSFA41", "GFSFA51",
  "GFSFA61", "GFSFA71", "GFSFA81", "GFSFA91", "GFSFA101",
  "GFSFA111", "GFSFA121", "GFSFA131", 
  "GFSFA12", "GFSFA22", "GFSFA32", "GFSFA42", "GFSFA52",
  "GFSFA62", "GFSFA72", "GFSFA82", "GFSFA92", "GFSFA102",
  "GFSFA112", "GFSFA122", "GFSFA132"
};

int64_t totalizer[NUM_FLOWMETERS];
int flowrate[NUM_FLOWMETERS];

static bool eth_connected = false;
int ping_fail_counter = 0;
Ticker secondTick;
volatile int watchdogCount = 0;

WiFiClient wifiClient;
HttpClient client(wifiClient, "gowatr.com", 80);

TaskHandle_t bluetoothTaskHandle = NULL;
 
bool readModbusFlowMeter(int fmIndex);
void sendToServer(const char* data);
void WiFiEvent(WiFiEvent_t event);
void ping();
void ISRwatchdog();

void bluetoothTask(void* pvParameters) {
  char buffer[32];
  int index = 0;

  for (;;) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '\n' || c == '\r') {
        if (index > 0) {
          buffer[index] = '\0';
          for (int i = 0; buffer[i]; i++) buffer[i] = tolower(buffer[i]);

          if (strcmp(buffer, "restart") == 0) {
            SerialBT.println("Restarting ESP32...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP.restart();
          } else if (strncmp(buffer, "interval:", 9) == 0) {
            char* value = buffer + 9;
            unsigned long newInterval = 0;
            char unit = value[strlen(value) - 1];
            value[strlen(value) - 1] = '\0';
            newInterval = atol(value);

            if (unit == 's') {
            } else if (unit == 'm') {
              newInterval *= 60;
            } else if (unit == 'h') {
              newInterval *= 3600;
            } else {
              SerialBT.println("Invalid format. Use s, m, or h.");
              Serial.println("Invalid format. Use s, m, or h.");
              index = 0;
              continue;
            }

            if (newInterval >= 10 && newInterval <= 86400) {
              pushData_interval = newInterval;
              EEPROM.writeULong(EEPROM_ADDR_PUSH_INTERVAL, pushData_interval);
              EEPROM.commit();
              secondTick.detach();
              secondTick.attach(pushData_interval, ISRwatchdog);
              SerialBT.printf("Interval set to %lu seconds.\n", pushData_interval);
              Serial.printf("Interval set to %lu seconds.\n", pushData_interval);
            } else {
              SerialBT.println("Interval must be 10s to 24h");
              Serial.println("Interval must be 10s to 24h");
            }
          } else if (strncmp(buffer, "network:", 8) == 0) {
            char* value = buffer + 8;
            if (strcmp(value, "ethernet") == 0) {
              EEPROM.writeByte(EEPROM_ADDR_NETWORK_MODE, NETWORK_MODE_ETHERNET);
              EEPROM.commit();
              SerialBT.println("Network set to Ethernet. Restarting...");
              Serial.println("Network set to Ethernet. Restarting...");
              vTaskDelay(1000 / portTICK_PERIOD_MS);
              ESP.restart();
            } else if (strcmp(value, "wifi") == 0) {
              EEPROM.writeByte(EEPROM_ADDR_NETWORK_MODE, NETWORK_MODE_WIFI);
              EEPROM.commit();
              SerialBT.println("Network set to WiFi. Restarting...");
              Serial.println("Network set to WiFi. Restarting...");
              vTaskDelay(1000 / portTICK_PERIOD_MS);
              ESP.restart();
            } else {
              SerialBT.println("Invalid network mode. Use ethernet or wifi.");
              Serial.println("Invalid network mode. Use ethernet or wifi.");
            }
          } 
else if (strncmp(buffer, "reset:", 6) == 0) {

    char *token;
    char *rest = buffer;

    token = strtok(rest, ":");        // "reset"
    token = strtok(NULL, ":");        // slave ID

    if (token == NULL) {
        SerialBT.println("Invalid format. Use reset:<id> or reset:<id>:<value>");
        continue;
    }

    int slaveID = atoi(token);

    token = strtok(NULL, ":");        // value (optional)
    int writeValue = 0;               // default value

    if (token != NULL) {
        writeValue = atoi(token);     // user provided value
    }

  // Valid Modbus slave ID range = 1 to 247
  if (slaveID >= 1 && slaveID <= 247) {

      SerialBT.printf("Writing value %d to Slave ID %d\n", writeValue, slaveID);
      Serial.printf("Writing value %d to Slave ID %d\n", writeValue, slaveID);

      writeSingleRegister(slaveID, writeValue);

  } else {
      SerialBT.println("Invalid Slave ID (Valid range: 1-247)");
      Serial.println("Invalid Slave ID (Valid range: 1-247)");
  }

}
          else {
            SerialBT.println("Unknown command. Try restart, interval:<value><s/m/h>, or network:<ethernet/wifi>");
            Serial.println("Unknown command. Try restart, interval:<value><s/m/h>, or network:<ethernet/wifi>");
          }

          index = 0;
        }
      } else if (index < sizeof(buffer) - 1) {
        buffer[index++] = c;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

uint16_t calculateCRC(byte* buffer, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void writeSingleRegister(byte slaveID, uint16_t value) {

    byte writeCmd[8];

    // IMPORTANT:
    // Change register address below if required.
    // DO NOT use totalizer register address.
    // Example using 0x0002 (Change if needed)

    uint16_t registerAddress = 0x0002;   // <-- SAFE REGISTER (NOT totalizer)

    writeCmd[0] = slaveID;                  // Slave ID
    writeCmd[1] = 0x06;                     // Function Code (Write Single Register)
    writeCmd[2] = registerAddress >> 8;     // Register High
    writeCmd[3] = registerAddress & 0xFF;   // Register Low
    writeCmd[4] = value >> 8;               // Data High
    writeCmd[5] = value & 0xFF;             // Data Low

    uint16_t crc = calculateCRC(writeCmd, 6);
    writeCmd[6] = crc & 0xFF;
    writeCmd[7] = crc >> 8;

    while (mod.available()) mod.read();

    mod.write(writeCmd, 8);
    mod.flush();

    delay(300);

    SerialBT.println("Write command sent successfully.");
    Serial.println("Write command sent successfully.");
}


void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  SerialBT.begin(msg);  // Bluetooth name
  Serial.print("Bluetooth started. Pair with ");
  Serial.println(msg);
  SerialBT.print("Bluetooth started. Pair with ");
  SerialBT.println(msg);

  byte networkMode = EEPROM.readByte(EEPROM_ADDR_NETWORK_MODE);
  if (networkMode != NETWORK_MODE_ETHERNET && networkMode != NETWORK_MODE_WIFI) {
    networkMode = NETWORK_MODE_ETHERNET;  // Default to Ethernet
    EEPROM.writeByte(EEPROM_ADDR_NETWORK_MODE, networkMode);
    EEPROM.commit();
  }

  if (networkMode == NETWORK_MODE_WIFI) {
    // WiFi setup
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    SerialBT.print("Connecting to WiFi...");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(1000);
      Serial.print(".");
      SerialBT.print(".");
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected");
      SerialBT.println("\nWiFi connected");
      Serial.print("IP Address: ");
      SerialBT.print("IP Address: ");
      Serial.println(WiFi.localIP());
      SerialBT.println(WiFi.localIP());
      eth_connected = true;
    } else {
      Serial.println("\nWiFi connection failed");
      SerialBT.println("\nWiFi connection failed");
      eth_connected = false;
    }
  } else {
    // Ethernet setup
    ETH.begin(MISO_GPIO, MOSI_GPIO, SCLK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, SPI_HOST);
    delay(3000);
    ETH.setHostname("esp32-ethernet");

    int i = 0;
    while (!eth_connected && i < 2) {
      i++;
      delay(5000);
      Serial.println("Waiting for Ethernet connection");
      SerialBT.println("Waiting for Ethernet connection");
    }
  }

  pushData_interval = EEPROM.readULong(EEPROM_ADDR_PUSH_INTERVAL);
  Serial.printf("Interval set to %lu seconds.\n", pushData_interval);
  SerialBT.printf("Interval set to %lu seconds.\n", pushData_interval);
  if (pushData_interval == 0xFFFFFFFF || pushData_interval == 0) {
    pushData_interval = 60;
    Serial.printf("Interval set to %lu seconds.\n", pushData_interval);
    SerialBT.printf("Interval set to %lu seconds.\n", pushData_interval);
  }

  // Create Bluetooth monitoring task
  xTaskCreate(
    bluetoothTask,        // Task function
    "BluetoothTask",      // Task name
    4096,                 // Stack size (bytes)
    NULL,                 // Task parameters
    1,                    // Priority (same as loop)
    &bluetoothTaskHandle  // Task handle
  );

  secondTick.attach(pushData_interval, ISRwatchdog);
  mod.begin(9600);
}

void loop() {
  ping();
  for (int i = 0; i < NUM_FLOWMETERS; i++) {
    readModbusFlowMeter(i);
  }

  delay(1000);
  Serial.println(" ");
  SerialBT.println(" ");
}

void ISRwatchdog() {
  watchdogCount++;
  Serial.print("Watchdog tick: ");
  Serial.println(watchdogCount);
  SerialBT.print("Watchdog tick: ");
  SerialBT.println(watchdogCount);

  if (eth_connected) {
    char data[128];
    for (int i = 0; i < NUM_FLOWMETERS; i++) {
      snprintf(data, sizeof(data), "flowmeter_id:%s,flowrate:%d,totalizer:%lld,ver:1.0",
               flowmeter_id[i], flowrate[i], totalizer[i]);
      sendToServer(data);
    }
    watchdogCount = 0;
  } else {
    Serial.println("Network not connected, skipping data send");
    SerialBT.println("Network not connected, skipping data send");
  }

  if (watchdogCount >= 5) {
    Serial.println("Watchdog triggered! Restarting ESP32...");
    SerialBT.println("Watchdog triggered! Restarting ESP32...");
    ESP.restart();
  }
}

bool readModbusFlowMeter(int fmIndex)
{
    byte response[20];
    bool success = false;

    for (int attempt = 0; attempt < 5; attempt++)
    {
        while (mod.available()) mod.read();
        delay(10);

        // -------------------------------
        // Build Modbus Request Automatically
        // -------------------------------
        byte request[8];

        request[0] = flowmeterSlaveID[fmIndex];   // Slave ID
        request[1] = 0x03;                        // Function Code
        request[2] = 0x00;                        // Start Address High
        request[3] = 0x00;                        // Start Address Low
        request[4] = 0x00;                        // Number of Registers High
        request[5] = 0x04;                        // Number of Registers Low

        uint16_t crc = calculateCRC(request, 6);

        request[6] = crc & 0xFF;
        request[7] = crc >> 8;

        if (mod.write(request, 8) == 8)
        {
            mod.flush();
            delay(500);

            int bytesRead = 0;
            unsigned long startTime = millis();

            while (mod.available() && bytesRead < 20 && millis() - startTime < 1000)
            {
                response[bytesRead++] = mod.read();
            }

            if (bytesRead >= 13)
            {
                Serial.print("Received BYTES FM");
                Serial.print(fmIndex + 1);
                Serial.print(": ");

                SerialBT.print("Received BYTES FM");
                SerialBT.print(fmIndex + 1);
                SerialBT.print(": ");

                for (byte i = 0; i < bytesRead; i++)
                {
                    Serial.print(response[i], HEX);
                    Serial.print(' ');

                    SerialBT.print(response[i], HEX);
                    SerialBT.print(' ');
                }

                Serial.println();
                SerialBT.println();

                int adjust = 0;

                if (response[0] != flowmeterSlaveID[fmIndex])
                {
                    if (bytesRead > 1 && response[1] == flowmeterSlaveID[fmIndex])
                    {
                        adjust = 1;

                        Serial.printf("Byte shift detected for FM%d\n", fmIndex + 1);
                        SerialBT.printf("Byte shift detected for FM%d\n", fmIndex + 1);
                    }
                    else
                    {
                        Serial.printf("Invalid Device ID for FM%d Expected:%02X Received:%02X\n",
                                      fmIndex + 1,
                                      flowmeterSlaveID[fmIndex],
                                      response[0]);

                        SerialBT.printf("Invalid Device ID for FM%d Expected:%02X Received:%02X\n",
                                        fmIndex + 1,
                                        flowmeterSlaveID[fmIndex],
                                        response[0]);

                        continue;
                    }
                }

                if (response[0 + adjust] == flowmeterSlaveID[fmIndex] &&
                    response[1 + adjust] == 0x03 &&
                    response[2 + adjust] == 0x08)
                {
                    uint16_t calculatedCRC = calculateCRC(response + adjust, 11);
                    uint16_t receivedCRC =
                        response[11 + adjust] |
                        (response[12 + adjust] << 8);

                    if (calculatedCRC == receivedCRC)
                    {
                        totalizer[fmIndex] =
                            (response[3 + adjust] << 24) |
                            (response[4 + adjust] << 16) |
                            (response[5 + adjust] << 8) |
                            response[6 + adjust];

                        flowrate[fmIndex] =
                            (response[7 + adjust] << 24) |
                            (response[8 + adjust] << 16) |
                            (response[9 + adjust] << 8) |
                            response[10 + adjust];

                        Serial.print("FLOW METER-");
                        Serial.print(fmIndex + 1);
                        Serial.print(" TOTALIZER: ");
                        Serial.println(totalizer[fmIndex]);

                        Serial.print("FLOW METER-");
                        Serial.print(fmIndex + 1);
                        Serial.print(" FLOWRATE: ");
                        Serial.println(flowrate[fmIndex]);

                        SerialBT.print("FLOW METER-");
                        SerialBT.print(fmIndex + 1);
                        SerialBT.print(" TOTALIZER: ");
                        SerialBT.println(totalizer[fmIndex]);

                        SerialBT.print("FLOW METER-");
                        SerialBT.print(fmIndex + 1);
                        SerialBT.print(" FLOWRATE: ");
                        SerialBT.println(flowrate[fmIndex]);

                        success = true;
                        break;
                    }
                    else
                    {
                        Serial.print("CRC Failed FM-");
                        Serial.println(fmIndex + 1);

                        SerialBT.print("CRC Failed FM-");
                        SerialBT.println(fmIndex + 1);
                    }
                }
                else
                {
                    Serial.print("Invalid Response FM-");
                    Serial.println(fmIndex + 1);

                    SerialBT.print("Invalid Response FM-");
                    SerialBT.println(fmIndex + 1);
                }
            }
            else
            {
                Serial.print("Incomplete Response FM-");
                Serial.println(fmIndex + 1);

                SerialBT.print("Incomplete Response FM-");
                SerialBT.println(fmIndex + 1);
            }
        }
        else
        {
            Serial.print("Request Send Failed FM-");
            Serial.println(fmIndex + 1);

            SerialBT.print("Request Send Failed FM-");
            SerialBT.println(fmIndex + 1);
        }

        delay(100);
    }

    if (!success)
    {
        totalizer[fmIndex] = -1;
        flowrate[fmIndex] = -1;

        Serial.print("FLOW METER-");
        Serial.print(fmIndex + 1);
        Serial.print(" TOTALIZER: ");
        Serial.println(totalizer[fmIndex]);

        Serial.print("FLOW METER-");
        Serial.print(fmIndex + 1);
        Serial.print(" FLOWRATE: ");
        Serial.println(flowrate[fmIndex]);

        SerialBT.print("FLOW METER-");
        SerialBT.print(fmIndex + 1);
        SerialBT.print(" TOTALIZER: ");
        SerialBT.println(totalizer[fmIndex]);

        SerialBT.print("FLOW METER-");
        SerialBT.print(fmIndex + 1);
        SerialBT.print(" FLOWRATE: ");
        SerialBT.println(flowrate[fmIndex]);
    }

    return success;
}

void ping() {
  const IPAddress google_ip(8, 8, 8, 8);   // Google DNS
  const char* gowatr_host = "gowatr.com";  // Gowatr hostname
  bool googleOk = false;
  bool gowatrOk = false;

  // Ping Google DNS
  Serial.print("Pinging ");
  SerialBT.print("Pinging ");
  Serial.print(google_ip);
  SerialBT.print(google_ip);
  googleOk = Ping.ping(google_ip, 1);
  if (googleOk) {
    Serial.printf(" Google ping success, response time: %.2f ms\n", Ping.averageTime());
    SerialBT.printf(" Google ping success, response time: %.2f ms\n", Ping.averageTime());
  } else {
    Serial.println(" Google ping failed");
    SerialBT.println(" Google ping failed");
  }

  // Ping gowatr.com
  Serial.print("Pinging ");
  SerialBT.print("Pinging ");
  Serial.print(gowatr_host);
  SerialBT.print(gowatr_host);
  gowatrOk = Ping.ping(gowatr_host, 1);
  if (gowatrOk) {
    Serial.printf(" Gowatr ping success, response time: %.2f ms\n", Ping.averageTime());
    SerialBT.printf(" Gowatr ping success, response time: %.2f ms\n", Ping.averageTime());
  } else {
    Serial.println(" Gowatr ping failed");
    SerialBT.println(" Gowatr ping failed");
  }

  // Set network status: connected if either ping succeeds
  eth_connected = googleOk || gowatrOk;

  if (eth_connected) {
    digitalWrite(LED, LOW);
    ping_fail_counter = 0;
  } else {
    digitalWrite(LED, HIGH);
    ping_fail_counter++;
    if (ping_fail_counter >= 100) {
      Serial.println("Too many ping failures, restarting ESP32...");
      SerialBT.println("Too many ping failures, restarting ESP32...");
      ESP.restart();
    }
  }

  delay(1000);
}

void sendToServer(const char* data) {
  char url[256];
  snprintf(url, sizeof(url), "http://gowatr.com/wms/pushdata/sis_api.php?project_id=sis_florence&data=%s", data);
   
  //https://www.gowatr.com/wms/pushdata/sis_api.php?project_id=sis_florence&data=flowmeter_id:GFSFA01,flowrate:10,totalizer:202,ver:0.00


  const int maxRetries = 3;     // Max retry attempts
  const int retryDelay = 3000;  // Delay between retries in ms
  int attempt = 0;
  int httpResponseCode = 0;

  while (attempt < maxRetries) {
    HTTPClient http;

    Serial.print("Sending URL (Attempt ");
    Serial.print(attempt + 1);
    Serial.print("): ");
    Serial.println(url);
    SerialBT.print("Sending URL (Attempt ");
    SerialBT.print(attempt + 1);
    SerialBT.print("): ");
    SerialBT.println(url);

    http.begin(url);
    http.setTimeout(15000);  // 15 seconds timeout
    httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.print("Response: ");
      Serial.println(response);

      SerialBT.print("HTTP Response code: ");
      SerialBT.println(httpResponseCode);
      SerialBT.print("Response: ");
      SerialBT.println(response);

      http.end();
      break;  // Success, exit loop
    } else {
      Serial.print("HTTP Request failed, code: ");
      Serial.println(httpResponseCode);
      Serial.print("Error: ");
      Serial.println(http.errorToString(httpResponseCode));

      SerialBT.print("HTTP Request failed, code: ");
      SerialBT.println(httpResponseCode);
      SerialBT.print("Error: ");
      SerialBT.println(http.errorToString(httpResponseCode));

      // Specific error handling
      if (httpResponseCode == HTTPC_ERROR_CONNECTION_REFUSED) {
        Serial.println("Connection refused by server");
        SerialBT.println("Connection refused by server");
      } else if (httpResponseCode == HTTPC_ERROR_CONNECTION_LOST) {
        Serial.println("Connection lost");
        SerialBT.println("Connection lost");
      } else if (httpResponseCode == HTTPC_ERROR_READ_TIMEOUT) {
        Serial.println("Connection timed out");
        SerialBT.println("Connection timed out");
      }

      http.end();
      attempt++;

      if (attempt < maxRetries) {
        Serial.printf("Retrying in %d ms...\n", retryDelay);
        SerialBT.printf("Retrying in %d ms...\n", retryDelay);
        delay(retryDelay);
      }
    }
  }

  if (httpResponseCode <= 0) {
    Serial.println("All retry attempts failed.");
    SerialBT.println("All retry attempts failed.");
  }
}

void WiFiEvent(WiFiEvent_t event) {
  byte networkMode = EEPROM.readByte(EEPROM_ADDR_NETWORK_MODE);
  if (networkMode == NETWORK_MODE_WIFI) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("WiFi Started");
        SerialBT.println("WiFi Started");
        break;
      case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("WiFi Connected");
        SerialBT.println("WiFi Connected");
        break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("WiFi IPv4: ");
        SerialBT.print("WiFi IPv4: ");
        Serial.println(WiFi.localIP());
        SerialBT.println(WiFi.localIP());
        eth_connected = true;
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("WiFi Disconnected");
        SerialBT.println("WiFi Disconnected");
        eth_connected = false;
        break;
      case ARDUINO_EVENT_WIFI_STA_STOP:
        Serial.println("WiFi Stopped");
        SerialBT.println("WiFi Stopped");
        eth_connected = false;
        break;
      default:
        break;
    }
  } else {
    switch (event) {
      case ARDUINO_EVENT_ETH_START:
        Serial.println("ETH Started");
        SerialBT.println("ETH Started");
        break;
      case ARDUINO_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        SerialBT.println("ETH Connected");
        break;
      case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("ETH MAC: ");
        SerialBT.print("ETH MAC: ");
        Serial.print(ETH.macAddress());
        SerialBT.print(ETH.macAddress());
        Serial.print(", IPv4: ");
        SerialBT.print(", IPv4: ");
        Serial.print(ETH.localIP());
        SerialBT.print(ETH.localIP());
        Serial.print(", ");
        SerialBT.print(", ");
        Serial.print(ETH.linkSpeed());
        SerialBT.print(ETH.linkSpeed());
        Serial.println("Mbps");
        SerialBT.println("Mbps");
        eth_connected = true;
        break;
      case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        SerialBT.println("ETH Disconnected");
        eth_connected = false;
        break;
      case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        SerialBT.println("ETH Stopped");
        eth_connected = false;
        break;
      default:
        break;
    }
  }
}