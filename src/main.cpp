#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFi_credentials.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// #include <AsyncElegantOTA.h>
// #include <WebSerial.h>

#include "ModbusClientRTU.h"
#include "Logging.h"
#include "socomec_modbus_registers.h"

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

// Set timezone string according to https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// Examples:
//  Pacific Time: "PST8PDT"
//  Eastern: "EST5EDT"
//  Japanesse: "JST-9"
//  Central Europe: "CET-1CEST,M3.5.0,M10.5.0/3"
// #define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"
#define TZ_INFO "Africa/Casablanca"

#define MODBUS_DIR_PIN 4 // connect DR, RE pin of MAX485 to gpio 4
#define MODBUS_RX_PIN 16 // ESP32 Rx pin | Receiver Output (RO) of MAX485
#define MODBUS_TX_PIN 17 // ESP32 Tx pin | Driver Input (DI) of MAX485
#define MODBUS_BAUDRATE 9600 // 19200

#define MODBUS_NRJ_METER_ADDR 5

#define ENABLE_INFLUXDB 0
#define ENABLE_SERIAL_PRINT 1

bool data_ready = false;

uint32_t r_data1;
uint16_t r_data_twos_comp;
float r_datafloat1;

/*
Array for float values retrieved from NRJ meter
pos 0 - Phase 1 amp
pos 1 - Phase 2 amp
pos 2 - Phase 3 amp
pos 3 - 
*/
#define NRJ_ARRAY_SIZE 12
float nrj_values[NRJ_ARRAY_SIZE];

// Create a ModbusRTU client instance
// In my case the RS485 module had auto halfduplex, so no second parameter with the DE/RE pin is required!
HardwareSerial Serial_RS485(2);
ModbusClientRTU MB(MODBUS_DIR_PIN);

// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient influxDBclient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Data point
Point wifi_sensor("wifi_status");
Point nrj_points("nrj_points_soco");

// Set Static IP address & gateway
// IPAddress local_IP(192, 168, 1, 166);
// IPAddress gateway(192, 168, 1, 254);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress dns1(192, 168, 1, 254);
// IPAddress dns2(0, 0, 0, 0);

// Define an onData handler function to receive the regular responses
// Arguments are Modbus server ID, the function code requested, the message data and length of it,
// plus a user-supplied token to identify the causing request
void handleData(ModbusMessage response, uint8_t token)
{
  Serial.printf("Response: serverID=%d, FC=%d, Token=%08X, length=%d:\n", response.getServerID(), response.getFunctionCode(), token, response.size());
  for (auto& byte : response) {
    Serial.printf("%02X ", byte);
  }

  Serial.printf("\n\n");
  // First value is on pos 3, after server ID, function code and length byte
  r_data1 = (response[3] << 8) | response[4] ;

  /** Code sandbox tested at onlinegdb.com **
    uint16_t data = 32767;
    uint16_t datainv = 0;
    float datafloat = 0.00;
    datainv = ~data + 1;
    datafloat = 0 - (float)datainv / 10;
    printf("data hex = %X\n", data);
    printf("data dig = %d\n", data);
    printf("datainv = %X\n", datainv);
    printf("datainv = %d\n", datainv);
    printf("datafloat = %.2f", datafloat);

    65535 / 2 = 32767
    65535 = -1

    32767 (+)
    --limit--
    32768 (-)
  */

  // Check if response contains negative values and convert it using 2's complement principle
  if (r_data1 > 32767)
  {
    r_data_twos_comp = ~r_data1 + 1;
    r_datafloat1 = 0 - float(r_data_twos_comp) / 10;
  }
  else
    r_datafloat1 = float(r_data1) / 10;

  // Switch case to evaluate token match
  switch (token)
  {

  // 0 REG_SOCO_VOLTAGE
  case 0:
    nrj_values[0] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 100;
    break;

  // 1 REG_SOCO_CURRENT
  case 1:
    nrj_values[1] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 2 REG_SOCO_FREQ
  case 2:
    nrj_values[2] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 3 REG_SOCO_ACTIV_POWER
  case 3:
    nrj_values[3] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 4 REG_SOCO_REACT_POWER
  case 4:
    nrj_values[4] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 5 REG_SOCO_APPAR_POWER
  case 5:
    nrj_values[5] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 6 REG_SOCO_SUM_POWER_FACTOR
  case 6:
    nrj_values[6] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 7 REG_SOCO_PH1_POWER_FACTOR
  case 7:
    nrj_values[7] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 8 REG_SOCO_TOT_POS_ACT_ENERGY
  case 8:
    nrj_values[8] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 9 REG_SOCO_TOT_POS_REACT_ENERGY
  case 9:
    nrj_values[9] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 10 REG_SOCO_TOT_NEG_ACT_ENERGY
  case 10:
    nrj_values[10] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 11 REG_SOCO_TOT_NEG_REACT_ENERGY
  case 11:
    nrj_values[11] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  default:
    break;
  }

  // Signal "data is complete"
  data_ready = true;
}

// Define an onError handler function to receive error responses
// Arguments are the error code returned and a user-supplied token to identify the causing request
void handleError(Error error, uint8_t token)
{
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  Serial.printf("Error response : %02X - %s\n", (int)me, (const char *)me);
}

void Task_retrieve_modbus_params(void *pvParameters);
void Task_print_values(void *pvParameters);
void Task_push_to_influxdb(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  Serial.printf("__ SERIAL OK __\n");

  // ************************ InfluxDB init ************************ //
  // Setup wifi
  WiFi.mode(WIFI_AP_STA);

  int n_wifi = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n_wifi == 0) {
      Serial.println("no networks found");
  } else {
    Serial.print(n_wifi);
    Serial.println(" networks found");
    for (int i = 0; i < n_wifi; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }

  wifiMulti.addAP(WIFI_SSID_1, WIFI_PASSWORD_1);
  wifiMulti.addAP(WIFI_SSID_2, WIFI_PASSWORD_2);
  wifiMulti.addAP(WIFI_SSID_3, WIFI_PASSWORD_3);

  // if (!WiFi.config(local_IP, gateway, subnet, dns1, dns2)){
  //   Serial.println("Failed to configure IP address !");
  // }

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Display connection details
  Serial.print("Wi-Fi SSID : ");
  Serial.println(WiFi.SSID());
  Serial.print("Wi-Fi Channel : ");
  Serial.println(WiFi.channel());
  Serial.print("Station IP Address : ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway IP : ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet Mask : ");
  Serial.println(WiFi.subnetMask());
  Serial.print("DNS 1 : ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("DNS 2 : ");
  Serial.println(WiFi.dnsIP(1));

  // Add tags
  wifi_sensor.addTag("device", DEVICE);
  wifi_sensor.addTag("SSID", WiFi.SSID());
  nrj_points.addTag("socomec", "4850 3043");

  // Accurate time is necessary for certificate validation and writing in batches
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (influxDBclient.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    // WebSerial.print("Connected to InfluxDB: ");
    Serial.println(influxDBclient.getServerUrl());
    // WebSerial.println(influxDBclient.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed : ");
    Serial.println(influxDBclient.getLastErrorMessage());
  }

  // ************************ MODBUS Init ************************ //
  // Set up ModbusRTU client.
  RTUutils::prepareHardwareSerial(Serial_RS485);

  // Baudrate, Serial param, TXD, RXD
  // Baudrate 9600, Parity none, stop bit 1
  Serial_RS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);

  // - provide onData handler function
  MB.onDataHandler(&handleData);
  // - provide onError handler function
  MB.onErrorHandler(&handleError);
  // Set message timeout to 2000ms
  MB.setTimeout(2000);
  // Start ModbusRTU background task
  MB.begin(Serial_RS485);

  Serial.println(xTaskCreate(Task_retrieve_modbus_params,
                          "Task_retrieve_modbus_params",
                          10000,
                          NULL,
                          4,
                          NULL));

  Serial.println(xTaskCreate(Task_print_values,
                          "Task_print_values",
                          10000,
                          NULL,
                          4,
                          NULL));

  Serial.println(xTaskCreate(Task_push_to_influxdb,
                          "Task_push_to_influxdb",
                          10000,
                          NULL,
                          4,
                          NULL));
}

void loop() {
  // put your main code here, to run repeatedly:
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Task_retrieve_modbus_params(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  while (1) // A Task shall never return or exit.
  {
    Serial.println("\n---- Task_retrieve_modbus_params ----");
    data_ready = false;
    // Issue the request

    vTaskDelay(pdMS_TO_TICKS(1000)); // 2000 / portTICK_PERIOD_MS);

    // Params : Token, Slave address, Function code, Register address, Length
    Error err = MB.addRequest(0, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_VOLTAGE, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(1, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_CURRENT, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(2, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_FREQ, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(3, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_ACTIV_POWER, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(4, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_REACT_POWER, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }  

    /*--------------------*/

    err = MB.addRequest(5, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_APPAR_POWER, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(6, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_SUM_POWER_FACTOR, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(7, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_PH1_POWER_FACTOR, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(8, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_TOT_POS_ACT_ENERGY, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(9, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_TOT_POS_REACT_ENERGY);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(10, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_TOT_NEG_ACT_ENERGY, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    /*--------------------*/

    err = MB.addRequest(11, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_TOT_NEG_REACT_ENERGY, 2);
    if (err!=SUCCESS) {
      ModbusError e(err);
      LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }

    // /*---------- LEGRAND ENERGY METER QUERIES ----------*/

    // // Params : Token, Slave address, Function code, Register address, Length
    // err = MB.addRequest(5, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_CURRENT_PH1, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(6, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_CURRENT_PH2, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(7, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_CURRENT_PH3, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(8, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_ACTIV_POWER_TOTAL, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(9, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_REACT_POWER_TOTAL, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(10, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_APPAR_POWER_TOTAL, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(11, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_VOLTAGE_PH1, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(12, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_VOLTAGE_PH2, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(13, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_VOLTAGE_PH3, 2);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(14, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_POWER_FACTOR_PH1, 1);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(15, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_POWER_FACTOR_PH2, 1);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }

    // /*--------------------*/

    // err = MB.addRequest(16, NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_POWER_FACTOR_PH3, 1);
    // if (err!=SUCCESS) {
    //   ModbusError e(err);
    //   LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    // }
  }
}

void Task_print_values(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  while (1) // A Task shall never return or exit.
  {
    if (ENABLE_SERIAL_PRINT)
    {
      Serial.println("\n---- Task_print_values ----");
      vTaskDelay(pdMS_TO_TICKS(1000)); // 2000 / portTICK_PERIOD_MS);

      Serial.printf("NRJ Voltage : %.2f V\n", nrj_values[0]);
      Serial.printf("NRJ Current : %.2f A\n", nrj_values[1]);
      Serial.printf("NRJ Frequency : %.2f Hz\n", nrj_values[2]);
      Serial.printf("NRJ Active Power : %.2f W\n", nrj_values[3]);
      Serial.printf("NRJ Reactive Power : %.2f VAR\n\n", nrj_values[4]);
      Serial.printf("NRJ Apparent Power : %.2f VAR\n\n", nrj_values[5]);
      Serial.printf("NRJ Sum Power Factor : %.2f\n\n", nrj_values[6]);
      Serial.printf("NRJ PH1 Power Factor : %.2f\n\n", nrj_values[7]);
      Serial.printf("NRJ Total Positive Active Energy : %.2f Wh\n\n", nrj_values[8]);
      Serial.printf("NRJ Total Positive Reactive Energy : %.2f Wh\n\n", nrj_values[9]);
      Serial.printf("NRJ Total Negative Active Energy : %.2f Wh\n\n", nrj_values[10]);
      Serial.printf("NRJ Total Negative Reactive Energy : %.2f Wh\n\n", nrj_values[11]);

    // Serial.print("unixtime : ");
    // Serial.println(now.unixtime(), DEC);
    // Serial.println();
    // Serial.print(now.year(), DEC);
    // Serial.print('/');
    // Serial.print(now.month(), DEC);
    // Serial.print('/');
    // Serial.print(now.day(), DEC);
    // Serial.print(" (");
    // Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    // Serial.print(") ");
    // Serial.print(now.hour(), DEC);
    // Serial.print(':');
    // Serial.print(now.minute(), DEC);
    // Serial.print(':');
    // Serial.println(now.second(), DEC);
    // Serial.println();

    // Serial.println("");

    Serial.println("Wait 2s");
    Serial.println("-------------------------------------------------------------------------------");
    }
  }
}

void Task_push_to_influxdb(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  // vTaskDelay(pdMS_TO_TICKS(1000)); // 2000 / portTICK_PERIOD_MS);

  while (1) // A Task shall never return or exit.
  {
    if (ENABLE_INFLUXDB)
    {
      Serial.println("\n---- Task_push_to_influxdb ----");
      // Print what are we exactly writing
      Serial.println("Writing : ");
      Serial.println(wifi_sensor.toLineProtocol());
      Serial.println(nrj_points.toLineProtocol());

      vTaskDelay(pdMS_TO_TICKS(1000)); // 2000 / portTICK_PERIOD_MS);

      // Clear fields for reusing the point. Tags will remain untouched
      wifi_sensor.clearFields();
      nrj_points.clearFields();

      // Store measured value into point
      // Report RSSI of currently connected network
      wifi_sensor.addField("rssi", WiFi.RSSI());

      nrj_points.addField("soco_volt", nrj_values[0]);
      nrj_points.addField("soco_amps", nrj_values[1]);
      nrj_points.addField("soco_freq", nrj_values[2]);
      nrj_points.addField("soco_activ_pow", nrj_values[3]);
      nrj_points.addField("soco_react_pow", nrj_values[4]);
      nrj_points.addField("soco_appart_pow", nrj_values[5]);
      nrj_points.addField("soco_sum_pow_fact", nrj_values[6]);
      nrj_points.addField("soco_ph1_pow_fact", nrj_values[7]);
      nrj_points.addField("soco_tot_pos_act_energy", nrj_values[8]);
      nrj_points.addField("soco_tot_pos_react_energy", nrj_values[9]);
      nrj_points.addField("soco_tot_neg_act_energy", nrj_values[10]);
      nrj_points.addField("soco_tot_neg_react_energy", nrj_values[11]);


      // Check WiFi connection and reconnect if needed
      if (wifiMulti.run() != WL_CONNECTED) {
        Serial.println("Wifi connection lost");
      }

      // Write point
      if (!influxDBclient.writePoint(wifi_sensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(influxDBclient.getLastErrorMessage());
        wifiMulti.run();
      }

      if (!influxDBclient.writePoint(nrj_points)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(influxDBclient.getLastErrorMessage());
        wifiMulti.run();
      }
    }
  }
}
