#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFi_credentials.h>
#include <EspDash.h>
#include <esp_task_wdt.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// #include <AsyncElegantOTA.h>
// #include <WebSerial.h>

#include "ModbusClientRTU.h"
#include "Logging.h"
#include "socomec_modbus_registers.h"

#include <InfluxDbClient.h>
#include "ca_certs.h"

// Dashboard login (OTA is behind it). Override in WiFi_credentials.h.
#ifndef DASH_USER
#define DASH_USER "admin"
#endif
#ifndef DASH_PASSWORD
#define DASH_PASSWORD "admin"
#endif

struct WifiNetwork { const char *ssid; const char *password; };
static const WifiNetwork wifi_networks[] = { WIFI_NETWORKS };

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
// ESP32 needs a POSIX TZ string, not an Olson name ("Africa/Casablanca" is ignored -> UTC).
#define TZ_INFO "<+01>-1"

#define MODBUS_DIR_PIN 4 // connect DR, RE pin of MAX485 to gpio 4
#define MODBUS_RX_PIN 16 // ESP32 Rx pin | Receiver Output (RO) of MAX485
#define MODBUS_TX_PIN 17 // ESP32 Tx pin | Driver Input (DI) of MAX485
#define MODBUS_BAUDRATE 9600 // 19200

#define MODBUS_NRJ_METER_ADDR 5

#define ENABLE_INFLUXDB 1
#define ENABLE_SERIAL_PRINT 1

bool data_ready = false;

uint32_t r_data1;
uint16_t r_data_twos_comp;
float r_datafloat1;

// #define NRJ_ARRAY_SIZE 12
// float nrj_values[NRJ_ARRAY_SIZE];
float nrj_values_flt[5];
int nrj_values_int[7];

// Create a ModbusRTU client instance
// In my case the RS485 module had auto halfduplex, so no second parameter with the DE/RE pin is required!
HardwareSerial Serial_RS485(2);
ModbusClientRTU MB(MODBUS_DIR_PIN);

// InfluxDB v2 client (Telegraf listener behind HTTPS), server cert verified against ISRG roots
InfluxDBClient influxDBclient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, INGEST_CA_CERTS);

// IoT schema v1 identifiers (see iot-schema/vocabulaire.yaml on the VPS).
// Any extra tag or field makes telegraf-iot reject the line into `iot_rejected`.
#define IOT_SITE        "home"
#define IOT_METER_TYPE  "energy_meter"
#define IOT_METER_ID    "socomec_main"
#define IOT_NODE_ID     "esp32_socomec"   // also used as device_id

// One bit per Modbus token: telemetry is only pushed once every register has been read,
// so the cumulative energy indexes never go back to 0 after a reboot.
#define MODBUS_ALL_TOKENS 0x0FFFu
volatile uint32_t modbus_received_mask = 0;

EspDash dash(80);

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
    nrj_values_flt[0] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 100;
    break;

  // 1 REG_SOCO_CURRENT
  case 1:
    nrj_values_flt[1] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 2 REG_SOCO_FREQ
  case 2:
    nrj_values_flt[2] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 3 REG_SOCO_ACTIV_POWER
  case 3:
    nrj_values_int[0] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 4 REG_SOCO_REACT_POWER
  case 4:
    nrj_values_int[1] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 5 REG_SOCO_APPAR_POWER
  case 5:
    nrj_values_int[2] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 6 REG_SOCO_SUM_POWER_FACTOR
  case 6:
    nrj_values_flt[3] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 7 REG_SOCO_PH1_POWER_FACTOR
  case 7:
    nrj_values_flt[4] = float((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 1000;
    break;

  // 8 REG_SOCO_TOT_POS_ACT_ENERGY
  case 8:
    nrj_values_int[3] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 9 REG_SOCO_TOT_POS_REACT_ENERGY
  case 9:
    nrj_values_int[4] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 10 REG_SOCO_TOT_NEG_ACT_ENERGY
  case 10:
    nrj_values_int[5] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  // 11 REG_SOCO_TOT_NEG_REACT_ENERGY
  case 11:
    nrj_values_int[6] = int((response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]) / 0.1;
    break;

  default:
    break;
  }

  if (token < 12) {
    modbus_received_mask |= (1u << token);
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

  dash.autoConnect();

  dash.begin(DASH_USER, DASH_PASSWORD);

  // ************************ InfluxDB init ************************ //
  // Setup wifi (STA only: AP_STA would expose an open soft-AP)
  WiFi.mode(WIFI_STA);

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

  for (const auto &net : wifi_networks) {
    wifiMulti.addAP(net.ssid, net.password);
  }

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

  // Accurate time is necessary for certificate validation and writing in batches
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Keep the TLS session open between writes (avoids a full handshake every second)
  influxDBclient.setHTTPOptions(HTTPOptions().connectionReuse(true));

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
                          16384, // TLS handshake needs headroom
                          NULL,
                          4,
                          NULL));

  // dash.handle() calls esp_task_wdt_reset() on every loop(): loopTask must be subscribed
  // to the task watchdog, otherwise it floods the UART with "task not found" errors.
  // 15 s leaves room for the dashboard's synchronous WiFi scan and OTA upload chunks.
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 15000,
    .idle_core_mask = (1 << 0), // keep the core default: watch CPU0 idle task only
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);
}

void loop() {
  // Modbus / print / InfluxDB are handled in their tasks.
  // The dashboard (incl. OTA on /update) uses the synchronous WebServer: it must be polled here.
  dash.handle();
  delay(2);
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

    err = MB.addRequest(9, MODBUS_NRJ_METER_ADDR, READ_HOLD_REGISTER, REG_SOCO_TOT_POS_REACT_ENERGY, 2);
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

      Serial.printf("NRJ Voltage : %.2f V\n", nrj_values_flt[0]);
      Serial.printf("NRJ Current : %.2f A\n", nrj_values_flt[1]);
      Serial.printf("NRJ Frequency : %.2f Hz\n", nrj_values_flt[2]);
      Serial.printf("NRJ Active Power : %d W\n", nrj_values_int[0]);
      Serial.printf("NRJ Reactive Power : %d VAR\n", nrj_values_int[1]);
      Serial.printf("NRJ Apparent Power : %d VAR\n", nrj_values_int[2]);
      Serial.printf("NRJ Sum Power Factor : %.2f\n", nrj_values_flt[3]);
      Serial.printf("NRJ PH1 Power Factor : %.2f\n", nrj_values_flt[4]);
      Serial.printf("NRJ Total Positive Active Energy : %d Wh\n", nrj_values_int[3]);
      Serial.printf("NRJ Total Positive Reactive Energy : %d Wh\n", nrj_values_int[4]);
      Serial.printf("NRJ Total Negative Active Energy : %d Wh\n", nrj_values_int[5]);
      Serial.printf("NRJ Total Negative Reactive Energy : %d Wh\n\n", nrj_values_int[6]);

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

// Append one IoT schema v1 line: <property>,site,asset_type,asset_id,data_kind value=<float>,device_id="..."
static void addSchemaLine(String &body, const char *property, const char *assetType,
                          const char *assetId, const char *dataKind, double value, int decimals)
{
  Point p(property);
  p.addTag("site", IOT_SITE);
  p.addTag("asset_type", assetType);
  p.addTag("asset_id", assetId);
  p.addTag("data_kind", dataKind);
  p.addField("value", value, decimals);
  p.addField("device_id", IOT_NODE_ID);
  body += p.toLineProtocol();
  body += '\n';
}

static void addMeterLine(String &body, const char *dataKind, double value, int decimals)
{
  addSchemaLine(body, "telemetry", IOT_METER_TYPE, IOT_METER_ID, dataKind, value, decimals);
}

static void addNodeLine(String &body, const char *dataKind, double value)
{
  addSchemaLine(body, "status", "node", IOT_NODE_ID, dataKind, value, 0);
}

void Task_push_to_influxdb(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  while (1) // A Task shall never return or exit.
  {
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!ENABLE_INFLUXDB)
      continue;

    Serial.println("\n---- Task_push_to_influxdb ----");

    String body;
    body.reserve(2600);

    // Node health (status)
    addNodeLine(body, "rssi", WiFi.RSSI());
    addNodeLine(body, "uptime", millis() / 1000UL);
    addNodeLine(body, "free_heap", ESP.getFreeHeap());

    // Meter telemetry, converted to the schema units (W, VA, var, kWh, kvarh)
    if (modbus_received_mask == MODBUS_ALL_TOKENS) {
      addMeterLine(body, "voltage",                nrj_values_flt[0], 2);
      addMeterLine(body, "current",                nrj_values_flt[1], 3);
      addMeterLine(body, "frequency",              nrj_values_flt[2], 3);
      addMeterLine(body, "power",                  nrj_values_int[0], 1);
      addMeterLine(body, "reactive_power",         nrj_values_int[1], 1);
      addMeterLine(body, "apparent_power",         nrj_values_int[2], 1);
      addMeterLine(body, "power_factor",           nrj_values_flt[3], 3);
      addMeterLine(body, "energy",                 nrj_values_int[3] / 1000.0, 3);
      addMeterLine(body, "reactive_energy",        nrj_values_int[4] / 1000.0, 3);
      addMeterLine(body, "energy_export",          nrj_values_int[5] / 1000.0, 3);
      addMeterLine(body, "reactive_energy_export", nrj_values_int[6] / 1000.0, 3);
    } else {
      Serial.printf("Modbus not complete yet (mask 0x%03X), telemetry skipped\n",
                    (unsigned)modbus_received_mask);
    }

    Serial.print("Writing :\n");
    Serial.print(body);

    // Check WiFi connection and reconnect if needed
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Wifi connection lost");
      continue;
    }

    // Single HTTPS POST for all lines
    if (!influxDBclient.writeRecord(body)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(influxDBclient.getLastErrorMessage());
    }
  }
}
