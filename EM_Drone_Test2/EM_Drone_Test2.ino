#include <mavlink.h>
#include <ArduinoJson.h>

#define TINY_GSM_MODEM_SIM868

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define SerialMon Serial
//#define SerialAT Serial2
#define SerialAT Serial1
#define SerialTEL Serial2
#define TINY_GSM_DEBUG SerialMon

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient mqtt(client);

#define LED_PIN 13
#define pwrkey 5
#define Status 4

// Your GPRS credentials, if any
const char apn[] = "smart";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
const char* broker = "iotlab.aenccambodia.com";
const char* mqttUsername = "drone-AwhL:drone-AwhL";  // MQTT username
const char* mqttPassword = "UEfDe8sJnYaw";  // MQTT password

const char* topicReceive = "EMDrone/ServerToDrone";
const char* topicRequest = "EMDrone/DroneRequest";

uint32_t lastReconnectAttempt = 0;

///////////////////

uint16_t mission_num = 7;
float x_home = 11.6539484;
float y_home = 104.9114010;

float x_des ;//= 11.6537968;
float y_des ;//= 104.9108170;

float x_cur;
float y_cur;

boolean MissionUpload_SUCCESSFULL = false;
boolean t = false;

void setup() {
  SerialTEL.begin(57600);
  SerialMon.begin(115200);
  SerialAT.begin(115200);

  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }
#endif

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  //  char json[] =
  //    "{\"data\":[116537968,1049108170]}";
  //  StaticJsonDocument<48> doc;
  //
  //  DeserializationError error = deserializeJson(doc, json);
  //
  //  if (error) {
  //    Serial.print(F("deserializeJson() failed: "));
  //    Serial.println(error.c_str());
  //    return;
  //  }

  //  unsigned long data_0 = doc["data"][0]; // "11.6539484"
  //  unsigned long data_1 = doc["data"][1]; // "104.9114010"

  //  x_des = data_0 / 10000000.0f;
  //  y_des = data_1 / 10000000.0f;

  //  Serial.print(x_des, 7);
  //  Serial.print(" , ");
  //  Serial.println(y_des, 7);

  //mission_count(mission_num);
  //Command_long_ARM(0);

}

void loop() {
  MavLink_receive();
  if (SerialMon.available()) {
    String text = SerialMon.readString();
    if (text == "start") {
      SerialMon.println("START");
      request_datastream(1);
    } else if (text == "stop") {
      SerialMon.println("STOP");
      request_datastream(0);
    }
  }
  //    delay(100);
  //    Command_long_ARM(1);
  //    delay(3000);
  //    setmode_Auto();

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  mqtt.loop();

}
