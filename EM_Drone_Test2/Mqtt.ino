void mqttCallback(char* topic, byte* payload, unsigned int len) {
  if (String(topic) == topicReceive) {
    String text_data = payload;
    String data_json = text_data.substring(0, len);
    SerialMon.println(data_json);

    //char json[] =
    //  "{\"data\":[116537968,1049108170]}"; // {"data":[116537968,1049108170]}

    if (data_json.indexOf("data") > 0) {
      StaticJsonDocument<48> doc;

      DeserializationError error = deserializeJson(doc, data_json);

      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
      }

      unsigned long data_0 = doc["data"][0]; // "11.6539484"
      unsigned long data_1 = doc["data"][1]; // "104.9114010"

      x_des = data_0 / 10000000.0f;
      y_des = data_1 / 10000000.0f;

      Serial.print(x_des, 7);
      Serial.print(" , ");
      Serial.println(y_des, 7);
    }
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  //Connect to MQTT Broker
  //boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientName", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  //mqtt.publish(topicInit, "GsmClientTest started");
  mqtt.subscribe(topicReceive);
  return mqtt.connected();
}
