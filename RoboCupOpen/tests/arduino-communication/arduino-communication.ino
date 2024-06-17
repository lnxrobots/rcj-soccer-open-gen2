#include <ArduinoJson.h>

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(115200);
    while (!Serial) continue;
}

void loop() {
    StaticJsonDocument<24> doc_in;
    DeserializationError error = deserializeJson(doc_in, Serial);
    digitalWrite(13, doc_in["light"]);

    // StaticJsonDocument<32> doc_out;
    // doc_out["result"] = "done";
    // serializeJson(doc_out, Serial);
    // Serial.write('\n');
    Serial.println("{\"result\":\"done\"}");
}
