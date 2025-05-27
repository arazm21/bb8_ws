// #include <Arduino.h>
// #define led_pin 2
// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   // int result = myFunction(2, 3);
//   pinMode(led_pin, OUTPUT);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   digitalWrite(led_pin,HIGH);
//   delay(500);
//   digitalWrite(led_pin,LOW);
//   delay(500);
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "your-ssid";
const char* password = "your-password";
const char* mqtt_server = "raspberry-pi-ip";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) {
    while (!client.connected()) {
      if (client.connect("ESP32Client")) {
        client.subscribe("topic");
      } else {
        delay(5000);
      }
    }
  }
  client.loop();
}
