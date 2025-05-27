#include <Arduino.h>

#define led_pin 2
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:

  // int result = myFunction(2, 3);

  pinMode(led_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led_pin, HIGH);
  delay(1000);
  digitalWrite(led_pin, LOW);
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}