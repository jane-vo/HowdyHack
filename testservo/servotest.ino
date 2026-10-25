#include <ESP32Servo.h>

Servo myServo;

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(4, 500, 2400);
  Serial.println("Servo test started");
  myServo.write(90);
  delay(1000);
}

void loop() {
  Serial.println("hello");
  delay(1000);
  myServo.write(45);
  Serial.println("45");
  delay(1000);
  myServo.write(135);
  Serial.println("135");
  delay(1000);
}