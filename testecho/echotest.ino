const int trigPin = 1;
const int echoPin = 2;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // 30 ms timeout
  if (duration == 0) {
    Serial.println("No echo");
  } else {
    float cm = duration * 0.0343 / 2;
    Serial.print(cm);
    Serial.println(" cm");
  }
  delay(100);
}