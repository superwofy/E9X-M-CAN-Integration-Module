#define EXHAUST_FLAP_SOLENOID_PIN 17

void setup() 
{
  pinMode(EXHAUST_FLAP_SOLENOID_PIN, OUTPUT);
}

void loop() {
  digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, HIGH);
  Serial.println("Flap closed.");
  delay(4000);
  digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
  Serial.println("Flap opened.");
  delay(4000);
}