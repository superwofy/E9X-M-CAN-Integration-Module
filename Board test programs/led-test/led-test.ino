#define POWER_LED_PIN 3
#define FOG_LED_PIN 4

void setup() 
{
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(FOG_LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(POWER_LED_PIN, HIGH);
  digitalWrite(FOG_LED_PIN, HIGH);
  Serial.println("LEDs ON");
  delay(4000);
  digitalWrite(POWER_LED_PIN, LOW);
  digitalWrite(FOG_LED_PIN, LOW);
  Serial.println("LEDs OFF");
  delay(4000);
}