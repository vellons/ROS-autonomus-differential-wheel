void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  Serial.println("CIAO!");
}

void loop() {
  if (Serial.available() > 0) {
    char state = Serial.read();
    // Serial.println(state);
    if (state == 48) { // 0 char
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (state == 49) { // 1 char
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (state != 10) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
