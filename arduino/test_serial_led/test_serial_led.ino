byte state = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("CIAO");
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  /*for (int i = 120; i < 260; i++) {
    Serial.write(i);
    delay(5);
  }*/
}

void loop() {
  if (Serial.available() > 0) {
    state = Serial.read();
    //Serial.print(state, DEC);
    Serial.write(state);
    if (state == 48) { // 0 char
      digitalWrite(LED_BUILTIN, LOW);
    } else if (state == 49 || state == 250) { // 1 char
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
