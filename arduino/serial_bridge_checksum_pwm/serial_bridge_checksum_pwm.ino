#define BAUDRATE 115200
#define READ_TIMEOUT 100

// PIN - LEFT MOTOR (Arduino motor shield - L298P)
#define PIN_DIRECTION_SX 12
#define PIN_BRAKE_SX 9
#define PIN_SPEED_SX 3

// PIN - RIGHT MOTOR (Arduino motor shield - L298P)
#define PIN_DIRECTION_DX 13
#define PIN_BRAKE_DX 8
#define PIN_SPEED_DX 11

int i = 0; // Indice per i cicli/calcoli temporanei
byte dato = 0; // Per letture su seriale
byte buffer_length = 0; // Numero di byte della comunicazione seriale
byte buffer[128]; // Buffer la comunicazione ricevuta da seriale
byte motor = 0; // Usato per calcoli
unsigned int checksum = 0; // Variabile per calcolare il checksum
bool checksum_correct = false; // Stato ultimo checksum
unsigned long int tempo = 0;
unsigned long int tempo_attesa = 0;

#define NPIN 6
const byte pin[NPIN] = {
  PIN_DIRECTION_SX,
  PIN_BRAKE_SX,
  PIN_SPEED_SX,
  PIN_DIRECTION_DX,
  PIN_BRAKE_DX,
  PIN_SPEED_DX
};

void wait_for_bytes(int num_bytes, unsigned long timeout) {
  tempo_attesa = millis();
  while ((Serial.available() < num_bytes) && (millis() - tempo_attesa < timeout)) {}
}

void execute() {
  // Esecuzione dei comandi nel buffer
  // buffer[0] = direction of SX motor (0 = brake, 1=forward, 2=backward)
  // buffer[1] = PWM of SX motor
  for (i = 0; i < buffer_length; i += 2) {
    if (i == 0 || i == 2) { // SX or  DX motor data
      motor = (i == 0) ? 0 : 1;
      if (buffer[i] == 1 || buffer[i] == 2) { // Direction
        digitalWrite(pin[motor * 3], buffer[i] == 1 ? HIGH : LOW); // Direction
        digitalWrite(pin[motor * 3 + 1], LOW); // No brake
        analogWrite(pin[motor * 3 + 2], buffer[i + 1]); // PWM (speed)
      } else if (buffer[i] == 0) {
        digitalWrite(pin[motor * 3 + 1], HIGH); // Brake
        analogWrite(pin[motor * 3 + 2], 0);
      }
    }
  }
}

void serial_parse() {
  // Funzione per prelavare il payload della comunicazione dalla seriale e salvarlo nel buffer
  wait_for_bytes(1, READ_TIMEOUT);
  buffer_length = Serial.read(); // Il primo carattere della comunicazione è la quantità di byte
  checksum = buffer_length;
  Serial.write(buffer_length); // Conferma lunghezza buffer

  for (i = 0; i < buffer_length; i++) {
    wait_for_bytes(1, READ_TIMEOUT);
    buffer[i] = Serial.read(); // Salvo tutte le informazioni utili
    Serial.write(buffer[i]); // Conferma dato
    checksum = (checksum + buffer[i]) % 256; // CHECKSUM
  }

  // Check checksum
  checksum_correct = false;
  wait_for_bytes(1, READ_TIMEOUT);
  dato = Serial.read();
  Serial.write(checksum);
  if (dato == checksum) {
    checksum_correct = true;
  }

  // Chiusura della comunicazione
  wait_for_bytes(1, READ_TIMEOUT);
  dato = Serial.read();
  if (dato == 251) { // Carattere di fine comunicazione
    // Comunicazione non compromessa, il carattere di terminazione è corretto
    Serial.write(251); // Conferma fine comunicazione
    if (checksum_correct) {
      // Se il checksum è verificato eseguo il messaggio
      execute();
    }
  } else {
    // Comunicazione corrotta, scarto i dati nel buffer
    while (Serial.available()) {
      dato = Serial.read();
    }
  }
}

void serial_read() {
  // Funzione per sincronizzarsi con l'inizio della comunicazione seriale
  // Dopo i caratteri di sincronizzazione si chiama serial_parse()
  // per prendere il payload della comunicazione
  wait_for_bytes(1, READ_TIMEOUT);
  dato = Serial.read();
  if (dato == 254) {
    Serial.write(254);
    wait_for_bytes(1, READ_TIMEOUT);
    dato = Serial.read();
    if (dato == 253) {
      Serial.write(253);
      wait_for_bytes(1, READ_TIMEOUT);
      dato = Serial.read();
      if (dato == 252) {
        Serial.write(252);
        serial_parse();
      }
    }
  } else if (dato == 250) { // Ping only
    Serial.write(250);
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_DIRECTION_SX, OUTPUT);
  pinMode(PIN_BRAKE_SX, OUTPUT);
  pinMode(PIN_SPEED_SX, OUTPUT);
  pinMode(PIN_DIRECTION_DX, OUTPUT);
  pinMode(PIN_BRAKE_DX, OUTPUT);
  pinMode(PIN_SPEED_DX, OUTPUT);
  Serial.begin(BAUDRATE);

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(PIN_BRAKE_SX, HIGH); // Brake
  digitalWrite(PIN_BRAKE_DX, HIGH); // Brake
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  Serial.flush();
  while (Serial.available()) Serial.read(); // Pulizia buffer seriale
  tempo = millis();
  digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
  if (Serial.available()) {
    serial_read();
    tempo = millis();
  } else if (digitalRead(PIN_BRAKE_DX) == HIGH && millis() - tempo > 3000) {
    digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // Blink led
    delay(33);
    digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // Blink led
    tempo = millis();
  }
}
