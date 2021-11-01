#define BAUDRATE 115200
#define READ_TIMEOUT 100

int i = 0; // Indice per i cicli/calcoli temporanei
char dato = 0; // Per letture su seriale
char buffer_length = 0; // Numero di byte della comunicazione seriale
char buffer[128]; // Buffer la comunicazione ricevuta da seriale
unsigned long int tempo = 0;
unsigned long int tempo_attesa = 0;

void wait_for_bytes(int num_bytes, unsigned long timeout) {
  Serial.flush();
  delay(10);  // Concedo del tempo per fare arrivare il dato nel buffer
  //tempo_attesa = millis();
  //while ((Serial.available() < num_bytes) && (millis() - tempo_attesa < timeout)) {}
}

void execute() {
  // Esecuzione dei comandi nel buffer
  // buffer[i] = id del pin
  // buffer[i+1] = comando da eseguire
  for (i = 0; i < buffer_length; i += 2) {
    // Esecuzion comando
    if (buffer[i + 1] > 0) {
      digitalWrite(LED_BUILTIN, HIGH); //Accendo il led
    } else if (buffer[i + 1] == 0) {
      digitalWrite(LED_BUILTIN, LOW); // Spengo il led
    }
  }
}

void serial_parse() {
  // Funzione per prelavare il payload della comunicazione dalla seriale e salvarlo nel buffer
  wait_for_bytes(1, READ_TIMEOUT);
  buffer_length = Serial.read(); // Il primo carattere della comunicazione è la quantità di byte
  Serial.write(buffer_length); // ACK lunghezza buffer
  for (i = 0; i < buffer_length; i++) {
    wait_for_bytes(1, READ_TIMEOUT);
    buffer[i] = Serial.read(); // Salvo tutte le informazioni utili
    Serial.write(buffer[i]); // ACK dato
  }
  wait_for_bytes(1, READ_TIMEOUT);
  dato = Serial.read();
  if (dato == 251) { // Carattere di fine comunicazione
    // Comunicazione non compromessa, il carattere di terminazione è corretto
    Serial.write(251); // ACK fine comunicazione
    execute();
  } else {
    // Comunicazione corrotta, scarto i dati nel buffer
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
    delay(100);
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
  // Serial.write(dato); // TODO: remove
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
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(BAUDRATE);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(900);
  Serial.flush();
  while (Serial.available()) Serial.read(); // Pulizia buffer seriale
  tempo = millis();
  digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
  if (Serial.available()) {
    serial_read();
  }
}
