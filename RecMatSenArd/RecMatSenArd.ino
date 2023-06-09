void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // Inicializar el puerto Serial1
}

void sendToArduino(const String& message) {
  Serial.println("Enviando a otro Arduino: " + message);
  Serial1.println(message); // Enviar el mensaje a otro Arduino a través de Serial1
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readString();
    Serial.println(message); // Imprimir los datos recibidos en el monitor serial de Arduino IDE
    sendToArduino(message); // Enviar el mensaje a otro Arduino
  }
}
