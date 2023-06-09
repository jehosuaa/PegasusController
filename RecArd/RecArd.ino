void setup() {
  Serial.begin(9600); // Inicializar el puerto serial a 9600 baudios
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readString();
    Serial.println(message); // Imprimir los datos recibidos en el monitor serial de Arduino IDE
  }
}
