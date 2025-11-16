/*
 * TEST SERIAL ESP32-S3
 * Código mínimo para verificar que Serial funciona
 */

void setup() {
  // Inicializar Serial con delay largo
  Serial.begin(115200);
  delay(3000);  // Esperar 3 segundos
  
  // Configurar LED
  pinMode(48, OUTPUT);
  
  // Parpadeo inicial
  for(int i = 0; i < 10; i++) {
    digitalWrite(48, HIGH);
    delay(100);
    digitalWrite(48, LOW);
    delay(100);
  }
  
  // Enviar mensajes
  Serial.println();
  Serial.println("========================================");
  Serial.println("   TEST SERIAL ESP32-S3");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Si ves esto, el Serial FUNCIONA!");
  Serial.println();
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("Frecuencia CPU: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.print("Flash: ");
  Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
  Serial.println(" MB");
  Serial.println();
  Serial.println("LED parpadeando cada 1 segundo...");
  Serial.println("Contador en Serial cada 1 segundo...");
  Serial.println();
}

void loop() {
  static int counter = 0;
  
  // Parpadear LED
  digitalWrite(48, HIGH);
  delay(500);
  digitalWrite(48, LOW);
  delay(500);
  
  // Imprimir contador
  Serial.print("Contador: ");
  Serial.print(counter++);
  Serial.print(" | Tiempo: ");
  Serial.print(millis() / 1000);
  Serial.println(" segundos");
}
