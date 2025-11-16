/*
 * OBTENER MAC ADDRESS - ESP32-S3
 * Código MÍNIMO para obtener la dirección MAC
 */

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(3000);  // ESP32-S3 necesita más tiempo
  
  // Esperar que Serial esté listo
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  
  Serial.println("\n\n\n");
  Serial.println("========================================");
  Serial.println("   OBTENIENDO MAC ADDRESS");
  Serial.println("========================================");
  Serial.println();
  
  // Parpadear LED como confirmación
  pinMode(48, OUTPUT);
  for(int i = 0; i < 5; i++) {
    digitalWrite(48, HIGH);
    delay(100);
    digitalWrite(48, LOW);
    delay(100);
  }
  
  Serial.println("LED parpadeó 5 veces");
  Serial.println();
  
  // Inicializar WiFi
  Serial.print("Iniciando WiFi... ");
  Serial.flush();
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.println("OK");
  
  // Obtener MAC
  String mac = WiFi.macAddress();
  
  Serial.println();
  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║  ESP32-S3 (Receptor/Brazo)        ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.print("║  MAC: ");
  Serial.print(mac);
  Serial.println("          ║");
  Serial.println("╚════════════════════════════════════╝");
  
  Serial.println();
  Serial.println("📝 ANOTA ESTA DIRECCIÓN MAC");
  Serial.println();
  
  Serial.print("MAC en bytes (para código): {");
  uint8_t macBytes[6];
  WiFi.macAddress(macBytes);
  for(int i = 0; i < 6; i++) {
    Serial.print("0x");
    if(macBytes[i] < 16) Serial.print("0");
    Serial.print(macBytes[i], HEX);
    if(i < 5) Serial.print(", ");
  }
  Serial.println("}");
  
  Serial.println();
  Serial.println("✓ Proceso completado");
  Serial.println("Ahora tienes ambas MACs para el proyecto");
  Serial.println();
}

void loop() {
  static unsigned long lastMsg = 0;
  
  if(millis() - lastMsg > 5000) {
    lastMsg = millis();
    Serial.println("Sistema activo - MAC obtenida exitosamente");
  }
  
  delay(100);
}
