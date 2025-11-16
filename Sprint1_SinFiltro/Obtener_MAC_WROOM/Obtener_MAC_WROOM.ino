/*
 * OBTENER MAC ADDRESS - ESP32 WROOM
 * Código MÍNIMO para obtener la dirección MAC
 */

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("   OBTENIENDO MAC ADDRESS");
  Serial.println("========================================\n");
  
  // Inicializar WiFi
  WiFi.mode(WIFI_STA);
  delay(1000);
  
  // Obtener MAC
  String mac = WiFi.macAddress();
  
  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║  ESP32 WROOM (Transmisor/Guante)  ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.print("║  MAC: ");
  Serial.print(mac);
  Serial.println("          ║");
  Serial.println("╚════════════════════════════════════╝");
  
  Serial.println("\n📝 ANOTA ESTA DIRECCIÓN MAC\n");
  
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
  
  Serial.println("\n✓ Proceso completado");
  Serial.println("Puedes desconectar el ESP32 WROOM");
  Serial.println("y probar con el ESP32-S3");
}

void loop() {
  // Nada - todo en setup
}
