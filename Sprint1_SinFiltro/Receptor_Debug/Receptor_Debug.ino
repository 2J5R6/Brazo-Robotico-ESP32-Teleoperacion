/*
 * RECEPTOR - VERSION DEBUG
 * Código con diagnóstico paso a paso para encontrar dónde se cuelga
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Pines
#define SERVO1_PIN 6
#define SERVO2_PIN 7
#define LED_PIN 48

Servo servo1;
Servo servo2;

void setup() {
  // Serial con inicialización robusta
  Serial.begin(115200);
  delay(2000);
  
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("   RECEPTOR - VERSION DEBUG");
  Serial.println("========================================");
  Serial.println();
  
  // LED
  Serial.print("Paso 1: Configurando LED... ");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("OK");
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  // Servos - ESTA PARTE PUEDE CAUSAR PROBLEMAS
  Serial.print("Paso 2: Configurando Servo1 (GPIO");
  Serial.print(SERVO1_PIN);
  Serial.print(")... ");
  Serial.flush();  // Forzar salida antes de posible cuelgue
  
  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo1.write(90);
  
  Serial.println("OK");
  
  Serial.print("Paso 3: Configurando Servo2 (GPIO");
  Serial.print(SERVO2_PIN);
  Serial.print(")... ");
  Serial.flush();
  
  ESP32PWM::allocateTimer(1);
  servo2.setPeriodHertz(50);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo2.write(90);
  
  Serial.println("OK");
  
  // WiFi
  Serial.print("Paso 4: Configurando WiFi... ");
  Serial.flush();
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println("OK");
  Serial.print("  MAC: ");
  Serial.println(WiFi.macAddress());
  
  // ESP-NOW - ESTA PARTE TAMBIÉN PUEDE CAUSAR PROBLEMAS
  Serial.print("Paso 5: Inicializando ESP-NOW... ");
  Serial.flush();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("FALLO!");
    Serial.println("  ERROR: esp_now_init() falló");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  Serial.println("OK");
  
  // Registro callback
  Serial.print("Paso 6: Registrando callback... ");
  Serial.flush();
  
  esp_now_register_recv_cb(onDataRecv);
  
  Serial.println("OK");
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("   SISTEMA LISTO");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Esperando datos por ESP-NOW...");
  Serial.println("LED debe encenderse al recibir datos");
  Serial.println();
  
  // LED indica listo
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // Imprimir que estamos vivos cada 5 segundos
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    Serial.print("Sistema activo | Tiempo: ");
    Serial.print(millis() / 1000);
    Serial.println(" s");
  }
  
  delay(100);
}

// Callback simplificado
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Serial.println("¡Datos recibidos!");
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}
