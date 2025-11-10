/*
 * SPRINT 1 - RECEPTOR (ESP32-S3 en Brazo Robótico)
 * Sistema de Tele-operación Brazo Robótico 2DOF
 * 
 * Este código:
 * 1. Recibe datos de la IMU por ESP-NOW
 * 2. Controla 2 servomotores MG90S según la posición de la mano
 * 3. Implementa lógica de selección de servo según eje Z
 * 
 * LÓGICA DE CONTROL:
 * - Mano ABAJO (handPosition=0): Controla SERVO1 (base del brazo)
 * - Mano ARRIBA (handPosition=1): Controla SERVO2 (extremo del brazo)
 * 
 * HARDWARE:
 * - ESP32-S3 (siempre va con el brazo robótico)
 * - 2x Servomotores MG90S
 * - IMU MPU6050 en el extremo del brazo (opcional para feedback)
 * 
 * CONEXIONES ESP32-S3:
 * - Servo1 (Base): GPIO 12
 * - Servo2 (Extremo): GPIO 13
 * - LED indicador: GPIO 48
 * - I2C (opcional): SDA=GPIO 8, SCL=GPIO 10 (NO usar GPIO 9)
 * 
 * REGLAS GPIO ESP32-S3:
 * ✅ Usar GPIO 1-10, 12-15, 21, 45-48 para OUTPUT
 * ❌ NUNCA usar GPIO 19/20 (USB D-/D+)
 * ❌ GPIO 9 tiene problemas - usar GPIO 10 para SCL
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// ========== CONFIGURACIÓN DE PINES ==========
#define SERVO1_PIN 12   // Servo base (se activa cuando mano está abajo)
#define SERVO2_PIN 13   // Servo extremo (se activa cuando mano está arriba)
#define LED_PIN 48      // LED integrado ESP32-S3

// ========== OBJETOS SERVO ==========
Servo servo1;  // Base
Servo servo2;  // Extremo

// ========== ESTRUCTURA DE DATOS ==========
typedef struct struct_message {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  unsigned long timestamp;
  uint8_t handPosition;  // 0=abajo (servo1), 1=arriba (servo2)
} struct_message;

struct_message receivedData;

// ========== VARIABLES DE CONTROL ==========
int servo1Position = 90;  // Posición central inicial
int servo2Position = 90;  // Posición central inicial
uint8_t activeServo = 1;  // Por defecto, servo2 está activo (extremo)
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT = 500;  // Timeout de 500ms sin datos

// Variables para mapeo de movimiento
// Espacio de trabajo del operador: ±4G en X e Y
const float ACCEL_MIN = -4.0;  // -4G
const float ACCEL_MAX = 4.0;   // +4G
const int SERVO_MIN = 0;       // Ángulo mínimo servo
const int SERVO_MAX = 180;     // Ángulo máximo servo

// ========== CALLBACK ESP-NOW ==========
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copiar datos recibidos
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();
  
  // Actualizar servo activo según posición de la mano
  activeServo = receivedData.handPosition;
  
  // Parpadear LED al recibir datos
  digitalWrite(LED_PIN, HIGH);
  
  // Mapear aceleraciones a ángulos de servos
  // AccelX controla el movimiento principal del servo activo
  // AccelY puede usarse para refinamiento o segundo eje
  
  if (activeServo == 0) {
    // Mano ABAJO - Controlar SERVO1 (base)
    servo1Position = map(receivedData.accelX * 100, -400, 400, SERVO_MIN, SERVO_MAX);
    servo1Position = constrain(servo1Position, SERVO_MIN, SERVO_MAX);
    servo1.write(servo1Position);
    
  } else {
    // Mano ARRIBA - Controlar SERVO2 (extremo)
    servo2Position = map(receivedData.accelX * 100, -400, 400, SERVO_MIN, SERVO_MAX);
    servo2Position = constrain(servo2Position, SERVO_MIN, SERVO_MAX);
    servo2.write(servo2Position);
  }
  
  // Debug por serial (cada 10 recepciones)
  static int debugCounter = 0;
  if (debugCounter++ >= 10) {
    debugCounter = 0;
    Serial.print("Recibido | AccelX: "); Serial.print(receivedData.accelX, 2);
    Serial.print(" | AccelY: "); Serial.print(receivedData.accelY, 2);
    Serial.print(" | AccelZ: "); Serial.print(receivedData.accelZ, 2);
    Serial.print(" | Mano: ");
    Serial.print(activeServo == 1 ? "ARRIBA" : "ABAJO");
    Serial.print(" | Servo activo: "); Serial.print(activeServo == 0 ? "1" : "2");
    Serial.print(" | Ángulo: ");
    Serial.println(activeServo == 0 ? servo1Position : servo2Position);
  }
}

// ========== SETUP ==========
void setup() {
  // Inicializar Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SPRINT 1 - RECEPTOR (Brazo Robótico) ===");
  Serial.println("ESP32-S3 con 2x Servos MG90S + ESP-NOW");
  
  // Configurar LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Configurar e inicializar servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  servo1.setPeriodHertz(50);  // Frecuencia estándar para servos
  servo2.setPeriodHertz(50);
  
  servo1.attach(SERVO1_PIN, 500, 2400);  // Min/Max pulse width para MG90S
  servo2.attach(SERVO2_PIN, 500, 2400);
  
  // Posición inicial (centro)
  servo1.write(servo1Position);
  servo2.write(servo2Position);
  
  Serial.println("✓ Servos inicializados");
  Serial.println("  Servo1 (Base): GPIO12");
  Serial.println("  Servo2 (Extremo): GPIO13");
  Serial.println("  Posición inicial: 90°");
  
  // Configurar WiFi en modo estación
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("✓ MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("✗ ERROR: Fallo al inicializar ESP-NOW");
    return;
  }
  
  Serial.println("✓ ESP-NOW inicializado");
  
  // Registrar callback de recepción
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("\n=== Sistema listo ===");
  Serial.println("Esperando datos del guante...\n");
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

// ========== LOOP ==========
void loop() {
  // Verificar timeout (sin datos recibidos)
  if (millis() - lastReceiveTime > TIMEOUT && lastReceiveTime > 0) {
    // Timeout - volver a posición de seguridad
    static bool timeoutWarning = false;
    if (!timeoutWarning) {
      Serial.println("⚠ TIMEOUT: Sin datos del transmisor");
      Serial.println("  Volviendo a posición de seguridad (90°)");
      timeoutWarning = true;
    }
    
    // Mover gradualmente a posición central
    if (servo1Position != 90) {
      servo1Position += (servo1Position < 90) ? 1 : -1;
      servo1.write(servo1Position);
    }
    if (servo2Position != 90) {
      servo2Position += (servo2Position < 90) ? 1 : -1;
      servo2.write(servo2Position);
    }
    
    delay(20);
  }
  
  // Apagar LED después de recibir
  static unsigned long ledTime = 0;
  if (digitalRead(LED_PIN) == HIGH && millis() - lastReceiveTime > 50) {
    digitalWrite(LED_PIN, LOW);
  }
  
  delay(10);
}
