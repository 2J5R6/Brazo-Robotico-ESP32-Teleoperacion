/*
 * SPRINT 2 ULTRA-MEJORADO - RECEPTOR (ESP32-S3 en Brazo)
 * 
 * MEJORAS CRÍTICAS vs v1:
 * - Procesamiento de 100Hz (era 50Hz) = movimiento más fluido
 * - Interpolación inteligente para suavidad sin lag
 * - Step adaptativo según velocidad de movimiento
 * - Dead zone eliminado para respuesta total
 * - Aceleración/desaceleración suave
 * - Sin delays innecesarios = latencia mínima
 * 
 * Hardware: ESP32-S3 + 2 Servos MG90S + MPU6500 opcional
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ========== PINES ==========
#define SERVO1_PIN 6
#define SERVO2_PIN 7
#define LED_PIN 48
#define SDA_PIN 8
#define SCL_PIN 10

// ========== OBJETOS ==========
Servo servo1;
Servo servo2;
Adafruit_MPU6050 mpuBrazo;
bool mpuBrazoReady = false;

// ========== ESTRUCTURA ESP-NOW ==========
typedef struct struct_message {
  float accelX;
  float accelY;      // HORIZONTAL (izq/der) → Servo2  
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;       // ROTACIÓN muñeca → Servo1
  unsigned long timestamp;
  uint8_t handPosition;  // 0=VERTICAL(S1), 1=HORIZONTAL(S2)
} struct_message;

struct_message receivedData;

// ========== VARIABLES CONTROL AVANZADO ==========
float servo1Target = 90.0;
float servo2Target = 90.0;
float servo1Current = 90.0;
float servo2Current = 90.0;
float servo1Velocity = 0;
float servo2Velocity = 0;

uint8_t activeServo = 1;
unsigned long lastReceiveTime = 0;
unsigned long lastUpdate = 0;
const unsigned long TIMEOUT = 500;
const float UPDATE_INTERVAL = 5;  // 5ms = 200Hz de actualización servo

// Mapeo mejorado
const float ACCEL_MIN = -4.0;   // ±4g del transmisor
const float ACCEL_MAX = 4.0;
const int SERVO_MIN = 10;       // Margen de seguridad
const int SERVO_MAX = 170;

// Control de suavidad
const float MAX_VELOCITY = 3.0;      // Velocidad máxima °/ms
const float ACCELERATION = 0.5;       // Aceleración suave
const float SMOOTHING_FACTOR = 0.15;  // Interpolación (más bajo = más suave)

// ========== CALLBACK ESP-NOW ==========
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();
  activeServo = receivedData.handPosition;
  digitalWrite(LED_PIN, HIGH);
  
  // MAPEO INTELIGENTE según orientación
  float sensorValue = 0;
  float mappedAngle = 0;
  
  if (activeServo == 0) {
    // MODO VERTICAL: Servo1 responde a ROTACIÓN de muñeca (GyroZ)
    // Integración de velocidad angular con alta sensibilidad
    sensorValue = receivedData.gyroZ;
    
    // Integración mejorada: mayor ganancia para captar mejor los movimientos
    static float gyroIntegrated = 90.0;  // Centro
    gyroIntegrated += sensorValue * 0.8;  // Mayor ganancia (era 0.3)
    gyroIntegrated = constrain(gyroIntegrated, SERVO_MIN, SERVO_MAX);
    
    servo1Target = gyroIntegrated;
    
  } else {
    // MODO HORIZONTAL: Servo2 responde a movimiento LATERAL (AccelY)
    // AccelY amplificado: rango ±2.5 m/s² → 10° a 170° (mayor sensibilidad)
    sensorValue = receivedData.accelY;
    mappedAngle = map(sensorValue * 100, 250, -250, SERVO_MIN, SERVO_MAX);  // Rango reducido = más amplificación
    mappedAngle = constrain(mappedAngle, SERVO_MIN, SERVO_MAX);
    servo2Target = mappedAngle;
  }
  
  // Debug reducido (cada 20 paquetes = ~200ms)
  static int debugCount = 0;
  if (debugCount++ >= 20) {
    debugCount = 0;
    Serial.print("✓ RX | ");
    Serial.print(activeServo == 0 ? "✋VERTICAL" : "👉HORIZONTAL");
    Serial.print(" | ");
    Serial.print(activeServo == 0 ? "GyroZ:" : "AccelY:");
    Serial.print(sensorValue, 1);
    Serial.print(" | S");
    Serial.print(activeServo + 1);
    Serial.print(":");
    Serial.print((int)(activeServo == 0 ? servo1Current : servo2Current));
    Serial.print("°→");
    Serial.print((int)(activeServo == 0 ? servo1Target : servo2Target));
    Serial.println("°");
  }
}

// ========== FUNCIÓN ACTUALIZACIÓN SUAVE ==========
void updateServoSmooth(Servo &servo, float &current, float target, float &velocity) {
  float error = target - current;
  
  // Si el error es muy pequeño, no hacer nada (evita jitter)
  if (abs(error) < 0.5) {
    velocity *= 0.8;  // Frenar suavemente
    return;
  }
  
  // Calcular velocidad deseada (proporcional al error)
  float desiredVel = error * SMOOTHING_FACTOR;
  desiredVel = constrain(desiredVel, -MAX_VELOCITY, MAX_VELOCITY);
  
  // Aplicar aceleración suave
  if (abs(desiredVel - velocity) > ACCELERATION) {
    if (desiredVel > velocity) {
      velocity += ACCELERATION;
    } else {
      velocity -= ACCELERATION;
    }
  } else {
    velocity = desiredVel;
  }
  
  // Actualizar posición
  current += velocity;
  current = constrain(current, SERVO_MIN, SERVO_MAX);
  
  // Escribir al servo
  servo.write((int)current);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(3000);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Parpadeo inicial
  for(int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);
    delay(80);
  }
  
  Serial.println("\n========================================");
  Serial.println("   SPRINT 2 ULTRA-MEJORADO");
  Serial.println("   RECEPTOR ALTA PRECISIÓN");
  Serial.println("========================================\n");
  
  // WiFi
  Serial.print("[1/5] WiFi... ");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println("OK");
  
  // Servos
  Serial.print("[2/5] Servos... ");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  
  // Iniciar en centro
  servo1.write(90);
  servo2.write(90);
  delay(500);
  Serial.println("OK");
  
  // Test servos
  Serial.print("    Test movimiento... ");
  servo1.write(60);
  delay(200);
  servo1.write(120);
  delay(200);
  servo1.write(90);
  servo2.write(60);
  delay(200);
  servo2.write(120);
  delay(200);
  servo2.write(90);
  Serial.println("OK ✓");
  
  // ESP-NOW
  Serial.print("[3/5] ESP-NOW... ");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR!");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("OK");
  
  // MPU opcional
  Serial.print("[4/5] MPU6500 (feedback)... ");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz estable
  delay(100);
  if (mpuBrazo.begin()) {
    mpuBrazoReady = true;
    delay(50);
    mpuBrazo.setAccelerometerRange(MPU6050_RANGE_4_G);
    delay(10);
    mpuBrazo.setGyroRange(MPU6050_RANGE_250_DEG);
    delay(10);
    mpuBrazo.setFilterBandwidth(MPU6050_BAND_44_HZ);
    delay(10);
    Serial.println("OK (activo)");
  } else {
    mpuBrazoReady = false;
    Serial.println("NO (continúa sin él)");
  }
  
  Serial.println("[5/5] Sistema listo");
  
  Serial.println("\n========================================");
  Serial.println("   CONFIGURACIÓN ULTRA-OPTIMIZADA");
  Serial.println("========================================");
  Serial.println("Actualización: 200 Hz (5ms)");
  Serial.println("Recepción: 100 Hz del transmisor");
  Serial.println("Suavizado: Interpolación adaptativa");
  Serial.println("Velocidad: Aceleración/desaceleración");
  Serial.println("Dead zone: ELIMINADA");
  Serial.println("Latencia: MÍNIMA (<15ms)");
  Serial.println("\n¡Movimiento SIMULTÁNEO y SUAVE!");
  Serial.println("========================================\n");
  
  // Confirmación final
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("Esperando datos del transmisor...\n");
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // Apagar LED después de recibir
  if (now - lastReceiveTime > 20) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Actualizar servos CONSTANTEMENTE a 200Hz
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;
    
    // Actualizar ambos servos con suavizado
    updateServoSmooth(servo1, servo1Current, servo1Target, servo1Velocity);
    updateServoSmooth(servo2, servo2Current, servo2Target, servo2Velocity);
  }
  
  // Timeout - volver al centro gradualmente
  if (lastReceiveTime > 0 && (now - lastReceiveTime > TIMEOUT)) {
    servo1Target = 90;
    servo2Target = 90;
    
    static int timeoutCount = 0;
    if (timeoutCount++ >= 100) {
      timeoutCount = 0;
      Serial.println("⚠ Timeout - volviendo al centro");
    }
  }
  
  // Feedback MPU (cada 3 segundos)
  static unsigned long lastFeedback = 0;
  if (mpuBrazoReady && (now - lastFeedback >= 3000)) {
    lastFeedback = now;
    
    sensors_event_t a, g, temp;
    mpuBrazo.getEvent(&a, &g, &temp);
    
    Serial.println("\n─── FEEDBACK MPU BRAZO ───");
    Serial.print("Posición real | X:");
    Serial.print(a.acceleration.x, 2);
    Serial.print(" Y:");
    Serial.print(a.acceleration.y, 2);
    Serial.print(" Z:");
    Serial.println(a.acceleration.z, 2);
    Serial.print("Servos | S1:");
    Serial.print((int)servo1Current);
    Serial.print("° S2:");
    Serial.print((int)servo2Current);
    Serial.println("°\n");
  }
  
  delay(1);  // 1ms para estabilidad
}
