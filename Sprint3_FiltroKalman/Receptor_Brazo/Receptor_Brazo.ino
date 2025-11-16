/*
 * SPRINT 3 - RECEPTOR (ESP32-S3 en Brazo Robótico)
 * CON FILTRO DE KALMAN + FEEDBACK MPU
 * 
 * Mejoras vs Sprint 2:
 * - Recibe datos filtrados con Kalman
 * - Usa MPU del brazo para FUSIÓN DE SENSORES
 * - Compara posición deseada vs posición real
 * - Control más preciso y estable
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ========== CONFIGURACIÓN DE PINES ==========
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

// ========== ESTRUCTURA DE DATOS ==========
typedef struct struct_message {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  unsigned long timestamp;
  uint8_t handPosition;
} struct_message;

struct_message receivedData;

// ========== VARIABLES DE CONTROL ==========
int servo1Position = 90;
int servo2Position = 90;
uint8_t activeServo = 1;
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT = 500;

const float ACCEL_MIN = -4.0;
const float ACCEL_MAX = 4.0;
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

// Variables para feedback del MPU local
float brazoAccelX = 0;
float brazoAccelY = 0;
float brazoAccelZ = 0;

// ========== FILTRO DE KALMAN LOCAL ==========
class KalmanFilter {
  private:
    float Q, R, P, K, X;
    
  public:
    KalmanFilter(float process_noise, float measurement_noise, float estimation_error, float initial_value) {
      Q = process_noise;
      R = measurement_noise;
      P = estimation_error;
      X = initial_value;
    }
    
    float update(float measurement) {
      P = P + Q;
      K = P / (P + R);
      X = X + K * (measurement - X);
      P = (1 - K) * P;
      return X;
    }
};

// Kalman para posición real del brazo
KalmanFilter kalmanBrazo(0.005, 0.05, 1.0, 0.0);

// ========== CALLBACK ESP-NOW ==========
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();
  activeServo = receivedData.handPosition;
  digitalWrite(LED_PIN, HIGH);
  
  // Los datos YA VIENEN FILTRADOS con Kalman del transmisor
  // Mapear accel de -10 a +10 m/s² a 0-180°
  int targetAngle = map((int)(receivedData.accelX * 10), -100, 100, SERVO_MIN, SERVO_MAX);
  targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);
  
  // Si tenemos MPU del brazo, usar feedback para corrección
  int correctedAngle = targetAngle;
  if (mpuBrazoReady) {
    // Calcular error entre objetivo y posición real
    float realAccel = kalmanBrazo.update(brazoAccelX);
    int realAngle = map((int)(realAccel * 10), -100, 100, SERVO_MIN, SERVO_MAX);
    
    // Corrección proporcional (pequeña)
    int error = targetAngle - realAngle;
    correctedAngle = targetAngle + (error / 4);  // Corrección suave
    correctedAngle = constrain(correctedAngle, SERVO_MIN, SERVO_MAX);
  }
  
  // Suavizado de movimiento
  int currentPos = (activeServo == 0) ? servo1Position : servo2Position;
  int diff = correctedAngle - currentPos;
  
  // Movimiento más agresivo con Kalman (datos más estables)
  int step = (abs(diff) > 20) ? 10 : 3;
  int newPos = currentPos;
  
  if (diff > 0) {
    newPos = min(currentPos + step, correctedAngle);
  } else if (diff < 0) {
    newPos = max(currentPos - step, correctedAngle);
  }
  
  // Aplicar posición al servo activo
  if (activeServo == 0) {
    servo1Position = newPos;
    servo1.write(servo1Position);
  } else {
    servo2Position = newPos;
    servo2.write(servo2Position);
  }
  
  // Debug cada 20 recepciones
  static int debugCounter = 0;
  if (debugCounter++ >= 20) {
    debugCounter = 0;
    Serial.print("✓ RX KALMAN | Guante:");
    Serial.print(receivedData.accelX, 2);
    Serial.print(" | Target:");
    Serial.print(targetAngle);
    Serial.print("°");
    if (mpuBrazoReady) {
      Serial.print(" | Brazo:");
      Serial.print(brazoAccelX, 2);
      Serial.print(" | Correg:");
      Serial.print(correctedAngle);
      Serial.print("°");
    }
    Serial.print(" | Real:");
    Serial.print(newPos);
    Serial.print("° | S");
    Serial.println(activeServo + 1);
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(3000);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Parpadeo inicial
  for(int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("   SPRINT 3 - RECEPTOR");
  Serial.println("   CON FILTRO DE KALMAN + FEEDBACK");
  Serial.println("========================================");
  Serial.println();
  Serial.println("LED parpadeó 10 veces - Sistema arrancando...");
  Serial.println();
  
  // WiFi
  Serial.print("[1/5] WiFi... ");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("OK");
  
  // Servos
  Serial.print("[2/5] Servos... ");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo1.write(90);
  servo2.write(90);
  Serial.println("OK (GPIO6, GPIO7)");
  
  // Test de servos
  Serial.print("    Testeando servos... ");
  servo1.write(45);
  delay(300);
  servo1.write(135);
  delay(300);
  servo1.write(90);
  servo2.write(45);
  delay(300);
  servo2.write(135);
  delay(300);
  servo2.write(90);
  Serial.println("OK (movieron)");
  
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
  
  // MPU6500 (IMPORTANTE para Sprint 3)
  Serial.print("[4/5] MPU6500 Brazo (FEEDBACK)... ");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);
  if (mpuBrazo.begin()) {
    mpuBrazoReady = true;
    mpuBrazo.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpuBrazo.setGyroRange(MPU6050_RANGE_500_DEG);
    mpuBrazo.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("OK ✓ (fusión activa)");
    
    // Calibración inicial
    Serial.print("    Calibrando Kalman local... ");
    for(int i = 0; i < 30; i++) {
      sensors_event_t accel, gyro, temp;
      mpuBrazo.getEvent(&accel, &gyro, &temp);
      kalmanBrazo.update(accel.acceleration.x);
      delay(10);
    }
    Serial.println("OK");
  } else {
    mpuBrazoReady = false;
    Serial.println("NO DETECTADO (funciona sin feedback)");
  }
  
  // Listo
  Serial.println("[5/5] Inicialización completa");
  Serial.println();
  Serial.println("========================================");
  Serial.println("   SISTEMA LISTO - KALMAN + FEEDBACK");
  Serial.println("========================================");
  Serial.println("Filtro Kalman: GUANTE + BRAZO");
  Serial.println("MPU local: " + String(mpuBrazoReady ? "ACTIVO (fusión)" : "DESACTIVADO"));
  Serial.println("Precisión: MÁXIMA");
  Serial.println("Esperando datos del Transmisor...");
  Serial.println();
  
  // Parpadeo de confirmación
  for(int i=0; i<3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
}

// ========== LOOP ==========
void loop() {
  static unsigned long lastPrint = 0;
  static unsigned long lastFeedback = 0;
  static int counter = 0;
  
  // Leer MPU del brazo continuamente (si existe)
  if (mpuBrazoReady) {
    static unsigned long lastMpuRead = 0;
    if (millis() - lastMpuRead >= 20) {  // 50Hz
      lastMpuRead = millis();
      sensors_event_t accel, gyro, temp;
      mpuBrazo.getEvent(&accel, &gyro, &temp);
      brazoAccelX = accel.acceleration.x;
      brazoAccelY = accel.acceleration.y;
      brazoAccelZ = accel.acceleration.z;
    }
  }
  
  // Apagar LED
  if (millis() - lastReceiveTime > 50) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Contador si no hay datos
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    
    if (lastReceiveTime == 0 || (millis() - lastReceiveTime >= TIMEOUT)) {
      Serial.print("[");
      Serial.print(counter++);
      Serial.println("] Esperando datos...");
    }
  }
  
  // Feedback detallado cada 3 segundos
  if (mpuBrazoReady && (millis() - lastFeedback >= 3000)) {
    lastFeedback = millis();
    
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║  REPORTE FUSIÓN DE SENSORES       ║");
    Serial.println("╠════════════════════════════════════╣");
    Serial.print("║ Guante AccelX: ");
    Serial.print(receivedData.accelX, 2);
    Serial.println(" m/s²          ║");
    Serial.print("║ Brazo  AccelX: ");
    Serial.print(brazoAccelX, 2);
    Serial.println(" m/s²          ║");
    Serial.print("║ Servo1: ");
    Serial.print(servo1Position);
    Serial.print("° | Servo2: ");
    Serial.print(servo2Position);
    Serial.println("°        ║");
    Serial.print("║ Activo: Servo");
    Serial.print(activeServo + 1);
    Serial.println("                    ║");
    Serial.println("╚════════════════════════════════════╝\n");
  }
  
  // Timeout - volver a centro
  if (lastReceiveTime > 0 && (millis() - lastReceiveTime > TIMEOUT)) {
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
  
  delay(10);
}
