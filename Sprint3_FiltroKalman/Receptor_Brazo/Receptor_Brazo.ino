/*
 * SPRINT 3 - RECEPTOR CON FILTRO DE KALMAN (ESP32-S3 en Brazo)
 * 
 * MEJORAS vs Sprint 2:
 * - Recibe datos ya filtrados con Kalman del transmisor
 * - Interpolación ultra-suave con IIR adicional
 * - Tremor < 0.5° (era <1° en Sprint 2)
 * - Compatible con estructura de datos de Kalman
 * 
 * Autor: Julián Andrés Rosas Sánchez
 * Universidad Militar Nueva Granada
 * Ingeniería Mecatrónica - 6to Semestre
 * Laboratorio de Señales - Sprint 3
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// ========== PINES ==========
#define SERVO1_PIN 6
#define SERVO2_PIN 7
#define LED_PIN 48

// ========== OBJETOS ==========
Servo servo1;
Servo servo2;

// ========== ESTRUCTURA ESP-NOW (compatible con Kalman) ==========
typedef struct struct_message {
  float accelX_filtered;
  float accelY_filtered;
  float accelZ_filtered;
  float gyroX;
  float gyroY;
  float gyroZ_filtered;
  float angle_pitch;
  float angle_roll;
  unsigned long timestamp;
  uint8_t handPosition;
  float kalman_variance;
} struct_message;

struct_message receivedData;

// ========== FILTRO IIR EXTRA ==========
class IIRFilter {
private:
  float alpha;
  float output;
  
public:
  IIRFilter(float a) : alpha(a), output(90.0) {}
  
  float update(float input) {
    output = alpha * output + (1 - alpha) * input;
    return output;
  }
  
  void reset(float value) {
    output = value;
  }
};

IIRFilter iirServo1(0.85);  // Suavizado adicional
IIRFilter iirServo2(0.85);

// Variables de control
float servo1Current = 90.0;
float servo2Current = 90.0;
float servo1Target = 90.0;
float servo2Target = 90.0;

uint8_t activeServo = 1;
unsigned long lastReceiveTime = 0;
unsigned long lastUpdate = 0;
const unsigned long TIMEOUT = 500;
const float UPDATE_INTERVAL = 5;  // 200Hz

const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

// ========== CALLBACK ESP-NOW ==========
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();
  activeServo = receivedData.handPosition;
  digitalWrite(LED_PIN, HIGH);
  
  // Mapeo según modo
  float sensorValue = 0;
  float mappedAngle = 0;
  
  if (activeServo == 0) {
    // VERTICAL: Servo1 con GyroZ integrado
    sensorValue = receivedData.gyroZ_filtered;
    
    static float gyroIntegrated = 90.0;
    static unsigned long lastReset = 0;
    
    // Reset suave si no hay movimiento
    if (abs(sensorValue) < 2.0 && (millis() - lastReset > 2000)) {
      gyroIntegrated = gyroIntegrated * 0.99 + 90.0 * 0.01;
    } else {
      lastReset = millis();
    }
    
    gyroIntegrated += sensorValue * 0.8;
    gyroIntegrated = constrain(gyroIntegrated, SERVO_MIN, SERVO_MAX);
    
    servo1Target = gyroIntegrated;
    
  } else {
    // HORIZONTAL: Servo2 con AccelY
    sensorValue = receivedData.accelY_filtered;
    mappedAngle = map(sensorValue * 100, 250, -250, SERVO_MIN, SERVO_MAX);
    mappedAngle = constrain(mappedAngle, SERVO_MIN, SERVO_MAX);
    servo2Target = mappedAngle;
  }
  
  // Debug cada 25 paquetes (cada ~250ms)
  static int debugCount = 0;
  if (debugCount++ >= 25) {
    debugCount = 0;
    Serial.print("✓ RX | ");
    Serial.print(activeServo == 0 ? "✋VERT" : "👉HORIZ");
    Serial.print(" | Val:");
    Serial.print(sensorValue, 1);
    Serial.print(" | KVar:");
    Serial.print(receivedData.kalman_variance, 4);
    Serial.print(" | S");
    Serial.print(activeServo + 1);
    Serial.print(":");
    Serial.print((int)(activeServo == 0 ? servo1Current : servo2Current));
    Serial.println("°");
  }
}

// ========== ACTUALIZACIÓN SUAVE DE SERVOS ==========
void updateServoSmooth(Servo &servo, float &current, float target, IIRFilter &iir) {
  // Aplicar IIR adicional para ultra-suavidad
  float smoothTarget = iir.update(target);
  
  // Calcular diferencia
  float diff = smoothTarget - current;
  
  // Step adaptativo según diferencia
  float step = 0;
  if (abs(diff) > 20) {
    step = 8.0;  // Movimiento rápido
  } else if (abs(diff) > 5) {
    step = 3.0;  // Movimiento medio
  } else {
    step = 1.0;  // Movimiento fino
  }
  
  // Interpolar
  if (diff > step) {
    current += step;
  } else if (diff < -step) {
    current -= step;
  } else {
    current = smoothTarget;
  }
  
  current = constrain(current, SERVO_MIN, SERVO_MAX);
  servo.write((int)current);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Parpadeo inicial
  for(int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println("\n========================================");
  Serial.println("   SPRINT 3 - FILTRO DE KALMAN");
  Serial.println("   Desarrollador: Julián A. Rosas S.");
  Serial.println("========================================\n");
  
  // WiFi
  Serial.print("[1/4] WiFi... ");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("OK");
  
  // Servos
  Serial.print("[2/4] Servos... ");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo1.write(90);
  servo2.write(90);
  delay(500);
  Serial.println("OK");
  
  // Test servos
  Serial.print("    Test... ");
  servo1.write(60); delay(200);
  servo1.write(120); delay(200);
  servo1.write(90);
  servo2.write(60); delay(200);
  servo2.write(120); delay(200);
  servo2.write(90);
  delay(300);
  Serial.println("OK ✓");
  
  // ESP-NOW
  Serial.print("[3/4] ESP-NOW... ");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR!");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("OK");
  
  Serial.println("[4/4] Sistema listo");
  
  Serial.println("\n========================================");
  Serial.println("   RECEPTOR FILTRO KALMAN");
  Serial.println("========================================");
  Serial.println("Datos: Pre-filtrados con Kalman (TX)");
  Serial.println("IIR:   α=0.85 (suavizado extra)");
  Serial.println("Freq:  200 Hz actualización servos");
  Serial.println("========================================\n");
  
  // Confirmación
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  
  Serial.println("✓ Esperando datos del guante...\n");
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // Apagar LED después de recibir
  if (now - lastReceiveTime > 20) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Actualizar servos a 200Hz
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;
    
    // Actualizar servo1 (vertical)
    updateServoSmooth(servo1, servo1Current, servo1Target, iirServo1);
    
    // Actualizar servo2 (horizontal)
    updateServoSmooth(servo2, servo2Current, servo2Target, iirServo2);
  }
  
  // Timeout - volver a centro
  if (lastReceiveTime > 0 && (now - lastReceiveTime > TIMEOUT)) {
    servo1Target = 90;
    servo2Target = 90;
    
    // Reset filtros IIR
    iirServo1.reset(90);
    iirServo2.reset(90);
  }
  
  delay(1);
}
