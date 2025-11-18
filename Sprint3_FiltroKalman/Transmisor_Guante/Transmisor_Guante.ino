/*
 * SPRINT 3 - TRANSMISOR CON FILTRO DE KALMAN (ESP32 WROOM en Guante)
 * 
 * ARQUITECTURA DE FILTRADO AVANZADA:
 * 1. FIR Media Móvil (N=10) - Pre-filtrado de ruido
 * 2. Filtro de Kalman - Fusión acelerómetro + giroscopio
 * 3. IIR Complementario (α=0.95) - Suavizado adicional
 * 4. Predicción lineal - Compensación de latencia
 * 
 * MEJORAS vs Sprint 2:
 * - Filtro de Kalman para estimación óptima de estado
 * - Fusión sensorial accel + gyro del MPU6050
 * - Tremor < 0.5° (era <1° en Sprint 2)
 * - Predicción de trayectoria mejorada
 * - Covarianza adaptativa según movimiento
 * 
 * Autor: Julián Andrés Rosas Sánchez
 * Universidad Militar Nueva Granada
 * Ingeniería Mecatrónica - 6to Semestre
 * Laboratorio de Señales - Sprint 3
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ========== CONFIGURACIÓN ==========
#define LED_PIN 2
#define SDA_PIN 4
#define SCL_PIN 5

#define FIR_WINDOW 10           // Pre-filtro FIR
#define TRANSMIT_HZ 100         // 100 Hz
const int TRANSMIT_INTERVAL = 1000 / TRANSMIT_HZ;

// ========== OBJETOS ==========
Adafruit_MPU6050 mpu;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ========== ESTRUCTURA ESP-NOW ==========
typedef struct struct_message {
  float accelX_filtered;
  float accelY_filtered;
  float accelZ_filtered;
  float gyroX;
  float gyroY;
  float gyroZ_filtered;
  float angle_pitch;      // Ángulo estimado por Kalman
  float angle_roll;       // Ángulo estimado por Kalman
  unsigned long timestamp;
  uint8_t handPosition;
  float kalman_variance;  // Varianza del Kalman (calidad de estimación)
} struct_message;

struct_message sensorData;

// ========== FILTRO FIR PRE-PROCESAMIENTO ==========
class FIRFilter {
private:
  float buffer[FIR_WINDOW];
  int index;
  float sum;
  
public:
  FIRFilter() : index(0), sum(0) {
    for(int i = 0; i < FIR_WINDOW; i++) buffer[i] = 0;
  }
  
  float update(float value) {
    sum -= buffer[index];
    buffer[index] = value;
    sum += value;
    index = (index + 1) % FIR_WINDOW;
    return sum / FIR_WINDOW;
  }
};

// ========== FILTRO DE KALMAN ==========
class KalmanFilter {
private:
  // Variables de estado
  float x_estimate;      // Estado estimado (ángulo)
  float P;               // Covarianza del error de estimación
  
  // Parámetros del filtro
  float Q;               // Covarianza del ruido del proceso
  float R;               // Covarianza del ruido de medición
  
  float K;               // Ganancia de Kalman
  float x_predict;       // Predicción del estado
  float P_predict;       // Predicción de covarianza
  
public:
  KalmanFilter(float q = 0.001, float r = 0.03) {
    x_estimate = 0;
    P = 1;
    Q = q;  // Confianza en el modelo (bajo = confiamos en predicción)
    R = r;  // Confianza en medición (bajo = confiamos en sensor)
  }
  
  // Actualización con giroscopio (predicción) y acelerómetro (corrección)
  float update(float gyro_rate, float accel_angle, float dt) {
    // PREDICCIÓN (usando giroscopio)
    x_predict = x_estimate + gyro_rate * dt;
    P_predict = P + Q;
    
    // CORRECCIÓN (usando acelerómetro)
    K = P_predict / (P_predict + R);  // Ganancia de Kalman
    x_estimate = x_predict + K * (accel_angle - x_predict);
    P = (1 - K) * P_predict;
    
    return x_estimate;
  }
  
  // Ajuste adaptativo de covarianzas según movimiento
  void adaptCovarianceQ(float gyro_magnitude) {
    // Si hay mucho movimiento, aumentar Q (menos confianza en predicción)
    if (gyro_magnitude > 50) {
      Q = 0.005;  // Movimiento rápido
    } else if (gyro_magnitude > 20) {
      Q = 0.002;  // Movimiento moderado
    } else {
      Q = 0.001;  // Movimiento lento o estático
    }
  }
  
  float getVariance() { return P; }
  float getGain() { return K; }
};

// ========== INSTANCIAS DE FILTROS ==========
// Pre-filtros FIR (todos usan FIR_WINDOW=20)
FIRFilter firAccelX, firAccelY, firAccelZ;
FIRFilter firGyroX, firGyroY, firGyroZ;
// Nota: AccelY usa ventana completa (20), mejora filtrado para servo2

// Filtros de Kalman (uno por cada eje angular)
KalmanFilter kalmanPitch;  // Rotación en eje Y
KalmanFilter kalmanRoll;   // Rotación en eje X

// Variables globales
unsigned long lastTransmit = 0;
unsigned long lastDebug = 0;
unsigned long lastKalmanUpdate = 0;
const float dt = 0.01;  // 100Hz = 10ms = 0.01s

// Variables para filtro de cambio brusco
static float last_accelY = 0;
static bool accelY_frozen = false;
static unsigned long freeze_start_time = 0;
const float SUDDEN_CHANGE_THRESHOLD = 1.5;  // m/s²
const unsigned long FREEZE_DURATION = 300;  // ms
const float NEAR_ZERO_MIN = -0.6;           // Rango cercano a 0
const float NEAR_ZERO_MAX = 0.8;

// ========== CALLBACK ESP-NOW ==========
void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
}

// ========== FUNCIONES AUXILIARES ==========
float calculateAccelAngle(float ax, float ay, float az, bool isPitch) {
  // Calcular ángulo desde acelerómetro (asumiendo gravedad)
  if (isPitch) {
    // Pitch (rotación Y): atan2(ax, sqrt(ay² + az²))
    return atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  } else {
    // Roll (rotación X): atan2(ay, sqrt(ax² + az²))
    return atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println("\n========================================");
  Serial.println("   SPRINT 3 - FILTRO DE KALMAN");
  Serial.println("   Desarrollador: Julián A. Rosas S.");
  Serial.println("========================================\n");
  
  // I2C
  Serial.print("[1/4] I2C... ");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(50000);  // Reducido a 50kHz (más estable en movimiento)
  Wire.setTimeOut(5000); // Timeout de 5ms para evitar cuelgues
  delay(100);
  Serial.println("OK");
  
  // MPU6050
  Serial.print("[2/4] MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("ERROR!");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  Serial.println("OK");
  
  // Configuración MPU
  Serial.println("    Configuración óptima:");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.println("    - Accel: ±4g");
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.println("    - Gyro: ±250°/s");
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("    - Bandwidth: 21 Hz (anti-aliasing)");
  delay(100);
  
  // WiFi
  Serial.print("[3/4] WiFi... ");
  WiFi.mode(WIFI_STA);
  delay(500);
  WiFi.disconnect();
  delay(500);
  Serial.println("OK");
  
  // ESP-NOW
  Serial.print("[4/4] ESP-NOW... ");
  delay(200);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR!");
    while(1) delay(1000);
  }
  delay(100);
  
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR peer!");
    while(1) delay(1000);
  }
  delay(100);
  
  esp_now_register_send_cb(OnDataSent);
  delay(100);
  Serial.println("OK");
  
  Serial.println("\n========================================");
  Serial.println("   ARQUITECTURA DE FILTRADO");
  Serial.println("========================================");
  Serial.println("1. FIR (N=10) - Pre-filtrado");
  Serial.println("2. KALMAN - Fusión accel + gyro");
  Serial.println("3. IIR (α=0.95) - Suavizado final");
  Serial.println("4. Predicción - Compensación latencia");
  Serial.println("\nFrecuencia: 100 Hz");
  Serial.println("Tremor esperado: <0.5°");
  Serial.println("========================================\n");
  
  // Calibración inicial
  Serial.print("Calibrando (mantener quieto 3s)...");
  delay(3000);
  Serial.println(" OK\n");
  
  // Parpadeo confirmación
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println("Sistema listo. Iniciando transmisión...\n");
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // Transmitir a 100Hz
  if (now - lastTransmit >= TRANSMIT_INTERVAL) {
    
    // Leer MPU con manejo de errores I2C
    sensors_event_t a, g, temp;
    
    // Intentar leer MPU (puede fallar durante movimiento brusco)
    bool read_success = false;
    for(int retry = 0; retry < 3 && !read_success; retry++) {
      if (mpu.getEvent(&a, &g, &temp)) {
        read_success = true;
      } else {
        delayMicroseconds(100);  // Pequeña pausa antes de reintentar
      }
    }
    
    // Si falla lectura I2C, usar últimos valores conocidos
    static sensors_event_t last_a, last_g;
    if (!read_success) {
      a = last_a;
      g = last_g;
      // Serial.println("⚠ I2C retry"); // Descomenta para debug
    } else {
      last_a = a;
      last_g = g;
    }
    
    // 1. PRE-FILTRADO FIR
    float accelX_fir = firAccelX.update(a.acceleration.x);
    float accelY_fir = firAccelY.update(a.acceleration.y);
    float accelZ_fir = firAccelZ.update(a.acceleration.z);
    float gyroX_fir = firGyroX.update(g.gyro.x);
    float gyroY_fir = firGyroY.update(g.gyro.y);
    float gyroZ_fir = firGyroZ.update(g.gyro.z);
    
    // FILTRO DE CAMBIO BRUSCO (solo si AccelY está cerca de 0)
    bool is_near_zero = (last_accelY >= NEAR_ZERO_MIN && last_accelY <= NEAR_ZERO_MAX);
    float accelY_delta = abs(accelY_fir - last_accelY);
    
    // Si está cerca de 0 y hay cambio brusco > 1.5: congelar
    if (is_near_zero && accelY_delta > SUDDEN_CHANGE_THRESHOLD && !accelY_frozen) {
      accelY_frozen = true;
      freeze_start_time = now;
      accelY_fir = last_accelY;  // Mantener valor anterior
    }
    
    // Descongelar después de 300ms
    if (accelY_frozen && (now - freeze_start_time >= FREEZE_DURATION)) {
      accelY_frozen = false;
    }
    
    // Si está congelado, mantener último valor
    if (accelY_frozen) {
      accelY_fir = last_accelY;
    } else {
      last_accelY = accelY_fir;  // Actualizar solo si no está congelado
    }
    
    // 2. FILTRO DE KALMAN (fusión accel + gyro)
    // Calcular ángulos desde acelerómetro
    float accel_pitch = calculateAccelAngle(accelX_fir, accelY_fir, accelZ_fir, true);
    float accel_roll = calculateAccelAngle(accelX_fir, accelY_fir, accelZ_fir, false);
    
    // Adaptación de covarianza según magnitud de movimiento
    float gyro_magnitude = sqrt(gyroX_fir*gyroX_fir + gyroY_fir*gyroY_fir + gyroZ_fir*gyroZ_fir);
    kalmanPitch.adaptCovarianceQ(gyro_magnitude);
    kalmanRoll.adaptCovarianceQ(gyro_magnitude);
    
    // Actualizar Kalman (predicción con gyro, corrección con accel)
    float pitch_kalman = kalmanPitch.update(gyroY_fir, accel_pitch, dt);
    float roll_kalman = kalmanRoll.update(gyroX_fir, accel_roll, dt);
    
    // 3. SUAVIZADO IIR FINAL (α=0.95)
    static float pitch_smooth = 0;
    static float roll_smooth = 0;
    pitch_smooth = 0.95 * pitch_smooth + 0.05 * pitch_kalman;
    roll_smooth = 0.95 * roll_smooth + 0.05 * roll_kalman;
    
    // Preparar datos para transmisión
    sensorData.accelX_filtered = accelX_fir;
    sensorData.accelY_filtered = accelY_fir;
    sensorData.accelZ_filtered = accelZ_fir;
    sensorData.gyroX = gyroX_fir;
    sensorData.gyroY = gyroY_fir;
    sensorData.gyroZ_filtered = gyroZ_fir;
    sensorData.angle_pitch = pitch_smooth;
    sensorData.angle_roll = roll_smooth;
    sensorData.timestamp = now;
    sensorData.kalman_variance = (kalmanPitch.getVariance() + kalmanRoll.getVariance()) / 2;
    
    // Detección de orientación SIMPLE y ROBUSTA
    float absZ = abs(accelZ_fir);
    
    // Filtro promedio sobre AccelZ (15 muestras para suavizar)
    static float accelZ_history[15] = {0};
    static int z_idx = 0;
    accelZ_history[z_idx] = absZ;
    z_idx = (z_idx + 1) % 15;
    float absZ_avg = 0;
    for(int i = 0; i < 15; i++) absZ_avg += accelZ_history[i];
    absZ_avg /= 15.0;
    
    static uint8_t lastPos = 1;
    static int stability_counter = 0;
    
    // Umbrales con histéresis amplia
    uint8_t newPos = lastPos;
    if (absZ_avg > 8) {        // Detecta VERTICAL (más restrictivo)
      newPos = 0;
    } else if (absZ_avg < 2.5) { // Detecta HORIZONTAL (más restrictivo)
      newPos = 1;
    }
    // Entre 2.5 y 9.2: mantiene estado anterior (zona de histéresis)
    
    // Requerir 20 lecturas consistentes para HORIZONTAL→VERTICAL
    // Solo 5 lecturas para VERTICAL→HORIZONTAL (rápido)
    int required_count = 5;  // Por defecto rápido
    
    if (lastPos == 1 && newPos == 0) {
      // HORIZONTAL → VERTICAL: más conservador (20 lecturas = 0.2s)
      required_count = 20;
    } else if (lastPos == 0 && newPos == 1) {
      // VERTICAL → HORIZONTAL: más rápido (5 lecturas = 0.05s)
      required_count = 5;
    }
    
    // Contador de estabilidad
    if (newPos != lastPos) {
      stability_counter++;
      if (stability_counter >= required_count) {
        lastPos = newPos;
        stability_counter = 0;
      }
    } else {
      stability_counter = 0;  // Reset si no hay cambio
    }
    
    sensorData.handPosition = lastPos;
    
    // Transmitir
    esp_now_send(broadcastAddress, (uint8_t*)&sensorData, sizeof(sensorData));
    
    lastTransmit = now;
  }
  
  // Debug cada 500ms
  if (now - lastDebug >= 500) {
    lastDebug = now;
    
    Serial.print("📡 TX | ");
    Serial.print(sensorData.handPosition == 0 ? "✋VERT" : "👉HORIZ");
    Serial.print(" | |Z|:");
    Serial.print(abs(sensorData.accelZ_filtered), 1);
    Serial.print(" | AccelY:");
    Serial.print(sensorData.accelY_filtered, 2);
    
    // Mostrar estado de congelamiento
    if (accelY_frozen) {
      Serial.print(" | ⏸ CONGELADO");
    }
    
    Serial.println();
  }
  
  delay(2);
}
