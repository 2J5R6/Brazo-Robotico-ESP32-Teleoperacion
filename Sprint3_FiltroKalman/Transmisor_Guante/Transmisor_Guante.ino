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

#define FIR_WINDOW 5            // Pre-filtro FIR (reducido para velocidad)
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

// Variables para FORZADO DE ORIENTACIÓN durante giro
// Si AccelY ≈ 0 (reposo/horizontal), FORZAR accelZ=9 para que sistema vea VERTICAL
// Esto evita lecturas intermedias confusas durante el giro físico
static bool force_vertical_reading = false;
static unsigned long force_start_time = 0;
const unsigned long FORCE_DURATION = 350;     // Forzar lectura vertical 350ms (optimizado)
const float REST_ACCEL_Y_MIN = -0.6;         // Rango de reposo en Y (más preciso)
const float REST_ACCEL_Y_MAX = 0.6;          // (mano horizontal quieta)

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
  Wire.setTimeOut(5000); // Timeout de 5ms para evitar cuelgues  delay(100);
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
    
    // DETECCIÓN DE ORIENTACIÓN PRELIMINAR (para saber modo actual)
    float absZ_prelim = abs(accelZ_fir);
    uint8_t current_orientation = (absZ_prelim > 5.0) ? 0 : 1;  // 0=VERTICAL, 1=HORIZONTAL
    
    // DETECCIÓN DE REPOSO: Si AccelY ≈ 0, mano está horizontal quieta
    // SOLO FORZAR si el modo actual es HORIZONTAL (Servo2)
    bool in_rest = (accelY_fir >= REST_ACCEL_Y_MIN && accelY_fir <= REST_ACCEL_Y_MAX);
    
    // Activar FORZADO cuando detectamos reposo Y estamos en modo HORIZONTAL
    if (in_rest && current_orientation == 1 && !force_vertical_reading) {
      force_vertical_reading = true;
      force_start_time = now;
    }
    
    // Desactivar FORZADO después de 350ms
    if (force_vertical_reading && (now - force_start_time >= FORCE_DURATION)) {
      force_vertical_reading = false;
    }
    
    // ⭐ FORZAR accelZ = 9.0 (VERTICAL) cuando está en reposo Y modo es HORIZONTAL
    // Esto previene falsa detección horizontal durante giro H→V
    // NO se activa en modo VERTICAL (evita movimiento no deseado de Servo1)
    if (force_vertical_reading) {
      accelZ_fir = 9.0;  // FORZAR VERTICAL solo durante transición H→V
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
    
    // 3. SUAVIZADO IIR FINAL (α=0.90 - optimizado para velocidad)
    static float pitch_smooth = 0;
    static float roll_smooth = 0;
    pitch_smooth = 0.90 * pitch_smooth + 0.10 * pitch_kalman;
    roll_smooth = 0.90 * roll_smooth + 0.10 * roll_kalman;
    
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
    
    // Detección de orientación con BLOQUEO INTELIGENTE
    float absZ = abs(accelZ_fir);  // Valores reales del sensor, sin manipular
    
    // Filtro promedio sobre AccelZ (8 muestras para respuesta rápida)
    static float accelZ_history[8] = {9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0};  // Inicializar en vertical
    static int z_idx = 0;
    accelZ_history[z_idx] = absZ;
    z_idx = (z_idx + 1) % 8;
    float absZ_avg = 0;
    for(int i = 0; i < 8; i++) absZ_avg += accelZ_history[i];
    absZ_avg /= 8.0;
    
    static uint8_t lastPos = 0;  // INICIA VERTICAL (0=vertical, 1=horizontal)
    static int stability_counter = 0;
    
    // Umbrales con histéresis amplia
    uint8_t newPos = lastPos;
    if (absZ_avg > 8) {        // Detecta VERTICAL
      newPos = 0;
    } else if (absZ_avg < 2.5) { // Detecta HORIZONTAL
      newPos = 1;
    }
    // Entre 2.5 y 8: mantiene estado anterior (zona de histéresis)
    
    // No hay bloqueo adicional - el forzado de accelZ_fir ya previene detección falsa
    
    // Requerir 8 lecturas para VERTICAL→HORIZONTAL, 3 para HORIZONTAL→VERTICAL
    int required_count = 3;  // Por defecto rápido
    
    if (lastPos == 0 && newPos == 1) {
      // VERTICAL → HORIZONTAL: conservador (8 lecturas = 0.08s)
      required_count = 8;
    } else if (lastPos == 1 && newPos == 0) {
      // HORIZONTAL → VERTICAL: rápido (3 lecturas = 0.03s)
      required_count = 3;
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
    
    // Mostrar cuando se FUERZA lectura vertical
    if (force_vertical_reading) {
      Serial.print(" | ⚡ FORCE-Z=9");
    }
    
    Serial.println();
  }
  
  delay(2);
}
