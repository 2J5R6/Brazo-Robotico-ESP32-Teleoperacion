/*
 * SPRINT 3 - TRANSMISOR (ESP32 WROOM en Guante)
 * CON FILTRO DE KALMAN
 * 
 * Mejoras vs Sprint 2:
 * - Filtro de Kalman para filtrado óptimo
 * - Estimación precisa del estado
 * - Reducción máxima de ruido
 * 
 * HARDWARE:
 * - ESP32 WROOM (tiene DAC)
 * - MPU6050 (I2C: SDA=GPIO21, SCL=GPIO22)
 * - DAC Output: GPIO25
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <driver/dac.h>

// ========== CONFIGURACIÓN ==========
#define SDA_PIN 21
#define SCL_PIN 22
#define DAC_PIN 25
#define LED_PIN 2

// ========== OBJETOS ==========
Adafruit_MPU6050 mpu;

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

struct_message dataToSend;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ========== FILTRO DE KALMAN ==========
class KalmanFilter {
  private:
    float Q;  // Varianza del proceso
    float R;  // Varianza de la medición
    float P;  // Covarianza del error de estimación
    float K;  // Ganancia de Kalman
    float X;  // Estado estimado
    
  public:
    KalmanFilter(float process_noise, float measurement_noise, float estimation_error, float initial_value) {
      Q = process_noise;
      R = measurement_noise;
      P = estimation_error;
      X = initial_value;
    }
    
    float update(float measurement) {
      // Predicción
      P = P + Q;
      
      // Actualización
      K = P / (P + R);
      X = X + K * (measurement - X);
      P = (1 - K) * P;
      
      return X;
    }
    
    void reset(float value) {
      X = value;
      P = 1.0;
    }
    
    float getState() {
      return X;
    }
};

// Filtros Kalman para cada eje
// Parámetros: Q (ruido proceso), R (ruido medición), P (error inicial), X0 (valor inicial)
KalmanFilter kalmanX(0.01, 0.1, 1.0, 0.0);
KalmanFilter kalmanY(0.01, 0.1, 1.0, 0.0);
KalmanFilter kalmanZ(0.01, 0.1, 1.0, 9.8);

// ========== VARIABLES ==========
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 20;  // 50Hz
bool mpuReady = false;

// ========== CALLBACK ESP-NOW ==========
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=========================================");
  Serial.println("   SPRINT 3 - TRANSMISOR (Guante)");
  Serial.println("   CON FILTRO DE KALMAN");
  Serial.println("=========================================\n");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // DAC
  dac_output_enable(DAC_CHANNEL_1);
  Serial.println("✓ DAC configurado en GPIO25");
  
  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println("✓ I2C GPIO21/22");
  
  // MPU6050
  Serial.print("✓ Iniciando MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("FALLO");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  Serial.println("OK");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("✓ MPU6050 configurado (±8G, ±500°/s, 21Hz)");
  
  // Calibración inicial del filtro Kalman
  Serial.print("✓ Calibrando Filtro de Kalman... ");
  sensors_event_t accel, gyro, temp;
  for(int i = 0; i < 50; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    kalmanX.update(accel.acceleration.x);
    kalmanY.update(accel.acceleration.y);
    kalmanZ.update(accel.acceleration.z);
    delay(10);
  }
  Serial.println("OK (50 muestras)");
  mpuReady = true;
  
  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  MAC Transmisor (Guante)          ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.print("║  ");
  Serial.print(WiFi.macAddress());
  Serial.println("          ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("✗ ERROR: ESP-NOW init");
    return;
  }
  Serial.println("✓ ESP-NOW inicializado");
  
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("✗ ERROR: Peer add");
    return;
  }
  Serial.println("✓ Peer broadcast agregado");
  
  Serial.println("\n=========================================");
  Serial.println("   SISTEMA LISTO - KALMAN ACTIVO");
  Serial.println("=========================================");
  Serial.println("Frecuencia: 50Hz | Filtro: Kalman (Q=0.01, R=0.1)");
  Serial.println("Transmitiendo datos filtrados...\n");
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

// ========== LOOP ==========
void loop() {
  if (millis() - lastSendTime >= SEND_INTERVAL && mpuReady) {
    lastSendTime = millis();
    
    // Leer sensores MPU6050
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // APLICAR FILTRO DE KALMAN
    float filteredX = kalmanX.update(accel.acceleration.x);
    float filteredY = kalmanY.update(accel.acceleration.y);
    float filteredZ = kalmanZ.update(accel.acceleration.z);
    
    // Llenar estructura con datos FILTRADOS
    dataToSend.accelX = filteredX;
    dataToSend.accelY = filteredY;
    dataToSend.accelZ = filteredZ;
    dataToSend.gyroX = gyro.gyro.x;
    dataToSend.gyroY = gyro.gyro.y;
    dataToSend.gyroZ = gyro.gyro.z;
    dataToSend.timestamp = millis();
    
    // Determinar posición de la mano
    if (dataToSend.accelZ > 8.0) {
      dataToSend.handPosition = 1;  // Arriba
    } else if (dataToSend.accelZ < 2.0) {
      dataToSend.handPosition = 0;  // Abajo
    }
    
    // Enviar por ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
    
    // DAC con datos FILTRADOS por Kalman
    int dacValue = map(dataToSend.accelX * 100, -800, 800, 0, 255);
    dacValue = constrain(dacValue, 0, 255);
    dac_output_voltage(DAC_CHANNEL_1, dacValue);
    
    // Debug (cada 25 envíos = 0.5s)
    static int debugCounter = 0;
    if (debugCounter++ >= 25) {
      debugCounter = 0;
      Serial.print("AccelX(RAW):");
      Serial.print(accel.acceleration.x, 2);
      Serial.print(" → KALMAN:");
      Serial.print(filteredX, 2);
      Serial.print(" | Y:");
      Serial.print(filteredY, 1);
      Serial.print(" Z:");
      Serial.print(filteredZ, 1);
      Serial.print(" | Mano:");
      Serial.print(dataToSend.handPosition == 1 ? "↑" : "↓");
      Serial.print(" | ");
      Serial.println(result == ESP_OK ? "✓" : "✗");
    }
  }
}
