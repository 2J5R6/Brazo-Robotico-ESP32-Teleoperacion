/*
 * SPRINT 2 - TRANSMISOR (ESP32 WROOM en Guante)
 * CON FILTRO DE MEDIA MÓVIL
 * 
 * Mejoras vs Sprint 1:
 * - Filtro de media móvil para suavizar aceleraciones
 * - Reduce ruido del MPU6050
 * - Mantiene salida DAC para análisis
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

// Filtro de media móvil
#define FILTER_SIZE 10  // Promedio de 10 lecturas

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

// ========== FILTRO DE MEDIA MÓVIL ==========
class MovingAverageFilter {
  private:
    float buffer[FILTER_SIZE];
    int index;
    float sum;
    bool filled;
    
  public:
    MovingAverageFilter() : index(0), sum(0), filled(false) {
      for(int i = 0; i < FILTER_SIZE; i++) {
        buffer[i] = 0;
      }
    }
    
    float update(float newValue) {
      sum -= buffer[index];
      buffer[index] = newValue;
      sum += newValue;
      
      index = (index + 1) % FILTER_SIZE;
      if (index == 0) filled = true;
      
      return sum / (filled ? FILTER_SIZE : index + 1);
    }
    
    void reset() {
      index = 0;
      sum = 0;
      filled = false;
      for(int i = 0; i < FILTER_SIZE; i++) {
        buffer[i] = 0;
      }
    }
};

// Filtros para cada eje
MovingAverageFilter filterX;
MovingAverageFilter filterY;
MovingAverageFilter filterZ;

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
  Serial.println("   SPRINT 2 - TRANSMISOR (Guante)");
  Serial.println("   CON FILTRO DE MEDIA MÓVIL");
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
  Serial.println("   SISTEMA LISTO - FILTRO ACTIVO");
  Serial.println("=========================================");
  Serial.println("Frecuencia: 50Hz | Filtro: Media móvil (10 muestras)");
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
    
    // APLICAR FILTRO DE MEDIA MÓVIL
    float filteredX = filterX.update(accel.acceleration.x);
    float filteredY = filterY.update(accel.acceleration.y);
    float filteredZ = filterZ.update(accel.acceleration.z);
    
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
    
    // DAC con datos FILTRADOS
    int dacValue = map(dataToSend.accelX * 100, -800, 800, 0, 255);
    dacValue = constrain(dacValue, 0, 255);
    dac_output_voltage(DAC_CHANNEL_1, dacValue);
    
    // Debug (cada 25 envíos = 0.5s)
    static int debugCounter = 0;
    if (debugCounter++ >= 25) {
      debugCounter = 0;
      Serial.print("AccelX(RAW):");
      Serial.print(accel.acceleration.x, 2);
      Serial.print(" → FILT:");
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
