/*
 * SPRINT 2 MEJORADO - TRANSMISOR (ESP32 WROOM en Guante)
 * 
 * MEJORAS CRÍTICAS vs v1:
 * - Frecuencia 100Hz (era 50Hz) para captar movimientos rápidos
 * - Filtro media móvil 20 muestras (era 10) para menos tremor
 * - Filtro complementario ligero 95%/5% para balance suavidad/respuesta
 * - Buffer circular para suavizado adicional sin latencia
 * - Sensibilidad aumentada ±4g (era ±8g) para más precisión
 * - Predicción de movimiento para compensar latencia
 * 
 * Hardware: ESP32 WROOM + MPU6050 en muñeca
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
#define DAC_PIN 25

#define FILTER_WINDOW 20        // 20 muestras @ 100Hz = 200ms
#define BUFFER_SIZE 5           // Buffer circular para extra smoothing
#define TRANSMIT_HZ 100         // 100 Hz para captar movimientos rápidos
const int TRANSMIT_INTERVAL = 1000 / TRANSMIT_HZ;  // 10ms

// ========== OBJETOS ==========
Adafruit_MPU6050 mpu;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ========== ESTRUCTURA ESP-NOW ==========
typedef struct struct_message {
  float accelX;
  float accelY;      // HORIZONTAL (izq/der) → Servo2
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;       // ROTACIÓN muñeca → Servo1
  unsigned long timestamp;
  uint8_t handPosition;  // 0=mano vertical, 1=mano horizontal (basado en accelZ)
} struct_message;

struct_message sensorData;

// ========== CLASE FILTRO MEJORADO ==========
class AdvancedFilter {
private:
  float movingAvg[FILTER_WINDOW];
  float circularBuffer[BUFFER_SIZE];
  int avgIndex;
  int bufferIndex;
  float avgSum;
  float lastFiltered;
  float velocity;  // Para predicción
  
public:
  AdvancedFilter() {
    avgIndex = 0;
    bufferIndex = 0;
    avgSum = 0;
    lastFiltered = 0;
    velocity = 0;
    for(int i = 0; i < FILTER_WINDOW; i++) movingAvg[i] = 0;
    for(int i = 0; i < BUFFER_SIZE; i++) circularBuffer[i] = 0;
  }
  
  float update(float raw) {
    // 1. Media móvil (reduce tremor)
    avgSum -= movingAvg[avgIndex];
    movingAvg[avgIndex] = raw;
    avgSum += raw;
    avgIndex = (avgIndex + 1) % FILTER_WINDOW;
    float smoothed = avgSum / FILTER_WINDOW;
    
    // 2. Filtro complementario ligero (mantiene respuesta)
    float blended = 0.93 * smoothed + 0.07 * raw;
    
    // 3. Buffer circular adicional (extra smooth)
    circularBuffer[bufferIndex] = blended;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    float bufferSum = 0;
    for(int i = 0; i < BUFFER_SIZE; i++) {
      bufferSum += circularBuffer[i];
    }
    float extraSmooth = bufferSum / BUFFER_SIZE;
    
    // 4. Calcular velocidad para predicción
    velocity = extraSmooth - lastFiltered;
    
    // 5. Predicción ligera (compensa latencia ~20ms)
    float predicted = extraSmooth + (velocity * 0.3);
    
    lastFiltered = extraSmooth;
    return predicted;
  }
  
  float getVelocity() { return velocity; }
};

// ========== FILTROS ==========
AdvancedFilter filterX;
AdvancedFilter filterY;
AdvancedFilter filterZ;

unsigned long lastTransmit = 0;
unsigned long lastDebug = 0;

// ========== CALLBACK ESP-NOW ==========
void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(DAC_PIN, OUTPUT);
  
  Serial.println("\n========================================");
  Serial.println("   SPRINT 2 ULTRA-MEJORADO");
  Serial.println("   TRANSMISOR CON FILTRADO AVANZADO");
  Serial.println("========================================\n");
  
  // I2C
  Serial.print("[1/4] I2C... ");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz para estabilidad ESP32 WROOM
  delay(100);
  Serial.println("OK (100 kHz - estable)");
  
  // MPU6050
  Serial.print("[2/4] MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("ERROR! Revisar conexiones");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  Serial.println("OK");
  
  // Configuración MPU - ALTA SENSIBILIDAD
  Serial.println("    Configurando para máxima precisión:");
  delay(50);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);      // ±4g = más sensible
  delay(10);
  Serial.println("    - Accel: ±4g (alta sensibilidad)");
  
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);          // ±250°/s = más preciso
  delay(10);
  Serial.println("    - Gyro: ±250°/s (alta precisión)");
  
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);       // 44Hz para estabilidad
  delay(10);
  Serial.println("    - Bandwidth: 44 Hz (estable)");
  
  // WiFi
  Serial.print("[3/4] WiFi... ");
  WiFi.mode(WIFI_STA);
  delay(500);  // Tiempo crítico para estabilización
  WiFi.disconnect();
  delay(500);
  
  // Verificar que WiFi esté en modo STA
  if (WiFi.getMode() != WIFI_STA) {
    Serial.println("ERROR - Modo WiFi incorrecto!");
    WiFi.mode(WIFI_STA);
    delay(1000);
  }
  Serial.println("OK (STA mode)");
  
  // ESP-NOW
  Serial.print("[4/4] ESP-NOW... ");
  delay(200);
  
  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR en init!");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
  delay(100);
  
  // Configurar peer con más cuidado
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));  // Limpiar estructura
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;  // Especificar interfaz STA explícitamente
  peerInfo.encrypt = false;
  
  // Agregar peer con verificación
  esp_err_t addStatus = esp_now_add_peer(&peerInfo);
  if (addStatus != ESP_OK) {
    Serial.print("ERROR peer (");
    Serial.print(addStatus);
    Serial.println(")!");
    Serial.println("Verificar: WiFi debe estar en modo STA");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }
  delay(100);
  
  // Registrar callback
  esp_now_register_send_cb(OnDataSent);
  delay(100);
  Serial.println("OK");
  
  Serial.println("\n========================================");
  Serial.println("   CONFIGURACIÓN ULTRA-OPTIMIZADA");
  Serial.println("========================================");
  Serial.println("Filtro Multi-Capa:");
  Serial.println("  1. Media Móvil: 20 muestras");
  Serial.println("  2. Complementario: 93%/7%");
  Serial.println("  3. Buffer Circular: 5 muestras");
  Serial.println("  4. Predicción: compensación latencia");
  Serial.println();
  Serial.println("Frecuencia: 100 Hz (10ms)");
  Serial.println("Sensibilidad: MÁXIMA (±4g, ±250°/s)");
  Serial.println();
  Serial.println("¡Sistema listo para movimientos PRECISOS!");
  Serial.println("========================================\n");
  
  delay(500);
  
  // Parpadeo confirmación
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // Transmitir a 100Hz
  if (now - lastTransmit >= TRANSMIT_INTERVAL) {
    
    // Leer MPU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Aplicar filtros AVANZADOS a aceleraciones
    float filteredX = filterX.update(a.acceleration.x);
    float filteredY = filterY.update(a.acceleration.y);
    float filteredZ = filterZ.update(a.acceleration.z);
    
    // Preparar datos
    sensorData.accelX = filteredX;
    sensorData.accelY = filteredY;
    sensorData.accelZ = filteredZ;
    sensorData.gyroX = g.gyro.x;
    sensorData.gyroY = g.gyro.y;
    sensorData.gyroZ = g.gyro.z;
    sensorData.timestamp = now;
    
    // DETECCIÓN MEJORADA CON HISTÉRESIS
    // Gravedad = 9.8 m/s²
    // Si |accelZ| > 8.0 → mano VERTICAL (apuntando arriba/abajo) → Servo1 con GyroZ
    // Si |accelZ| < 4.0 → mano HORIZONTAL (plana) → Servo2 con AccelY
    static uint8_t lastPosition = 1;
    float absZ = abs(filteredZ);
    
    if (absZ > 8.0) {
      sensorData.handPosition = 0;  // VERTICAL → Servo1 rotación
      lastPosition = 0;
    } else if (absZ < 4.0) {
      sensorData.handPosition = 1;  // HORIZONTAL → Servo2 lateral
      lastPosition = 1;
    } else {
      sensorData.handPosition = lastPosition;  // Mantener modo (histéresis)
    }
    
    // Transmitir
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&sensorData, sizeof(sensorData));
    
    lastTransmit = now;
  }
  
  // Debug cada 500ms
  if (now - lastDebug >= 500) {
    lastDebug = now;
    
    Serial.print("📡 TX:");
    Serial.print(int((1000.0 / TRANSMIT_INTERVAL)));
    Serial.print("Hz | AccelY:");
    Serial.print(sensorData.accelY, 2);
    Serial.print(" GyroZ:");
    Serial.print(sensorData.gyroZ, 1);
    Serial.print(" | |Z|:");
    Serial.print(abs(sensorData.accelZ), 1);
    Serial.print(" | ");
    Serial.println(sensorData.handPosition == 0 ? "✋VERTICAL(S1)" : "👉HORIZONTAL(S2)");
  }
  
  delay(2);  // 2ms para estabilidad, ~100Hz efectivo
}
