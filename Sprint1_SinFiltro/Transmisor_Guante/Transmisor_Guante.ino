/*
 * SPRINT 1 - TRANSMISOR (ESP32 WROOM en Guante)
 * Sistema de Tele-operación Brazo Robótico 2DOF
 * 
 * Este código:
 * 1. Lee datos de la IMU MPU6050 (acelerómetro y giroscopio)
 * 2. Envía datos por ESP-NOW en modo broadcast
 * 3. Genera señal analógica por DAC para análisis de ruido
 * 
 * HARDWARE:
 * - ESP32 WROOM (tiene DAC para salida analógica)
 * - MPU6050 (I2C: SDA=GPIO21, SCL=GPIO22)
 * - DAC Output: GPIO25 (DAC1 en ESP32 WROOM)
 * 
 * NOTA IMPORTANTE:
 * ✅ ESP32 WROOM tiene DAC (GPIO25 y GPIO26)
 * ✅ ESP32-S3 NO tiene DAC (por eso NO va en el guante)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <driver/dac.h>

// ========== CONFIGURACIÓN DE PINES ==========
#define SDA_PIN 21    // I2C SDA - GPIO estándar para ESP32 WROOM
#define SCL_PIN 22    // I2C SCL - GPIO estándar para ESP32 WROOM
#define DAC_PIN 25    // DAC1 para salida analógica (GPIO25 en ESP32 WROOM)
#define LED_PIN 2     // LED integrado ESP32 WROOM

// ========== VARIABLES GLOBALES ==========
Adafruit_MPU6050 mpu;

// Estructura de datos para enviar por ESP-NOW
typedef struct struct_message {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  unsigned long timestamp;
  uint8_t handPosition;  // 0=abajo, 1=arriba (para seleccionar servo)
} struct_message;

struct_message dataToSend;

// Dirección broadcast (todos los dispositivos)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Variables de control
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 20;  // 50Hz (20ms) - Frecuencia de muestreo
bool mpuReady = false;

// ========== CALLBACK ESP-NOW ==========
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Opcional: LED parpadea si envío exitoso
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// ========== SETUP ==========
void setup() {
  // Inicializar Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SPRINT 1 - TRANSMISOR (Guante) ===");
  Serial.println("ESP32 WROOM con MPU6050 + ESP-NOW + DAC");
  
  // Configurar LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Configurar DAC para salida analógica
  dac_output_enable(DAC_CHANNEL_1);  // GPIO25
  Serial.println("✓ DAC configurado en GPIO25 (DAC1)");
  
  // Inicializar I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("✓ I2C inicializado (SDA=GPIO21, SCL=GPIO22)");
  
  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println("✗ ERROR: No se detectó MPU6050");
    Serial.println("  Verifica conexiones I2C");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  Serial.println("✓ MPU6050 detectado");
  
  // Configurar rangos del MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.print("  Rango acelerómetro: ±");
  Serial.print(mpu.getAccelerometerRange());
  Serial.println("G");
  Serial.print("  Rango giroscopio: ±");
  Serial.print(mpu.getGyroRange());
  Serial.println("°/s");
  
  mpuReady = true;
  
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
  
  // Registrar callback de envío
  esp_now_register_send_cb(OnDataSent);
  
  // Registrar peer (broadcast)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("✗ ERROR: Fallo al agregar peer");
    return;
  }
  
  Serial.println("✓ Peer broadcast agregado");
  Serial.println("\n=== Sistema listo ===");
  Serial.println("Moviendo la mano para enviar datos...\n");
  
  digitalWrite(LED_PIN, HIGH);
}

// ========== LOOP ==========
void loop() {
  // Verificar si es tiempo de enviar
  if (millis() - lastSendTime >= SEND_INTERVAL && mpuReady) {
    lastSendTime = millis();
    
    // Leer sensores MPU6050
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Llenar estructura de datos
    dataToSend.accelX = accel.acceleration.x;
    dataToSend.accelY = accel.acceleration.y;
    dataToSend.accelZ = accel.acceleration.z;
    dataToSend.gyroX = gyro.gyro.x;
    dataToSend.gyroY = gyro.gyro.y;
    dataToSend.gyroZ = gyro.gyro.z;
    dataToSend.timestamp = millis();
    
    // Determinar posición de la mano (arriba/abajo) según aceleración Z
    // Si Z > 8 m/s² → mano arriba (normal, apuntando al cielo)
    // Si Z < 2 m/s² → mano abajo
    if (dataToSend.accelZ > 8.0) {
      dataToSend.handPosition = 1;  // Arriba - Controla servo2 (extremo)
    } else if (dataToSend.accelZ < 2.0) {
      dataToSend.handPosition = 0;  // Abajo - Controla servo1 (base)
    }
    // Si está entre 2 y 8, mantiene el último estado
    
    // Enviar datos por ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
    
    // Generar señal DAC para análisis (usar aceleración X escalada)
    // Escalar de -8G..+8G a 0..255 para DAC
    int dacValue = map(dataToSend.accelX * 100, -800, 800, 0, 255);
    dacValue = constrain(dacValue, 0, 255);
    dac_output_voltage(DAC_CHANNEL_1, dacValue);
    
    // Debug por serial (cada 10 envíos para no saturar)
    static int debugCounter = 0;
    if (debugCounter++ >= 10) {
      debugCounter = 0;
      Serial.print("AccelX: "); Serial.print(dataToSend.accelX, 2);
      Serial.print(" | AccelY: "); Serial.print(dataToSend.accelY, 2);
      Serial.print(" | AccelZ: "); Serial.print(dataToSend.accelZ, 2);
      Serial.print(" | Mano: ");
      Serial.print(dataToSend.handPosition == 1 ? "ARRIBA" : "ABAJO");
      Serial.print(" | DAC: "); Serial.print(dacValue);
      Serial.print(" | Envío: ");
      Serial.println(result == ESP_OK ? "OK" : "FALLO");
    }
  }
  
  // Pequeño delay para no saturar el CPU
  delay(1);
}
