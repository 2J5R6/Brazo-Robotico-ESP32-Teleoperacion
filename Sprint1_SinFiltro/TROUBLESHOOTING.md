# Guía de Troubleshooting y Optimización - Sprint 1

## 🔧 Problemas Comunes y Soluciones

### 1. Problemas de Comunicación ESP-NOW

#### ❌ "Error: Fallo al inicializar ESP-NOW"

**Causas posibles**:
- WiFi no está en modo estación
- Conflicto con otros protocolos WiFi
- Memoria insuficiente

**Soluciones**:
```cpp
// Asegurar que WiFi esté configurado correctamente
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(100);

// Si persiste, reiniciar WiFi
esp_wifi_stop();
esp_wifi_start();
```

#### ❌ Receptor no recibe datos

**Verificación paso a paso**:

1. **Verificar MACs**:
```cpp
// En ambos ESP, imprimir MAC
Serial.print("MAC: ");
Serial.println(WiFi.macAddress());
```

2. **Verificar canal WiFi**:
```cpp
// Ambos deben estar en el mismo canal
Serial.print("Canal: ");
Serial.println(WiFi.channel());
```

3. **Probar recepción básica**:
```cpp
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  Serial.println("¡Datos recibidos!");  // Debug básico
}
```

#### ❌ Envío exitoso pero no llegan datos

**Problema**: Broadcast puede ser bloqueado por algunos routers WiFi

**Solución**: Usar MAC específica en lugar de broadcast
```cpp
// En el receptor, obtener MAC:
Serial.println(WiFi.macAddress());  // Ej: AA:BB:CC:DD:EE:FF

// En el transmisor, cambiar:
uint8_t broadcastAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

---

### 2. Problemas con MPU6050

#### ❌ "No se detectó MPU6050"

**Verificación de conexiones**:
```
┌─────────────────────────────────┐
│ Usar multímetro para verificar: │
├─────────────────────────────────┤
│ ESP32 WROOM (Guante):           │
│ VCC → 3.3V: Debe medir ~3.3V    │
│ GND → GND: Continuidad OK       │
│ SDA → GPIO21: Conectado         │
│ SCL → GPIO22: Conectado         │
│                                 │
│ ESP32-S3 (Brazo - opcional):    │
│ VCC → 3.3V: Debe medir ~3.3V    │
│ GND → GND: Continuidad OK       │
│ SDA → GPIO8: Conectado          │
│ SCL → GPIO10: Conectado         │
│ ⚠️ NO usar GPIO9 para SCL       │
└─────────────────────────────────┘
```

**Test I2C Scanner**:
```cpp
// Código para escanear dispositivos I2C
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  
  // Para ESP32 WROOM (Guante)
  Wire.begin(21, 22);  // SDA, SCL
  
  // Para ESP32-S3 (Brazo - opcional)
  // Wire.begin(8, 10);  // SDA, SCL - NO usar GPIO9
  
  Serial.println("Escaneando I2C...");
  for(byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Dispositivo encontrado en 0x");
      Serial.println(i, HEX);
    }
  }
}

void loop() {}

// MPU6050 debe aparecer en 0x68 o 0x69
```

#### ❌ Lecturas inestables o incorrectas

**Calibración del MPU6050**:
```cpp
// Agregar en setup() después de mpu.begin()
void calibrateMPU() {
  Serial.println("Calibrando MPU6050...");
  Serial.println("Mantener QUIETO durante 5 segundos");
  
  float ax_offset = 0, ay_offset = 0, az_offset = 0;
  int samples = 100;
  
  for(int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax_offset += a.acceleration.x;
    ay_offset += a.acceleration.y;
    az_offset += (a.acceleration.z - 9.81);  // Restar gravedad
    delay(10);
  }
  
  ax_offset /= samples;
  ay_offset /= samples;
  az_offset /= samples;
  
  Serial.println("Offsets calculados:");
  Serial.print("X: "); Serial.println(ax_offset);
  Serial.print("Y: "); Serial.println(ay_offset);
  Serial.print("Z: "); Serial.println(az_offset);
  
  // Guardar estos valores y restarlos en cada lectura
}
```

---

### 3. Problemas con Servomotores

#### ❌ Servos no se mueven

**Checklist**:
- [ ] Alimentación 5V externa conectada (MIN 2A)
- [ ] GND común entre ESP32 y fuente de servos
- [ ] Señal conectada a GPIO correcto (12, 13)
- [ ] Servo no está mecánicamente bloqueado

**Test individual de servo**:
```cpp
#include <ESP32Servo.h>

Servo testServo;

void setup() {
  testServo.attach(13);  // Probar un servo
  testServo.write(90);   // Posición central
}

void loop() {
  // Barrido de prueba
  for(int pos = 0; pos <= 180; pos += 10) {
    testServo.write(pos);
    delay(500);
  }
}
```

#### ❌ Servos se mueven erráticamente

**Causas**:
1. **Alimentación insuficiente**: Usar fuente de al menos 2A
2. **Ruido en la señal**: Agregar capacitor 100µF en alimentación
3. **Cable muy largo**: Máximo 30cm de cable de señal

**Solución - Suavizado del movimiento**:
```cpp
// En lugar de write() directo, usar movimiento gradual
void smoothMove(Servo &servo, int targetPos, int currentPos) {
  int step = (targetPos > currentPos) ? 1 : -1;
  
  for(int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos);
    delay(15);  // 15ms por grado = movimiento suave
  }
}
```

---

### 4. Problemas con DAC

#### ❌ Sin señal en GPIO25

**IMPORTANTE**: Solo el ESP32 WROOM tiene DAC, NO el ESP32-S3

**Verificación**:
```cpp
// Test simple de DAC - Solo en ESP32 WROOM
void setup() {
  dac_output_enable(DAC_CHANNEL_1);  // GPIO25
  Serial.println("DAC en GPIO25 (ESP32 WROOM)");
}

void loop() {
  // Generar rampa
  for(int i = 0; i < 256; i++) {
    dac_output_voltage(DAC_CHANNEL_1, i);
    delay(10);
  }
}

// Debe verse rampa de 0 a 3.3V en osciloscopio
```

#### ⚠️ Error: "DAC no disponible en ESP32-S3"

**Causa**: ESP32-S3 NO tiene DAC

**Solución**: Usar ESP32 WROOM en el guante (transmisor) para tener salida DAC

#### ❌ Señal con mucho ruido

**Mejoras**:
1. Cable corto y blindado al osciloscopio
2. GND común estable
3. Agregar capacitor de desacoplo 100nF cerca del ESP32

---

## ⚙️ Optimizaciones de Rendimiento

### 1. Reducir Latencia

**Problema**: Retraso entre movimiento de mano y respuesta del brazo

**Medición**:
```cpp
// En transmisor
dataToSend.timestamp = micros();  // Usar micros() en lugar de millis()

// En receptor
void OnDataRecv(...) {
  unsigned long latency = micros() - receivedData.timestamp;
  Serial.print("Latencia: ");
  Serial.print(latency / 1000.0);
  Serial.println(" ms");
}
```

**Optimizaciones**:

1. **Aumentar frecuencia de muestreo**:
```cpp
const unsigned long SEND_INTERVAL = 10;  // 100Hz en lugar de 50Hz
```

2. **Prioridad de tarea**:
```cpp
// Usar RTOS de ESP32 para mayor prioridad
void taskESPNOW(void *parameter) {
  while(1) {
    // Envío ESP-NOW
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  xTaskCreatePinnedToCore(
    taskESPNOW,   // Función
    "ESP-NOW",    // Nombre
    10000,        // Stack size
    NULL,         // Parámetros
    2,            // Prioridad (alta)
    NULL,         // Handle
    1             // Core 1
  );
}
```

---

### 2. Filtrado de Software Básico

**Promedio móvil simple** (para Sprint 1):

```cpp
// Buffer circular para promedio
#define FILTER_SIZE 5
float accelX_buffer[FILTER_SIZE];
int buffer_index = 0;

float filteredAccelX(float newValue) {
  accelX_buffer[buffer_index] = newValue;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;
  
  float sum = 0;
  for(int i = 0; i < FILTER_SIZE; i++) {
    sum += accelX_buffer[i];
  }
  
  return sum / FILTER_SIZE;
}
```

**Filtro paso bajo de primer orden**:

```cpp
float alpha = 0.3;  // Factor de suavizado (0-1)
float filtered_accelX = 0;

void loop() {
  float raw_accelX = /* leer sensor */;
  
  // Filtro IIR
  filtered_accelX = alpha * raw_accelX + (1 - alpha) * filtered_accelX;
  
  // Usar filtered_accelX para control
}
```

---

### 3. Manejo de Timeout Mejorado

```cpp
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT_WARNING = 500;    // 500ms - advertencia
const unsigned long TIMEOUT_SAFE = 2000;      // 2s - modo seguro

enum SystemState {
  ACTIVE,
  WARNING,
  SAFE_MODE
};

SystemState currentState = ACTIVE;

void loop() {
  unsigned long elapsed = millis() - lastReceiveTime;
  
  switch(currentState) {
    case ACTIVE:
      if(elapsed > TIMEOUT_WARNING) {
        currentState = WARNING;
        Serial.println("⚠ Advertencia: Conexión inestable");
        digitalWrite(LED_PIN, HIGH);  // LED constante
      }
      break;
      
    case WARNING:
      if(elapsed > TIMEOUT_SAFE) {
        currentState = SAFE_MODE;
        Serial.println("⚠ Modo seguro activado");
        // Mover a posición de reposo
        servo1.write(90);
        servo2.write(90);
      } else if(elapsed < TIMEOUT_WARNING) {
        currentState = ACTIVE;
        Serial.println("✓ Conexión recuperada");
        digitalWrite(LED_PIN, LOW);
      }
      break;
      
    case SAFE_MODE:
      if(elapsed < TIMEOUT_WARNING) {
        currentState = ACTIVE;
        Serial.println("✓ Sistema reactivado");
        digitalWrite(LED_PIN, LOW);
      }
      break;
  }
}
```

---

## 📊 Calibración y Ajuste de Parámetros

### 1. Rango de Aceleraciones

**Ajustar según el espacio de trabajo**:

```cpp
// Capturar máximos y mínimos reales
float accelX_min = 999, accelX_max = -999;

void loop() {
  float accelX = /* leer */;
  
  if(accelX < accelX_min) accelX_min = accelX;
  if(accelX > accelX_max) accelX_max = accelX;
  
  // Después de mover la mano por todo el rango
  Serial.print("Rango X: ");
  Serial.print(accelX_min);
  Serial.print(" a ");
  Serial.println(accelX_max);
}

// Usar estos valores para mapeo preciso
```

### 2. Zona Muerta (Deadband)

**Evitar micro-movimientos no deseados**:

```cpp
float applyDeadband(float value, float threshold = 0.5) {
  if(abs(value) < threshold) {
    return 0;
  }
  return value;
}

// Uso:
float accelX_filtered = applyDeadband(accelX, 0.5);
```

### 3. Limitador de Velocidad

**Evitar movimientos bruscos**:

```cpp
int lastServoPosition = 90;
const int MAX_SPEED = 5;  // Grados por ciclo

void safeServoWrite(Servo &servo, int targetPos) {
  int diff = targetPos - lastServoPosition;
  
  if(abs(diff) > MAX_SPEED) {
    // Limitar cambio
    targetPos = lastServoPosition + (diff > 0 ? MAX_SPEED : -MAX_SPEED);
  }
  
  servo.write(targetPos);
  lastServoPosition = targetPos;
}
```

---

## 🔬 Herramientas de Debug Avanzadas

### 1. Monitor de Rendimiento

```cpp
class PerformanceMonitor {
  private:
    unsigned long lastTime;
    int loopCount;
    
  public:
    PerformanceMonitor() : lastTime(0), loopCount(0) {}
    
    void tick() {
      loopCount++;
      
      if(millis() - lastTime > 1000) {
        Serial.print("FPS: ");
        Serial.println(loopCount);
        
        loopCount = 0;
        lastTime = millis();
      }
    }
};

PerformanceMonitor perfMon;

void loop() {
  perfMon.tick();
  // resto del código
}
```

### 2. Data Logger para Análisis Offline

```cpp
// Guardar datos en SD o SPIFFS para análisis posterior
#include <SD.h>

File dataFile;

void logData(float accelX, float accelY, float accelZ) {
  if(dataFile) {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(accelX);
    dataFile.print(",");
    dataFile.print(accelY);
    dataFile.print(",");
    dataFile.println(accelZ);
  }
}
```

---

## 📝 Checklist Pre-Experimento

Antes de iniciar el experimento, verificar:

### Hardware
- [ ] Todas las conexiones soldadas o bien ajustadas
- [ ] Fuente de 5V/2A para servos funcionando
- [ ] GND común entre todos los componentes
- [ ] IMU firmemente montada en el guante
- [ ] Servos montados sin juego mecánico
- [ ] Cables de señal < 30cm de largo
- [ ] Capacitores de desacoplo instalados
- [ ] **ESP32 WROOM en el guante (tiene DAC)**
- [ ] **ESP32-S3 en el brazo robótico (NO tiene DAC)**

### Software
- [ ] Librerías instaladas y verificadas
- [ ] Código compilado sin errores
- [ ] **Transmisor: "ESP32 Dev Module"**
- [ ] **Receptor: "ESP32S3 Dev Module"**
- [ ] Frecuencia de muestreo configurada (50Hz)
- [ ] Timeout configurado (500ms)
- [ ] Debug serial activado (115200 baud)
- [ ] MACs verificadas en ambos ESP

### Conexiones Específicas
- [ ] **Guante (ESP32 WROOM)**:
  - [ ] MPU6050: SDA=GPIO21, SCL=GPIO22
  - [ ] DAC: GPIO25 al osciloscopio
  - [ ] LED: GPIO2
- [ ] **Brazo (ESP32-S3)**:
  - [ ] Servo1: GPIO12
  - [ ] Servo2: GPIO13
  - [ ] LED: GPIO48
  - [ ] I2C (opcional): SDA=GPIO8, SCL=GPIO10 (NO GPIO9)

### Calibración
- [ ] MPU6050 calibrado en posición horizontal
- [ ] Rangos de aceleración medidos
- [ ] Servos en posición central (90°)
- [ ] Espacio de trabajo definido
- [ ] Puntos de prueba marcados

### Seguridad
- [ ] Brazo no puede golpear obstáculos
- [ ] Botón de emergencia accesible (desconectar fuente)
- [ ] Área de trabajo despejada
- [ ] Modo seguro probado (timeout)

---

**Última actualización**: Sprint 1 - Noviembre 2025
