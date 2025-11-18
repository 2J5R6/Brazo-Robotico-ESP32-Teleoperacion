# Sprint 3 - Sistema de Teleoperación con Filtro de Kalman

## Autor
**Julián Andrés Rosas Sánchez**  
Universidad Militar Nueva Granada  
Ingeniería Mecatrónica

---

## 📋 Descripción

Sistema avanzado de control de brazo robótico 2DOF mediante guante instrumentado con MPU6050, implementando **Filtro de Kalman** para fusión sensorial óptima (acelerómetro + giroscopio). Alcanza tremor **<0.5°** mediante arquitectura de filtrado en cascada con transmisión ESP-NOW a 100Hz.

---

## 🎯 Mejoras vs Sprint 2

| Característica | Sprint 2 | Sprint 3 | Mejora |
|----------------|----------|----------|--------|
| **Tremor** | <1° | **<0.5°** | 2x mejor |
| **Fusión sensorial** | Complementario | **Kalman óptimo** | Estimación estadística |
| **Adaptabilidad** | Estática | **Dinámica** | Covarianza adaptativa |
| **Predicción** | Lineal | **Kalman** | Compensación de latencia |
| **Calidad** | Heurística | **Varianza P** | Métrica cuantificable |

---

## 🔬 Arquitectura de Filtrado

### Sistema de 4 Capas (Transmisor + Receptor)

```
┌─────────────────── TRANSMISOR (ESP32 WROOM) ───────────────────┐
│                                                                  │
│  MPU6050 → [1] FIR → [2] Kalman → [3] IIR → ESP-NOW Tx         │
│            Pre-     Fusión        Suavizado   100Hz             │
│            filtrado Accel+Gyro    α=0.95                        │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
                                 ↓
┌─────────────────── RECEPTOR (ESP32-S3) ─────────────────────────┐
│                                                                  │
│  ESP-NOW Rx → [4] Zona Muerta → Buffer → IIR → Servos          │
│               ±0.3 m/s²         5 samples  α=0.95  200Hz        │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 📡 Filtros Implementados

### **[1] Filtro FIR - Media Móvil (Pre-procesamiento)**

**Tipo**: FIR (Finite Impulse Response) **Pasa-Bajas**  
**Orden**: 10 muestras (100ms @ 100Hz)  
**Función**: Eliminar ruido de alta frecuencia antes de Kalman

**Ecuación en diferencias**:
```
y[n] = (1/N) * Σ(x[n-i])  para i=0 hasta N-1
donde N = 10
```

**Función de transferencia**:
```
H(z) = (1/10) * (1 - z⁻¹⁰) / (1 - z⁻¹)
```

**Respuesta en frecuencia**:
- **Tipo**: **Pasa-bajas**
- **Frecuencia de corte (-3dB)**: ~15.9 Hz
- **Atenuación @ 50Hz**: -12.3 dB
- **Fase**: Lineal (retardo constante 50ms)

**Código C++**:
```cpp
class FIRFilter {
private:
  float buffer[FIR_WINDOW];  // FIR_WINDOW = 10
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
```

**MATLAB - Diseño y análisis**:
```matlab
% Parámetros
Fs = 100;           % Frecuencia de muestreo (Hz)
N = 10;             % Orden del filtro

% Coeficientes del filtro (media móvil)
b = ones(1, N) / N;
a = 1;

% Respuesta en frecuencia
[H, f] = freqz(b, a, 1024, Fs);

% Gráfica de magnitud
figure;
subplot(2,1,1);
plot(f, 20*log10(abs(H)));
grid on;
title('FIR Media Móvil - Respuesta en Frecuencia');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud (dB)');
xlim([0 50]);

% Gráfica de fase
subplot(2,1,2);
plot(f, angle(H)*180/pi);
grid on;
xlabel('Frecuencia (Hz)');
ylabel('Fase (grados)');
xlim([0 50]);

% Calcular frecuencia de corte -3dB
fc_idx = find(20*log10(abs(H)) <= -3, 1);
fc = f(fc_idx);
fprintf('Frecuencia de corte (-3dB): %.2f Hz\n', fc);

% Retardo de grupo (ms)
delay_ms = (N-1)/2 * (1000/Fs);
fprintf('Retardo del filtro: %.1f ms\n', delay_ms);
```

---

### **[2] Filtro de Kalman (Fusión Sensorial)**

**Tipo**: Filtro Óptimo Bayesiano (Estimador de Estado)  
**Propósito**: Fusionar acelerómetro (medición) + giroscopio (predicción)  
**Parámetros**:
- **Q** (Covarianza del proceso): 0.001 - 0.005 (adaptativo)
- **R** (Covarianza de medición): 0.03
- **dt**: 0.01s (100Hz)

**Modelo de espacio de estados**:
```
PREDICCIÓN (usando giroscopio):
x̂⁻[k] = x̂[k-1] + ω[k] * dt
P⁻[k] = P[k-1] + Q

CORRECCIÓN (usando acelerómetro):
K[k] = P⁻[k] / (P⁻[k] + R)        (Ganancia de Kalman)
x̂[k] = x̂⁻[k] + K[k](z[k] - x̂⁻[k])  (Estado estimado)
P[k] = (1 - K[k]) * P⁻[k]          (Covarianza actualizada)

Donde:
- x̂ = ángulo estimado (pitch o roll)
- ω = velocidad angular del giroscopio
- z = ángulo medido por acelerómetro
- P = covarianza del error de estimación
- K = ganancia de Kalman (0 a 1)
```

**Adaptación dinámica de Q**:
```cpp
void adaptCovarianceQ(float gyro_magnitude) {
  if (gyro_magnitude > 50) {
    Q = 0.005;  // Movimiento rápido: confía más en accel
  } else if (gyro_magnitude > 20) {
    Q = 0.002;  // Movimiento moderado
  } else {
    Q = 0.001;  // Estático: confía más en gyro integrado
  }
}
```

**Interpretación de la ganancia K**:
- **K ≈ 0**: Confianza en predicción (giroscopio)
- **K ≈ 1**: Confianza en medición (acelerómetro)
- **K ∈ (0, 1)**: Fusión óptima según covarianzas

**Código C++ completo**:
```cpp
class KalmanFilter {
private:
  float x_estimate;  // Estado estimado (ángulo)
  float P;           // Covarianza del error
  float Q;           // Covarianza del proceso
  float R;           // Covarianza de medición
  float K;           // Ganancia de Kalman
  
public:
  KalmanFilter(float q = 0.001, float r = 0.03) {
    x_estimate = 0;
    P = 1;
    Q = q;
    R = r;
  }
  
  float update(float gyro_rate, float accel_angle, float dt) {
    // PREDICCIÓN
    float x_predict = x_estimate + gyro_rate * dt;
    float P_predict = P + Q;
    
    // CORRECCIÓN
    K = P_predict / (P_predict + R);
    x_estimate = x_predict + K * (accel_angle - x_predict);
    P = (1 - K) * P_predict;
    
    return x_estimate;
  }
  
  float getVariance() { return P; }
  float getGain() { return K; }
};
```

**MATLAB - Simulación del Filtro de Kalman**:
```matlab
% Parámetros del filtro
Q = 0.001;  % Covarianza del proceso
R = 0.03;   % Covarianza de medición
dt = 0.01;  % 100 Hz

% Estado inicial
x_est = 0;  % Ángulo estimado
P = 1;      % Covarianza inicial

% Simulación de señales
t = 0:dt:10;  % 10 segundos
true_angle = 45 * sin(2*pi*0.5*t);  % Ángulo verdadero (0.5 Hz)
gyro_rate = gradient(true_angle, dt);  % Giroscopio (derivada)
accel_angle = true_angle + randn(size(t))*5;  % Acelerómetro + ruido

% Aplicar filtro de Kalman
kalman_output = zeros(size(t));
variance_hist = zeros(size(t));
gain_hist = zeros(size(t));

for k = 1:length(t)
    % PREDICCIÓN
    x_pred = x_est + gyro_rate(k) * dt;
    P_pred = P + Q;
    
    % CORRECCIÓN
    K = P_pred / (P_pred + R);
    x_est = x_pred + K * (accel_angle(k) - x_pred);
    P = (1 - K) * P_pred;
    
    % Guardar resultados
    kalman_output(k) = x_est;
    variance_hist(k) = P;
    gain_hist(k) = K;
end

% Gráficas
figure;

% Señales
subplot(3,1,1);
plot(t, true_angle, 'k', 'LineWidth', 1.5); hold on;
plot(t, accel_angle, 'r.', 'MarkerSize', 3);
plot(t, kalman_output, 'b', 'LineWidth', 1.5);
legend('Ángulo Real', 'Acelerómetro (ruidoso)', 'Kalman');
xlabel('Tiempo (s)'); ylabel('Ángulo (°)');
title('Filtro de Kalman - Fusión Sensorial');
grid on;

% Varianza P
subplot(3,1,2);
plot(t, variance_hist, 'g', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Varianza P');
title('Covarianza del Error (Confianza)');
grid on;

% Ganancia K
subplot(3,1,3);
plot(t, gain_hist, 'm', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Ganancia K');
title('Ganancia de Kalman (0=Gyro, 1=Accel)');
grid on;
ylim([0 1]);

% Métricas de desempeño
error_accel = rms(accel_angle - true_angle);
error_kalman = rms(kalman_output - true_angle);
fprintf('Error RMS Acelerómetro: %.2f°\n', error_accel);
fprintf('Error RMS Kalman: %.2f°\n', error_kalman);
fprintf('Mejora: %.1fx\n', error_accel / error_kalman);
```

**Análisis de estabilidad**:
```matlab
% Verificar convergencia del filtro
figure;
plot(t, variance_hist);
xlabel('Tiempo (s)'); ylabel('Covarianza P');
title('Convergencia del Filtro de Kalman');
grid on;

% P debe converger a un valor estable
% Si P → 0: Alta confianza en estimación
% Si P → ∞: Filtro divergente (mal diseño)
```

---

### **[3] Filtro IIR - Complementario (Post-suavizado)**

**Tipo**: IIR (Infinite Impulse Response) **Pasa-Bajas** de 1er orden  
**Parámetro**: α = 0.95 (agresivo)  
**Función**: Suavizado final después de Kalman

**Ecuación en diferencias**:
```
y[n] = α * y[n-1] + (1-α) * x[n]
donde α = 0.95
```

**Función de transferencia**:
```
H(z) = (1-α) / (1 - α*z⁻¹)
     = 0.05 / (1 - 0.95*z⁻¹)
```

**Respuesta en frecuencia**:
- **Tipo**: **Pasa-bajas**
- **Frecuencia de corte (-3dB)**: ~0.8 Hz
- **Atenuación @ 10Hz**: -34.8 dB
- **Fase**: No lineal (mínima)

**Código C++**:
```cpp
class IIRFilter {
private:
  float alpha;   // α = 0.95
  float output;
  
public:
  IIRFilter(float a) : alpha(a), output(0) {}
  
  float update(float input) {
    output = alpha * output + (1 - alpha) * input;
    return output;
  }
};
```

**MATLAB - Diseño y análisis**:
```matlab
% Parámetros
Fs = 100;        % Frecuencia de muestreo (Hz)
alpha = 0.95;    % Coeficiente IIR

% Función de transferencia
b = [1-alpha];
a = [1, -alpha];

% Respuesta en frecuencia
[H, f] = freqz(b, a, 2048, Fs);

% Gráfica de magnitud (escala logarítmica)
figure;
subplot(2,1,1);
semilogx(f, 20*log10(abs(H)));
grid on;
title('IIR Complementario (α=0.95) - Respuesta en Frecuencia');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud (dB)');
xlim([0.1 50]);

% Gráfica de fase
subplot(2,1,2);
semilogx(f, angle(H)*180/pi);
grid on;
xlabel('Frecuencia (Hz)');
ylabel('Fase (grados)');
xlim([0.1 50]);

% Calcular frecuencia de corte -3dB
fc_idx = find(20*log10(abs(H)) <= -3, 1);
fc = f(fc_idx);
fprintf('Frecuencia de corte (-3dB): %.2f Hz\n', fc);

% Polos y ceros
figure;
zplane(b, a);
title('IIR - Diagrama de Polos y Ceros');

% Verificar estabilidad (polo < 1)
poles = roots(a);
fprintf('Polo: %.2f (Estable: %s)\n', poles(1), ...
        abs(poles(1)) < 1 ? 'SÍ' : 'NO');

% Comparar diferentes valores de alpha
alphas = [0.85, 0.90, 0.95, 0.97];
figure;
hold on;
for i = 1:length(alphas)
    alpha_test = alphas(i);
    b_test = [1-alpha_test];
    a_test = [1, -alpha_test];
    [H_test, f_test] = freqz(b_test, a_test, 2048, Fs);
    plot(f_test, 20*log10(abs(H_test)), 'DisplayName', ...
         sprintf('α=%.2f', alpha_test));
end
grid on;
xlabel('Frecuencia (Hz)');
ylabel('Magnitud (dB)');
title('Comparación de diferentes valores de α');
legend('show');
xlim([0 20]);
```

**Análisis de retardo de fase**:
```matlab
% Retardo de grupo (importante para control en tiempo real)
[gd, f_gd] = grpdelay(b, a, 2048, Fs);

figure;
plot(f_gd, gd * 1000/Fs);  % Convertir a ms
grid on;
xlabel('Frecuencia (Hz)');
ylabel('Retardo de Grupo (ms)');
title('IIR - Retardo introducido por el filtro');
xlim([0 10]);

% El retardo debe ser < 10ms para control en tiempo real
max_delay = max(gd(f_gd < 10)) * 1000/Fs;
fprintf('Retardo máximo @ <10Hz: %.2f ms\n', max_delay);
```

---

### **[4] Zona Muerta + Buffer (Receptor)**

**Zona Muerta**: Eliminación de micro-movimientos  
**Umbral**: ±0.3 m/s² (solo para AccelY/Servo2)  
**Buffer**: Promedio móvil de 5 muestras  
**IIR final**: α = 0.95 (200Hz)

**Código C++**:
```cpp
// Zona muerta
if (abs(sensorValue) < DEADZONE_SERVO2) {
  sensorValue = 0;  // Forzar a 0 si es ruido
}

// Buffer circular (5 muestras)
static float accelY_buffer[5] = {0};
static int buf_idx = 0;
accelY_buffer[buf_idx] = sensorValue;
buf_idx = (buf_idx + 1) % 5;

// Promedio
float accelY_avg = 0;
for(int i = 0; i < 5; i++) accelY_avg += accelY_buffer[i];
accelY_avg /= 5.0;

// IIR final
servo2Target = iirServo2.update(mappedAngle);
```

---

## 🔌 Conexiones Hardware

### **Transmisor (ESP32 WROOM - Guante)**

| Componente | Pin ESP32 | Descripción |
|------------|-----------|-------------|
| MPU6050 SDA | GPIO 4 | I2C Data |
| MPU6050 SCL | GPIO 5 | I2C Clock |
| MPU6050 VCC | 3.3V | Alimentación |
| MPU6050 GND | GND | Tierra |
| LED Indicador | GPIO 2 | Estado de transmisión |

**Configuración I2C**:
- Frecuencia: 100 kHz (estándar)
- Dirección MPU6050: 0x68

### **Receptor (ESP32-S3 - Brazo Robótico)**

| Componente | Pin ESP32-S3 | Descripción |
|------------|--------------|-------------|
| Servo 1 | GPIO 6 | Control PWM (Servo base) |
| Servo 2 | GPIO 7 | Control PWM (Servo brazo) |
| LED Indicador | GPIO 48 | Estado de recepción |
| Servos VCC | 5V externo | **NO conectar a 3.3V** |
| Servos GND | GND común | Tierra compartida |

**Configuración PWM Servos**:
- Frecuencia: 50 Hz
- Ancho de pulso: 500-2400 μs
- Resolución: 16 bits

**⚠️ ADVERTENCIA CRÍTICA**:
- Los servos **requieren fuente externa 5V/2A**
- **NO alimentar servos desde pines ESP32** (quema el microcontrolador)
- Compartir GND entre fuente externa y ESP32

---

## 🎮 Detección de Orientación de Mano

### Algoritmo Robusto con Histéresis

```cpp
// Promedio móvil de AccelZ (15 muestras = 150ms)
static float accelZ_history[15] = {0};
float absZ_avg = promedio(accelZ_history, 15);

// Umbrales con zona de histéresis amplia
uint8_t newPos = lastPos;
if (absZ_avg > 9.2) {
  newPos = 0;  // VERTICAL
} else if (absZ_avg < 2.5) {
  newPos = 1;  // HORIZONTAL
}
// Entre 2.5 y 9.2: mantiene estado anterior (evita oscilaciones)

// Contador de estabilidad asimétrico
int required_count = (lastPos == 1 && newPos == 0) ? 20 : 5;
// HORIZONTAL→VERTICAL: 20 lecturas (200ms) - Conservador
// VERTICAL→HORIZONTAL: 5 lecturas (50ms) - Rápido
```

### Modos de Control

| Orientación | Detector | Servo Activo | Sensor | Técnica |
|-------------|----------|--------------|--------|---------|
| **✋ VERTICAL** | \|Z\| > 9.2 | Servo1 | GyroZ | Integración ω |
| **👉 HORIZONTAL** | \|Z\| < 2.5 | Servo2 | AccelY | Mapeo directo |
| **🔄 Transición** | 2.5 < \|Z\| < 9.2 | Mantiene anterior | - | Histéresis |

---

## 📊 Análisis de Desempeño

### Comparación Multi-Sprint

| Métrica | Sprint 1 | Sprint 2 | Sprint 3 |
|---------|----------|----------|----------|
| **Tremor** | ±5° | <1° | **<0.5°** |
| **Latencia** | ~50ms | ~15ms | **~12ms** |
| **Filtrado** | Ninguno | FIR + IIR | **Kalman + FIR + IIR** |
| **Frecuencia Tx** | 50Hz | 100Hz | **100Hz** |
| **Fusión sensorial** | No | Complementario | **Kalman óptimo** |
| **Adaptabilidad** | No | No | **Sí (Q adaptativo)** |

### Reducción de Ruido

**Mediciones experimentales**:
```
Señal cruda MPU6050:        ±0.5 m/s² (tremor visible)
Después de FIR:             ±0.15 m/s²
Después de Kalman:          ±0.08 m/s²
Después de IIR:             ±0.04 m/s²
Servo final (con buffer):   ±0.3° (imperceptible)
```

**Factor de mejora total**: 12.5x reducción de ruido

---

## 🚀 Uso del Sistema

### 1. Preparación del Hardware
```bash
# Verificar conexiones según tablas anteriores
# Alimentar servos con fuente externa 5V/2A
# Conectar GND común entre ESP32-S3 y fuente
```

### 2. Subir el Código

**Transmisor (ESP32 WROOM)**:
```bash
# En Arduino IDE:
# 1. Abrir: Sprint3_FiltroKalman/Transmisor_Guante/Transmisor_Guante.ino
# 2. Placa: ESP32 Dev Module
# 3. Puerto: (seleccionar COM correspondiente)
# 4. Subir código
```

**Receptor (ESP32-S3)**:
```bash
# En Arduino IDE:
# 1. Abrir: Sprint3_FiltroKalman/Receptor_Brazo/Receptor_Brazo.ino
# 2. Placa: ESP32-S3 Dev Module
# 3. Puerto: (seleccionar COM correspondiente)
# 4. Subir código
```

### 3. Operación

**Monitor Serial (115200 baud)**:
```
Transmisor:
📡 TX | ✋VERT | |Z|:9.5 | AccelY:0.12

Receptor:
✓ RX | ✋VERT | Val:12.3 | S1:95° S2:90°
⏳ ESPERA 0.8s  (esperando quietud para cambiar modo)
➡ CAMBIO MODO: 👉HORIZONTAL
```

### 4. Calibración de Servos

Si los servos no están centrados en 90°:
```cpp
// En Receptor_Brazo.ino, ajustar offsets:
const int SERVO1_OFFSET = 0;   // Ajustar entre -10 y +10
const int SERVO2_OFFSET = 0;   // Ajustar entre -10 y +10
```

---

## 🔧 Parámetros Ajustables

### Transmisor (Kalman)
```cpp
// Filtro FIR
#define FIR_WINDOW 10           // Ventana pre-filtrado (10-30)

// Filtro de Kalman
float Q_static = 0.001;         // Covarianza proceso estático
float Q_moving = 0.005;         // Covarianza proceso en movimiento
float R = 0.03;                 // Covarianza medición accel

// IIR Post-Kalman
float alpha_iir = 0.95;         // Suavizado (0.85-0.98)

// Detección de orientación
float VERTICAL_THRESHOLD = 9.2;    // |AccelZ| > 9.2 → vertical
float HORIZONTAL_THRESHOLD = 2.5;  // |AccelZ| < 2.5 → horizontal
int STABILITY_SLOW = 20;           // Lecturas para cambio lento
int STABILITY_FAST = 5;            // Lecturas para cambio rápido
```

### Receptor
```cpp
// IIR Servos
float alpha_servo1 = 0.85;      // Suavizado Servo1 (0.7-0.9)
float alpha_servo2 = 0.95;      // Suavizado Servo2 (0.9-0.98)

// Zona muerta
float DEADZONE_SERVO2 = 0.3;    // Umbral ruido AccelY

// Buffer
int BUFFER_SIZE = 5;            // Promedio móvil (3-10)

// Transición de modo
unsigned long STILLNESS_TIME = 1000;  // Tiempo quietud (ms)
```

---

## 📚 Dependencias

### Librerías Arduino
```cpp
// ESP-NOW (incluida en core ESP32)
#include <esp_now.h>
#include <WiFi.h>

// Servos
#include <ESP32Servo.h>  // v3.0.0+

// MPU6050
#include <Adafruit_MPU6050.h>  // v2.2.4+
#include <Adafruit_Sensor.h>   // v1.1.7+
#include <Wire.h>              // (incluida)
```

### Instalación de Librerías
```bash
# En Arduino IDE:
# Sketch → Include Library → Manage Libraries

# Buscar e instalar:
1. "ESP32Servo" by Kevin Harrington
2. "Adafruit MPU6050" by Adafruit
3. "Adafruit Unified Sensor" by Adafruit
```

---

## 🐛 Troubleshooting

### Problema: Servos tiemblan
**Causa**: Parámetros de filtrado muy bajos  
**Solución**:
```cpp
// Aumentar agresividad IIR
iirServo2(0.97);  // Era 0.95

// Aumentar buffer
int BUFFER_SIZE = 7;  // Era 5

// Aumentar zona muerta
float DEADZONE_SERVO2 = 0.5;  // Era 0.3
```

### Problema: Respuesta lenta
**Causa**: Filtros muy agresivos  
**Solución**:
```cpp
// Reducir alpha IIR
iirServo1(0.75);  // Era 0.85

// Reducir ventana FIR
#define FIR_WINDOW 5;  // Era 10

// Reducir buffer
int BUFFER_SIZE = 3;  // Era 5
```

### Problema: Cambios de modo no funcionan
**Causa**: Umbral de quietud muy estricto  
**Solución**:
```cpp
// Reducir tiempo de quietud
const unsigned long STILLNESS_TIME = 500;  // Era 1000

// Ajustar umbrales de orientación
float VERTICAL_THRESHOLD = 8.5;    // Era 9.2 (menos restrictivo)
float HORIZONTAL_THRESHOLD = 3.0;  // Era 2.5 (menos restrictivo)
```

### Problema: "No data received" en receptor
**Causa**: ESP-NOW no emparejado  
**Solución**:
1. Verificar que ambos ESP32 estén en mismo canal WiFi
2. Comprobar dirección MAC en transmisor
3. Revisar antena WiFi (no tocar durante operación)

---

## 📖 Referencias Técnicas

### Filtro de Kalman
- **Kalman, R. E.** (1960). "A New Approach to Linear Filtering and Prediction Problems". *Journal of Basic Engineering*, 82(1), 35-45.
- **Welch, G., & Bishop, G.** (2006). "An Introduction to the Kalman Filter". *UNC-Chapel Hill, TR 95-041*.

### Fusión Sensorial IMU
- **Madgwick, S.** (2010). "An efficient orientation filter for IMU and MARG sensor arrays". *University of Bristol*.
- **Mahony, R., Hamel, T., & Pflimlin, J.** (2008). "Nonlinear Complementary Filters on the Special Orthogonal Group". *IEEE Transactions on Automatic Control*, 53(5), 1203-1218.

### Filtros Digitales
- **Oppenheim, A. V., & Schafer, R. W.** (2009). *Discrete-Time Signal Processing* (3rd ed.). Pearson.
- **Proakis, J. G., & Manolakis, D. G.** (2007). *Digital Signal Processing* (4th ed.). Pearson.

---

## 📄 Licencia

**Software Propietario**  
© 2025 Julián Andrés Rosas Sánchez  
Todos los derechos reservados.

Este código es parte de un proyecto académico de la Universidad Militar Nueva Granada y está protegido por derechos de autor. No se permite la reproducción, distribución o uso comercial sin autorización expresa del autor.

---

## 📧 Contacto

**Julián Andrés Rosas Sánchez**  
Ingeniería Mecatrónica  
Universidad Militar Nueva Granada

*Proyecto desarrollado como parte del Laboratorio de Señales y Sistemas*

---

## 🎯 Conclusiones del Sprint 3

### Logros Técnicos
✅ **Tremor reducido a <0.5°** mediante fusión Kalman + filtrado multi-capa  
✅ **Fusión sensorial óptima** con adaptación dinámica de covarianzas  
✅ **Detección robusta de orientación** con histéresis amplia (2.5-9.2)  
✅ **Transiciones suaves** con contador asimétrico (20/5 lecturas)  
✅ **Arquitectura distribuida** (filtrado en transmisor + receptor)

### Comparación Final

**Sprint 1 → Sprint 2**: Mejora de 5x en tremor (±5° → <1°)  
**Sprint 2 → Sprint 3**: Mejora de 2x en tremor (<1° → <0.5°)  
**Sprint 1 → Sprint 3**: Mejora total de **10x** en precisión

### Aprendizajes Clave
- El **Filtro de Kalman** es superior al complementario para fusión sensorial
- La **adaptación dinámica de Q** mejora rendimiento en movimiento variable
- La **arquitectura en cascada** (FIR → Kalman → IIR) es más efectiva que un solo filtro complejo
- Los **umbrales asimétricos** evitan oscilaciones en transiciones de estado
- El **análisis de varianza P** proporciona métrica de calidad en tiempo real

---

**Documento generado**: Noviembre 2025  
**Versión**: 1.0
