# Guía de Debug I2C - MPU6050

## 🔍 Problema: "No se detectó MPU6050"

---

## 📋 Checklist Rápido

### 1. Verificación Física

- [ ] **MPU6050 alimentado con 3.3V** (NO 5V)
- [ ] **GND conectado** a GND del ESP32
- [ ] **Cables cortos** (< 20cm ideales)
- [ ] **Conexiones firmes** (no flojas)
- [ ] **Protoboard en buen estado** (contactos limpios)

### 2. Verificación con Multímetro

```
MPU6050 VCC → Debe medir 3.3V
MPU6050 GND → Continuidad con GND del ESP32
SDA/SCL     → Verificar continuidad cable
```

### 3. Verificación Visual del MPU6050

- [ ] LED del módulo enciende (si tiene)
- [ ] No hay componentes sueltos
- [ ] PCB no tiene daño visible

---

## 🔧 Opciones de Pines I2C para Probar

El código ahora tiene **4 opciones** de pines I2C. Prueba una por una:

### **OPCIÓN 1: Pines Estándar** (Comentada por defecto)
```cpp
#define SDA_PIN 21
#define SCL_PIN 22
```
```
MPU6050      ESP32 WROOM
-------      -----------
SDA    ───► GPIO 21
SCL    ───► GPIO 22
VCC    ───► 3.3V
GND    ───► GND
```

### **OPCIÓN 2: Pines Alternativos** (✅ ACTIVA)
```cpp
#define SDA_PIN 4
#define SCL_PIN 5
```
```
MPU6050      ESP32 WROOM
-------      -----------
SDA    ───► GPIO 4
SCL    ───► GPIO 5
VCC    ───► 3.3V
GND    ───► GND
```

### **OPCIÓN 3: Otra Alternativa**
```cpp
#define SDA_PIN 16
#define SCL_PIN 17
```
```
MPU6050      ESP32 WROOM
-------      -----------
SDA    ───► GPIO 16
SCL    ───► GPIO 17
VCC    ───► 3.3V
GND    ───► GND
```

### **OPCIÓN 4: GPIOs Seguros**
```cpp
#define SDA_PIN 23
#define SCL_PIN 19
```
```
MPU6050      ESP32 WROOM
-------      -----------
SDA    ───► GPIO 23
SCL    ───► GPIO 19
VCC    ───► 3.3V
GND    ───► GND
```

---

## 🔄 Cómo Cambiar de Opción

1. Abrir `Transmisor_Guante.ino`
2. Buscar la sección "CONFIGURACIÓN DE PINES"
3. **Comentar** la opción actual (agregar `//` al inicio)
4. **Descomentar** otra opción (quitar `//`)

### Ejemplo:
```cpp
// OPCIÓN 1 (Comentada)
// #define SDA_PIN 21
// #define SCL_PIN 22

// OPCIÓN 2 (Activa)
#define SDA_PIN 4
#define SCL_PIN 5
```

5. Subir código nuevamente
6. Abrir Monitor Serial
7. Ver resultado del escaneo I2C

---

## 📊 Interpretando el Escaneo I2C

El código ahora escanea automáticamente el bus I2C:

### ✅ **Si Encuentra el MPU6050**:
```
=== ESCANEO I2C ===
Dispositivo I2C en 0x68
✓ MPU6050 detectado
```
O también puede aparecer en:
```
Dispositivo I2C en 0x69
```

### ❌ **Si NO Encuentra Nada**:
```
=== ESCANEO I2C ===
No se encontraron dispositivos I2C

Prueba:
1. Verificar conexiones físicas
2. Probar otros pines I2C
3. Verificar alimentación del MPU6050
```

---

## 🛠️ Soluciones Paso a Paso

### Problema 1: No aparece nada en el escaneo

**Causas**:
- Cable SDA o SCL desconectado
- MPU6050 sin alimentación
- Corto circuito

**Solución**:
1. Con multímetro, verificar 3.3V en VCC del MPU6050
2. Verificar continuidad de cables
3. Probar con otro MPU6050 (si disponible)

### Problema 2: Aparece en dirección incorrecta

**El MPU6050 puede estar en**:
- `0x68` (por defecto, AD0 a GND)
- `0x69` (si AD0 está conectado a 3.3V)

**Solución**:
- Dejar AD0 sin conectar (flotante → 0x68)
- O conectar AD0 a GND explícitamente

### Problema 3: Pines ocupados por otra función

**Algunos pines tienen funciones especiales**:
- GPIO 1/3: UART (Serial)
- GPIO 6-11: Flash (NO USAR)
- GPIO 34-39: Solo INPUT

**Solución**:
- Usar opciones de pines proporcionadas
- Evitar GPIO 1, 3, 6-11, 34-39

---

## 🔬 Test Manual con I2C Scanner

Si quieres un test independiente:

```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);  // SDA, SCL - Cambiar según opción
  
  Serial.println("\nEscaneando I2C...");
  
  for(byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Encontrado en 0x");
      if(addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Escaneo completo");
}

void loop() {}
```

---

## 📐 Diagrama de Conexión (Opción 2 - GPIO 4/5)

```
      ESP32 WROOM              MPU6050
    ┌──────────────┐         ┌─────────┐
    │              │         │         │
    │   GPIO 4 ────┼────────►│ SDA     │
    │   GPIO 5 ────┼────────►│ SCL     │
    │              │         │         │
    │   3.3V   ────┼────────►│ VCC     │
    │   GND    ────┼────────►│ GND     │
    │              │         │         │
    │   GPIO 25────┼─► DAC   │         │
    │   GPIO 2 ────┼─► LED   │ [AD0]   │ (dejar sin conectar)
    │              │         │ [INT]   │ (dejar sin conectar)
    └──────────────┘         └─────────┘
```

---

## 💡 Tips Adicionales

### 1. Resistencias Pull-up
El MPU6050 **ya tiene** resistencias pull-up internas (generalmente 2.2kΩ). No es necesario agregar externas.

### 2. Velocidad I2C
Si el problema persiste, puedes reducir la velocidad:
```cpp
Wire.begin(SDA_PIN, SCL_PIN);
Wire.setClock(100000);  // 100kHz (más lento = más confiable)
```

### 3. Cables y Protoboard
- Usar cables cortos (< 20cm)
- Protoboard de buena calidad
- Evitar cables con mal contacto

### 4. Múltiples MPU6050
Si tienes otro MPU6050:
1. Pruébalo para descartar módulo defectuoso
2. Verifica que no sea un módulo GY-521 falso

---

## 📞 Preguntas Frecuentes

**P: ¿Por qué GPIO 21/22 no funciona?**
R: Puede haber conflicto con otra función. Prueba GPIO 4/5 que es muy confiable.

**P: ¿Puedo usar 5V en el MPU6050?**
R: ⚠️ NO. Solo 3.3V. Algunos módulos tienen regulador, pero es arriesgado.

**P: ¿Qué es la dirección I2C 0x68?**
R: Es la dirección por defecto del MPU6050. Es como su "número de teléfono" en el bus I2C.

**P: ¿Funciona con 400kHz?**
R: Sí, el MPU6050 soporta hasta 400kHz (Fast Mode). Por defecto Wire usa 100kHz.

---

## ✅ Checklist Final

Antes de rendirte:

- [ ] Probé las 4 opciones de pines
- [ ] Verifiqué voltaje 3.3V con multímetro
- [ ] Verifiqué continuidad de cables
- [ ] Cables cortos (< 20cm)
- [ ] Protoboard en buen estado
- [ ] AD0 del MPU6050 desconectado
- [ ] No hay cortos entre pines
- [ ] Reinicié el ESP32 después de cambiar pines
- [ ] Subí el código actualizado
- [ ] Abrí el Monitor Serial a 115200 baud

---

**Última actualización**: Noviembre 10, 2025  
**Con escaneo automático I2C integrado**
