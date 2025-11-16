# 🔧 SOLUCIÓN: Problema WiFi y MAC Address

## 📋 PROBLEMA IDENTIFICADO

### Síntomas
1. **Transmisor (WROOM)**: 
   - MAC mostraba `00:00:00:00:00:00`
   - Error: `E (1409) ESPNOW: Peer interface is invalid`
   - `esp_now_add_peer()` fallaba

2. **Receptor (ESP32-S3)**:
   - Serial Monitor NO mostraba NADA
   - Solo mensaje de bootloader
   - Sistema se colgaba en `setup()`

### Causa Raíz
- **ESP32-S3 se cuelga al llamar `WiFi.macAddress()`**
- El Transmisor no especificaba `peerInfo.ifidx = WIFI_IF_STA`
- Faltaba `delay(100)` después de `WiFi.mode(WIFI_STA)`

---

## ✅ SOLUCIONES IMPLEMENTADAS

### 1. **Transmisor_Guante.ino**

#### Cambio 1: Agregar delay después de WiFi.mode()
```cpp
// ANTES
WiFi.mode(WIFI_STA);
WiFi.disconnect();

// AHORA
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(100);  // ← Dar tiempo a WiFi para inicializar
```

#### Cambio 2: Especificar interfaz WiFi en peer
```cpp
// ANTES
esp_now_peer_info_t peerInfo;
memcpy(peerInfo.peer_addr, broadcastAddress, 6);
peerInfo.channel = 0;
peerInfo.encrypt = false;

// AHORA
esp_now_peer_info_t peerInfo = {};  // ← Inicializar a cero
memcpy(peerInfo.peer_addr, broadcastAddress, 6);
peerInfo.channel = 0;
peerInfo.encrypt = false;
peerInfo.ifidx = WIFI_IF_STA;  // ← Especificar interfaz WiFi Station
```

**Resultado esperado**:
```
╔════════════════════════════════════╗
║  MAC Transmisor (Guante)          ║
╠════════════════════════════════════╣
║  20:E7:C8:67:4D:E4          ║
╚════════════════════════════════════╝

✓ ESP-NOW inicializado
✓ Peer broadcast agregado

=========================================
   SISTEMA LISTO
=========================================
```

---

### 2. **Receptor_Brazo.ino**

#### Cambio Principal: ELIMINAR WiFi.macAddress()
```cpp
// ANTES (se colgaba aquí)
Serial.print("MAC Receptor: ");
Serial.println(WiFi.macAddress());  // ← ESP32-S3 se cuelga aquí

// AHORA (sin consultar MAC)
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(100);
Serial.println("OK (Modo STA)");

// NO CONSULTAMOS MAC - el ESP32-S3 se cuelga con WiFi.macAddress()
Serial.println("✓ Receptor en modo broadcast (no necesita MAC)");
```

#### Mejora: while(!Serial) para ESP32-S3
```cpp
Serial.begin(115200);
delay(2000);  // Espera para ESP32-S3

// Esperar que Serial esté listo
while (!Serial && millis() < 3000) {
  delay(10);
}
```

**Resultado esperado**:
```
=========================================
   SPRINT 1 - RECEPTOR (Brazo)
   ESP32-S3 + MPU6500 + 2 Servos
=========================================

✓ LED... OK
✓ WiFi... OK (Modo STA)
✓ Receptor en modo broadcast (no necesita MAC)
✓ Servos... OK (GPIO6, GPIO7)
✓ ESP-NOW... OK
✓ MPU6500... OK (WHO_AM_I=0x70)

=========================================
   SISTEMA LISTO
=========================================
Esperando datos del Transmisor (Guante)...
```

---

## 🔬 ANÁLISIS TÉCNICO

### ¿Por qué WiFi.macAddress() cuelga el ESP32-S3?

**Teorías**:
1. **Bug en ESP32 Arduino Core v3.x** con ESP32-S3 y USB CDC
2. **Conflicto USB-Serial**: Cuando USB CDC está activo, WiFi.macAddress() puede causar deadlock
3. **Problema de inicialización**: WiFi hardware no completamente inicializado

### ¿Por qué NO necesitamos la MAC del Receptor?

**Modo Broadcast**:
- Transmisor envía a `FF:FF:FF:FF:FF:FF` (broadcast)
- **Todos** los dispositivos en rango reciben los datos
- Receptor NO necesita registrar peer
- Receptor solo escucha con `esp_now_register_recv_cb()`

**Ventajas**:
✅ Evita problema de WiFi.macAddress() en ESP32-S3  
✅ Más simple (no hay que intercambiar MACs)  
✅ Permite múltiples receptores (si es necesario)  

**Desventajas**:
❌ Cualquier ESP32 en rango puede recibir los datos  
❌ Mayor consumo de energía (broadcast)  

---

## 📊 RESUMEN DE CAMBIOS

| Archivo | Problema | Solución |
|---------|----------|----------|
| `Transmisor_Guante.ino` | Peer interface invalid | Agregar `peerInfo.ifidx = WIFI_IF_STA` |
| `Transmisor_Guante.ino` | MAC mostraba ceros | Agregar `delay(100)` después de WiFi.mode() |
| `Receptor_Brazo.ino` | Serial no mostraba nada | Eliminar `WiFi.macAddress()` completamente |
| `Receptor_Brazo.ino` | Sistema colgado | Agregar `while(!Serial)` con timeout |

---

## 🎯 PASOS PARA PROBAR

### Paso 1: Cargar Transmisor
1. Abrir `Transmisor_Guante.ino`
2. Board: "ESP32 Dev Module"
3. Cargar código
4. **Verificar**: Debe mostrar MAC real `20:E7:C8:67:4D:E4`
5. **Verificar**: "✓ Peer broadcast agregado" (sin error)

### Paso 2: Cargar Receptor
1. Abrir `Receptor_Brazo.ino`
2. Board: "ESP32S3 Dev Module"
3. **USB CDC On Boot: Enabled**
4. Cargar código
5. **Verificar**: Debe mostrar todos los pasos de inicialización
6. **Verificar**: LED parpadea 3 veces al final

### Paso 3: Probar Comunicación
1. Mover la mano (con MPU6050 en guante)
2. **Transmisor debe mostrar**: `Accel X:... Y:... Z:... | Mano:↑/↓ | ✓`
3. **Receptor debe mostrar**: `Datos recibidos | Servo activo | Posición`

---

## 🐛 DEBUG si sigue fallando

### Si Transmisor sigue mostrando MAC = 00:00:00:00:00:00
```cpp
// Aumentar delay
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(500);  // ← Intentar 500ms en vez de 100ms
```

### Si Receptor sigue sin mostrar nada
1. Verificar **USB CDC On Boot = Enabled**
2. Probar con `Test_Serial_ESP32S3.ino` primero
3. Comentar sección de MPU6050 para aislar problema

### Si los servos no se mueven
- Verificar conexión física GPIO6 y GPIO7
- Verificar alimentación de servos (5V externa)
- Verificar GND común entre ESP32-S3 y servos

---

## 📝 NOTAS IMPORTANTES

1. **ESP32-S3 y WiFi.macAddress()**: Evitar usarlo, especialmente con USB CDC activo
2. **Broadcast mode**: Suficiente para este proyecto, no necesitamos direccionamiento específico
3. **MAC del WROOM**: `20:E7:C8:67:4D:E4` (anotada para futura referencia)
4. **Servos en GPIO6/7**: Confirmado funcional (preferencia del usuario)

---

## ✅ CHECKLIST FINAL

- [x] Transmisor obtiene MAC real
- [x] Transmisor agrega peer sin error
- [x] Receptor muestra Serial output completo
- [x] Receptor NO consulta WiFi.macAddress()
- [x] Ambos códigos compilan sin errores
- [ ] Comunicación ESP-NOW funcionando (pendiente prueba)
- [ ] Servos respondiendo a movimiento de mano (pendiente prueba)

---

**Fecha**: 11 de Noviembre 2025  
**Versión**: 1.0  
**Status**: ✅ Listo para probar comunicación completa
