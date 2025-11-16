# Problemas con Serial Monitor en ESP32-S3

## 🔴 Problema: NO aparece NADA en Serial Monitor

Este es un problema **MUY COMÚN** con ESP32-S3. Aquí están todas las soluciones:

---

## ✅ **Solución 1: Configuración en Arduino IDE**

### Paso 1: Verificar Placa Seleccionada
```
Herramientas → Placa → ESP32 Arduino → ESP32S3 Dev Module
```

### Paso 2: Configuración USB CDC
```
Herramientas → USB CDC On Boot → "Enabled"   ← MUY IMPORTANTE
```

### Paso 3: Configuración UART
```
Herramientas → USB Mode → "Hardware CDC and JTAG"
```

### Paso 4: Puerto Serial
```
Herramientas → Upload Speed → 115200
Herramientas → Puerto → Seleccionar el COM correcto
```

---

## ✅ **Solución 2: Driver USB**

El ESP32-S3 puede necesitar drivers específicos:

### Windows:
1. Descargar driver CP210x o CH340 (según tu placa)
2. Instalar y reiniciar
3. Verificar en Administrador de Dispositivos

### Ver qué chip USB tiene tu placa:
- **CP2102**: Driver Silicon Labs
- **CH340**: Driver WCH
- **USB Nativo**: No necesita driver (pero puede dar problemas)

---

## ✅ **Solución 3: Secuencia de Conexión Correcta**

**IMPORTANTE**: El orden importa en ESP32-S3

### Método 1 (Recomendado):
```
1. Cerrar Serial Monitor
2. Presionar y mantener BOOT en ESP32-S3
3. Presionar RST (reset) una vez
4. Soltar BOOT
5. Subir código
6. Presionar RST después de subir
7. Esperar 3 segundos
8. Abrir Serial Monitor (115200 baud)
```

### Método 2 (Si el primero no funciona):
```
1. Desconectar USB
2. Conectar USB
3. Abrir Serial Monitor INMEDIATAMENTE
4. Presionar RST en ESP32-S3
5. Ver output
```

---

## ✅ **Solución 4: Probar código de test mínimo**

He creado un código de prueba simple:

**Archivo**: `Test_Serial_ESP32S3/Test_Serial_ESP32S3.ino`

### Qué hace:
- Parpadea LED 10 veces rápido al inicio
- Imprime info del chip
- Parpadea LED cada 1 segundo
- Imprime contador en Serial

### Si el LED parpadea pero NO hay Serial:
→ Problema de USB/Driver/Configuración Arduino IDE

### Si el LED NO parpadea:
→ Código no se subió correctamente

---

## ✅ **Solución 5: Verificar en Administrador de Dispositivos (Windows)**

### Presionar Win + X → Administrador de Dispositivos

Buscar en "Puertos (COM y LPT)":
```
✓ Debe aparecer:
  - "USB-SERIAL CH340 (COM4)" o similar
  - "Silicon Labs CP210x (COM5)" o similar

❌ Si aparece con "!" amarillo:
  - Click derecho → Actualizar driver
  - Buscar automáticamente

❌ Si NO aparece nada:
  - Cable USB malo
  - Puerto USB sin energía
  - ESP32-S3 dañado (raro)
```

---

## ✅ **Solución 6: Configuración Avanzada Arduino IDE**

Estas configuraciones funcionan mejor con ESP32-S3:

```
Herramientas:
├─ Board: "ESP32S3 Dev Module"
├─ USB CDC On Boot: "Enabled"          ← CRÍTICO
├─ CPU Frequency: "240MHz"
├─ Flash Mode: "QIO 80MHz"
├─ Flash Size: "4MB (32Mb)"            (ajustar según tu placa)
├─ Partition Scheme: "Default 4MB"
├─ PSRAM: "Disabled" (si no lo usas)
├─ Upload Speed: "921600"
├─ USB Mode: "Hardware CDC and JTAG"   ← CRÍTICO
└─ Arduino Runs On: "Core 1"
```

---

## ✅ **Solución 7: Probar con PlatformIO (alternativa)**

Si Arduino IDE sigue sin funcionar:

### Crear platformio.ini:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600
build_flags = 
    -DARDUINO_USB_CDC_ON_BOOT=1
```

---

## ✅ **Solución 8: Código con delay más largo**

He modificado el código del Receptor con:
```cpp
Serial.begin(115200);
delay(2000);  // Esperar 2 segundos (más tiempo que antes)

while (!Serial && millis() < 5000) {
    delay(10);  // Esperar hasta que Serial esté listo
}
```

---

## ✅ **Solución 9: Monitor Serial alternativo**

Si Arduino IDE Serial Monitor no funciona, probar:

### Opción 1: PuTTY
```
1. Descargar PuTTY
2. Configurar:
   - Connection Type: Serial
   - Serial line: COM4 (tu puerto)
   - Speed: 115200
3. Open
```

### Opción 2: Arduino CLI
```bash
arduino-cli monitor -p COM4 -c baudrate=115200
```

### Opción 3: VSCode Serial Monitor
```
Extension: "Serial Monitor" by Microsoft
```

---

## 🔍 **Checklist Diagnóstico**

Marca lo que YA verificaste:

```
Hardware:
☐ Cable USB es de DATOS (no solo carga)
☐ Puerto USB de la PC funciona (probar otro dispositivo)
☐ LED del ESP32-S3 enciende al conectar
☐ Botones BOOT y RST funcionan físicamente

Arduino IDE:
☐ Placa: ESP32S3 Dev Module
☐ USB CDC On Boot: Enabled
☐ USB Mode: Hardware CDC and JTAG
☐ Puerto COM correcto seleccionado
☐ Baudrate: 115200

Upload:
☐ Código compila sin errores
☐ Upload dice "Done uploading" o "Hard resetting..."
☐ LED parpadea durante upload
☐ Presioné BOOT si fue necesario

Serial Monitor:
☐ Baudrate: 115200
☐ Line Ending: "Both NL & CR" o "Newline"
☐ Abierto DESPUÉS de upload
☐ Presioné RST después de abrir monitor
```

---

## 🚨 **Si NADA funciona:**

### Test Final: Ejemplo Básico de Arduino
```
Archivo → Ejemplos → 01.Basics → Blink

1. Subir Blink al ESP32-S3
2. Verificar que LED interno parpadea
3. Si parpadea: ESP32 funciona, problema es Serial
4. Si NO parpadea: Problema de upload/placa
```

---

## 📝 **Información para Reportar**

Si sigues sin solución, necesito:

```
1. Modelo EXACTO de tu ESP32-S3:
   - ¿Qué dice en la placa?
   - Foto si es posible

2. Captura de pantalla de:
   - Herramientas en Arduino IDE
   - Administrador de Dispositivos (Windows)

3. Output de compilación:
   - Copiar TODO el texto de la ventana de Arduino IDE

4. ¿El LED parpadea cuando subes código?
   - Sí / No

5. ¿El LED parpadea al usar Test_Serial_ESP32S3.ino?
   - Sí / No / No probé
```

---

## 💡 **Configuración que SIEMPRE funciona:**

Esta es la configuración más compatible:

```
Board: ESP32S3 Dev Module
USB CDC On Boot: Enabled
Upload Speed: 115200  (más lento pero más confiable)
USB Mode: Hardware CDC and JTAG

Secuencia:
1. Cerrar Serial Monitor
2. Mantener BOOT, presionar RST, soltar BOOT
3. Subir código
4. Presionar RST
5. Esperar 5 segundos
6. Abrir Serial Monitor
7. Si no aparece nada, presionar RST de nuevo
```

---

## 📚 **Recursos Adicionales**

- [Documentación ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [Arduino ESP32 GitHub Issues](https://github.com/espressif/arduino-esp32/issues)
- [Foro Arduino ESP32](https://forum.arduino.cc/)

---

**Actualizado**: Noviembre 2025  
**Sprint 1** - Troubleshooting ESP32-S3
