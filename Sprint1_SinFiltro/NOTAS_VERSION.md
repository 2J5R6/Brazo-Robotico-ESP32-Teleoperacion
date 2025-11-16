# Notas de Compatibilidad - ESP32 Arduino Core

## ⚠️ Cambios Importantes en el Código

### ESP32 Arduino Core v3.x

El código ha sido **actualizado** para ser compatible con **ESP32 Arduino Core v3.0+** (basado en ESP-IDF 5.x).

---

## 🔧 Cambios en ESP-NOW Callbacks

### ❌ Versión Antigua (Core v2.x)
```cpp
// NO FUNCIONA en Core v3.x
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // ...
}

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // ...
}
```

### ✅ Versión Nueva (Core v3.x) - IMPLEMENTADA
```cpp
// CORRECTO para Core v3.x
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // ...
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  // ...
}
```

---

## 📋 Versión Recomendada

**ESP32 Arduino Core**: v3.0.0 o superior

### Verificar versión instalada:
1. Arduino IDE → Herramientas → Placa → Gestor de Tarjetas
2. Buscar "esp32"
3. Ver versión instalada de "esp32 by Espressif Systems"

### Instalar/Actualizar:
1. Arduino IDE → Herramientas → Placa → Gestor de Tarjetas
2. Buscar "esp32"
3. Seleccionar versión 3.0.0 o superior
4. Clic en "Instalar" o "Actualizar"

---

## 🔍 Diferencias Principales

| Aspecto | Core v2.x | Core v3.x (Actual) |
|---------|-----------|-------------------|
| **IDF Base** | ESP-IDF 4.4 | ESP-IDF 5.x |
| **Callback Send** | `const uint8_t *mac_addr` | `const wifi_tx_info_t *tx_info` |
| **Callback Recv** | `const uint8_t *mac` | `const esp_now_recv_info_t *recv_info` |
| **Estructura** | Más simple | Más información disponible |

---

## 💡 Ventajas del Core v3.x

1. ✅ Basado en ESP-IDF 5.x (más estable)
2. ✅ Mejor soporte para ESP32-S3
3. ✅ Correcciones de bugs
4. ✅ Mejor rendimiento WiFi
5. ✅ Más información en callbacks (RSSI, etc.)

---

## 🛠️ Si Usas Core v2.x

Si por alguna razón necesitas usar Core v2.x, modifica los callbacks:

### Transmisor_Guante.ino
```cpp
// Para Core v2.x
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
```

### Receptor_Brazo.ino
```cpp
// Para Core v2.x
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();
  // ... resto del código
}
```

---

## 📊 Información Adicional en Core v3.x

### Estructura `wifi_tx_info_t`
```cpp
typedef struct {
    uint8_t dest_addr[6];  // Dirección MAC destino
    int8_t rssi;           // RSSI de la transmisión
    // ... otros campos
} wifi_tx_info_t;
```

### Estructura `esp_now_recv_info_t`
```cpp
typedef struct {
    uint8_t src_addr[6];   // Dirección MAC origen
    uint8_t des_addr[6];   // Dirección MAC destino
    int8_t rx_ctrl_rssi;   // RSSI de recepción
    // ... otros campos
} esp_now_recv_info_t;
```

### Ejemplo de uso avanzado:
```cpp
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  // Obtener dirección MAC del transmisor
  Serial.print("MAC origen: ");
  for(int i = 0; i < 6; i++) {
    Serial.printf("%02X", recv_info->src_addr[i]);
    if(i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Obtener RSSI (calidad de señal)
  Serial.print("RSSI: ");
  Serial.println(recv_info->rx_ctrl_rssi);
  
  // Procesar datos
  memcpy(&receivedData, data, sizeof(receivedData));
}
```

---

## 🔄 Actualización del Código

El código actual **YA ESTÁ ACTUALIZADO** para Core v3.x.

Los archivos modificados:
- ✅ `Transmisor_Guante/Transmisor_Guante.ino`
- ✅ `Receptor_Brazo/Receptor_Brazo.ino`

---

## 📝 Resumen

| Item | Estado |
|------|--------|
| Código compatible con Core v3.x | ✅ Sí |
| Código compatible con Core v2.x | ❌ No (requiere modificación) |
| Versión recomendada | v3.0.0+ |
| Callbacks actualizados | ✅ Sí |

---

**Última actualización**: Noviembre 10, 2025  
**Versión del código**: Compatible con ESP32 Arduino Core v3.x
