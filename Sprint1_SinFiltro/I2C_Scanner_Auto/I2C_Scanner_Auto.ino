/*
 * I2C SCANNER AVANZADO - Diagnóstico con WHO_AM_I
 * 
 * Este código:
 * 1. Escanea TODAS las combinaciones posibles de pines I2C
 * 2. Lee el registro WHO_AM_I (0x75) de cada dispositivo encontrado
 * 3. Identifica MPU6050, clones y otros sensores
 * 
 * Registro WHO_AM_I:
 * - MPU6050 original: 0x68 o 0x70
 * - MPU6500: 0x70
 * - MPU9250: 0x71
 * - Clones: pueden responder valores diferentes
 */

#include <Wire.h>

// Registro WHO_AM_I estándar para sensores MPU
#define WHO_AM_I_REG 0x75

// Pines a probar
int pinsSDA[] = {4, 21, 16, 23, 5, 18};
int pinsSCL[] = {5, 22, 17, 19, 4, 23};

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║   I2C SCANNER + WHO_AM_I DETECTOR         ║");
  Serial.println("║   Identificación de MPU6050 y clones      ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  delay(500);
  
  Serial.println("Probando combinaciones comunes:\n");
  
  testI2C(4, 5, "GPIO4 (SDA), GPIO5 (SCL) - Tu config actual");
  testI2C(21, 22, "GPIO21 (SDA), GPIO22 (SCL) - Estándar");
  testI2C(16, 17, "GPIO16 (SDA), GPIO17 (SCL) - Alternativa");
  testI2C(23, 19, "GPIO23 (SDA), GPIO19 (SCL) - Otra opción");
  testI2C(5, 4, "GPIO5 (SDA), GPIO4 (SCL) - Invertida");
  testI2C(18, 19, "GPIO18 (SDA), GPIO19 (SCL) - HSPI");
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║          ESCANEO FINALIZADO                ║");
  Serial.println("╚════════════════════════════════════════════╝");
  
  Serial.println("\n📋 INTERPRETACIÓN DE RESULTADOS:\n");
  Serial.println("WHO_AM_I = 0x68 → MPU6050 original ✓");
  Serial.println("WHO_AM_I = 0x70 → MPU6050/MPU6500 ✓");
  Serial.println("WHO_AM_I = 0x71 → MPU9250 (también compatible)");
  Serial.println("WHO_AM_I = 0x72 → Posible clon funcional");
  Serial.println("WHO_AM_I = 0x00 o 0xFF → Clon problemático ⚠️");
  Serial.println("WHO_AM_I = otro valor → Anótalo y prueba librería MPU6050_light\n");
  
  Serial.println("📝 PRÓXIMOS PASOS:\n");
  Serial.println("Si encontraste WHO_AM_I válido (0x68-0x72):");
  Serial.println("  1. Usa esos pines GPIO en Transmisor_Guante.ino");
  Serial.println("  2. Si Adafruit_MPU6050 falla, prueba librería MPU6050_light");
  Serial.println("\nSi WHO_AM_I = 0x00 o 0xFF:");
  Serial.println("  1. El clon es de baja calidad");
  Serial.println("  2. Usa el otro MPU6050 que SÍ funciona");
  Serial.println("  3. Considera comprar uno original GY-521\n");
}

void loop() {
  // Nada - todo en setup
}

void testI2C(int sda, int scl, String description) {
  Serial.print("🔍 ");
  Serial.print(description);
  Serial.print("... ");
  
  // Inicializar I2C
  Wire.begin(sda, scl);
  Wire.setClock(100000);  // 100kHz para mayor compatibilidad
  delay(100);
  
  // Escanear direcciones
  byte error, address;
  int nDevices = 0;
  bool foundMPU = false;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      nDevices++;
      
      // Mostrar información del dispositivo
      if (nDevices == 1) {
        Serial.println();  // Nueva línea para mejor formato
      }
      
      Serial.print("   └─ Dispositivo en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (dec: ");
      Serial.print(address);
      Serial.print(")");
      
      // Intentar leer WHO_AM_I
      uint8_t whoAmI = readWhoAmI(address);
      
      Serial.print(" → WHO_AM_I: 0x");
      if (whoAmI < 16) Serial.print("0");
      Serial.print(whoAmI, HEX);
      
      // Interpretar el valor
      String deviceType = identifyDevice(address, whoAmI);
      Serial.print(" ");
      Serial.println(deviceType);
      
      // Verificar si es MPU6050 o compatible
      if ((address == 0x68 || address == 0x69) && 
          (whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x71 || whoAmI == 0x72)) {
        foundMPU = true;
      }
    }
  }
  
  // Resumen
  if (foundMPU) {
    Serial.println("\n   ╔══════════════════════════════════╗");
    Serial.println("   ║  ✅ MPU6050 COMPATIBLE FOUND!   ║");
    Serial.println("   ╚══════════════════════════════════╝");
    Serial.print("   📌 Pines: SDA=GPIO");
    Serial.print(sda);
    Serial.print(", SCL=GPIO");
    Serial.println(scl);
    Serial.println("   👍 Usa estos pines en tu código!\n");
  } else if (nDevices > 0) {
    Serial.print("\n   ⚠️  ");
    Serial.print(nDevices);
    Serial.println(" dispositivo(s) encontrado(s), pero NO MPU compatible\n");
  } else {
    Serial.println("❌ Sin dispositivos");
  }
  
  Wire.end();
  delay(300);
}

// Función para leer el registro WHO_AM_I
uint8_t readWhoAmI(uint8_t deviceAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(WHO_AM_I_REG);  // Registro 0x75
  byte error = Wire.endTransmission(false);  // Repeated start
  
  if (error != 0) {
    return 0xFF;  // Error de lectura
  }
  
  Wire.requestFrom(deviceAddress, (uint8_t)1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  
  return 0xFF;  // No hay datos disponibles
}

// Función para identificar el dispositivo según dirección y WHO_AM_I
String identifyDevice(uint8_t address, uint8_t whoAmI) {
  // MPU6050 en dirección correcta
  if (address == 0x68 || address == 0x69) {
    if (whoAmI == 0x68) {
      return "→ ✅ MPU6050 ORIGINAL";
    } else if (whoAmI == 0x70) {
      return "→ ✅ MPU6050/MPU6500 (compatible)";
    } else if (whoAmI == 0x71) {
      return "→ ✅ MPU9250 (compatible con MPU6050)";
    } else if (whoAmI == 0x72) {
      return "→ ⚠️  Posible CLON funcional - prueba MPU6050_light";
    } else if (whoAmI == 0x00) {
      return "→ ❌ CLON DE BAJA CALIDAD (WHO_AM_I = 0x00)";
    } else if (whoAmI == 0xFF) {
      return "→ ❌ Error de lectura o clon problemático";
    } else {
      return "→ ⚠️  CLON desconocido - anota este valor: 0x" + String(whoAmI, HEX);
    }
  }
  
  // Otras direcciones conocidas
  if (address == 0x1E) return "→ HMC5883L Magnetómetro";
  if (address == 0x77 || address == 0x76) return "→ BMP280/BME280 Presión";
  if (address == 0x3C || address == 0x3D) return "→ OLED Display";
  if (address == 0x48) return "→ ADS1115 ADC";
  if (address == 0x50) return "→ EEPROM AT24Cxx";
  
  return "→ Dispositivo desconocido";
}