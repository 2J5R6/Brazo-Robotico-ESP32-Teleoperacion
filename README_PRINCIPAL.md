# Sistema de Tele-operación - Brazo Robótico 2DOF

## 📚 Laboratorio de Señales - Práctica 3
**Universidad Militar Nueva Granada**  
**Semestre VI - Noviembre 2025**

---

## 🎯 Descripción del Proyecto

Este proyecto implementa un **sistema de tele-operación** para controlar un brazo robótico de 2 grados de libertad mediante movimientos sincrónicos con el brazo humano. El sistema captura el movimiento de la mano del operador usando un sensor inercial (IMU) y replica el movimiento en el brazo robótico en tiempo real.

El desarrollo se realiza en **3 sprints** progresivos:

1. **Sprint 1**: Sistema básico sin filtrado avanzado
2. **Sprint 2**: Implementación de filtros digitales
3. **Sprint 3**: Aplicación de filtros de Kalman (EKF/UKF)

---

## 📁 Estructura del Proyecto

```
Lab3/
├── Sprint1_SinFiltro/              # ✅ COMPLETADO
│   ├── Transmisor_Guante/
│   │   └── Transmisor_Guante.ino
│   ├── Receptor_Brazo/
│   │   └── Receptor_Brazo.ino
│   ├── README.md
│   ├── CONEXIONES.md
│   ├── EXPERIMENTO.md
│   ├── TROUBLESHOOTING.md
│   ├── BOM_Y_RESUMEN.md
│   └── analisis_datos.py
│
├── Sprint2_ConFiltros/             # 🔜 PRÓXIMO
│   └── (Pendiente)
│
├── Sprint3_Kalman/                 # 🔜 FUTURO
│   └── (Pendiente)
│
├── ConFiltro/                      # Código legacy
│   └── ConFiltro.ino
├── FiltroKalman/                   # Código legacy
│   └── FiltroKalman.ino
├── SinFiltro/                      # Código legacy
│   └── SinFiltro.ino
│
└── README.md                       # Este archivo
```

---

## 🚀 Inicio Rápido

### Sprint 1 (Actual)

1. **Navegar a la carpeta del sprint**:
   ```
   Lab3/Sprint1_SinFiltro/
   ```

2. **Leer documentación**:
   - `README.md` - Guía completa de uso
   - `CONEXIONES.md` - Diagramas de conexión
   - `BOM_Y_RESUMEN.md` - Lista de materiales

3. **Preparar hardware**:
   - Ver diagramas en `CONEXIONES.md`
   - Verificar componentes con `BOM_Y_RESUMEN.md`

4. **Subir código**:
   - Transmisor → ESP32-S3
   - Receptor → ESP32 WROOM

5. **Realizar experimentos**:
   - Usar plantilla en `EXPERIMENTO.md`

6. **Analizar resultados**:
   - Script Python en `analisis_datos.py`

---

## 🔧 Hardware Requerido

### Componentes Principales

| Componente | Cantidad | Uso |
|------------|----------|-----|
| ESP32 WROOM | 1 | Transmisor (guante) - Tiene DAC |
| ESP32-S3 | 1 | Receptor (brazo) - NO tiene DAC |
| MPU6050 | 1-2 | Sensor inercial |
| Servo MG90S | 2 | Actuadores del brazo |
| Fuente 5V/2A | 1 | Alimentación |

**NOTA CRÍTICA**: 
- ✅ **ESP32 WROOM va en el GUANTE** (tiene DAC para análisis)
- ✅ **ESP32-S3 va en el BRAZO ROBÓTICO** (no necesita DAC)
- ❌ **ESP32-S3 NO tiene DAC** - Por eso NO va en el guante

**Ver lista completa en**: `Sprint1_SinFiltro/BOM_Y_RESUMEN.md`

---

## 📊 Progreso del Proyecto

### Sprint 1: Sistema Básico ✅
- [x] Código transmisor (ESP32-S3 + MPU6050)
- [x] Código receptor (ESP32 + Servos)
- [x] Comunicación ESP-NOW broadcast
- [x] Salida DAC para análisis
- [x] Control por eje Z (arriba/abajo)
- [x] Documentación completa
- [ ] Experimentos realizados
- [ ] Análisis de resultados

### Sprint 2: Filtrado Digital 🔜
- [ ] Filtro paso bajo
- [ ] Promedio móvil
- [ ] Filtro complementario
- [ ] Comparación de resultados
- [ ] Documentación

### Sprint 3: Filtro de Kalman 🔜
- [ ] Modelo del sistema
- [ ] Implementación EKF
- [ ] Implementación UKF
- [ ] Comparación EKF vs UKF
- [ ] Análisis de costo computacional
- [ ] Documentación final

---

## 🎓 Objetivos de Aprendizaje

### Técnicos
- Captura y procesamiento de señales inerciales
- Comunicación inalámbrica ESP-NOW
- Control de actuadores en tiempo real
- Filtrado digital de señales
- Estimación de estado con Kalman
- Análisis de errores y precisión

### Prácticos
- Diseño de sistemas embebidos
- Integración hardware/software
- Metodología de trabajo por sprints
- Documentación técnica
- Análisis experimental

---

## 📖 Documentación por Sprint

### Sprint 1
- **README**: Guía general de uso
- **CONEXIONES**: Diagramas de hardware
- **EXPERIMENTO**: Plantilla de resultados
- **TROUBLESHOOTING**: Solución de problemas
- **BOM_Y_RESUMEN**: Materiales y resumen

### Sprint 2 (Próximamente)
- Comparación de filtros
- Diseño de parámetros
- Pruebas offline
- Implementación en tiempo real

### Sprint 3 (Próximamente)
- Modelo matemático del sistema
- Diseño de filtros de Kalman
- Análisis de desempeño
- Informe final

---

## 🎯 Metodología de Trabajo

### Por Sprint

Cada sprint sigue esta estructura:

1. **Planificación**
   - Definir objetivos
   - Revisar documentación anterior
   - Preparar hardware/software

2. **Desarrollo**
   - Implementar código
   - Realizar pruebas unitarias
   - Integrar componentes

3. **Experimentación**
   - Definir puntos de prueba
   - Recolectar datos
   - Documentar observaciones

4. **Análisis**
   - Procesar datos
   - Calcular errores
   - Generar gráficas

5. **Documentación**
   - Completar plantillas
   - Responder preguntas
   - Preparar informe

---

## 📈 Métricas de Evaluación

### Por Sprint

| Métrica | Sprint 1 | Sprint 2 | Sprint 3 |
|---------|----------|----------|----------|
| Precisión de posición | ±5-10 cm | ±2-5 cm | ±1-2 cm |
| Latencia | 10-20 ms | 20-40 ms | 30-50 ms |
| Estabilidad | Baja | Media | Alta |
| Uso de CPU | ~10% | ~20-30% | ~40-60% |

### Criterios de Éxito

- ✅ Sistema funciona de forma estable
- ✅ Comunicación sin pérdida de paquetes
- ✅ Precisión dentro del rango esperado
- ✅ Documentación completa
- ✅ Experimentos reproducibles

---

## 🛠️ Herramientas Utilizadas

### Software
- **Arduino IDE** 2.x - Desarrollo
- **PlatformIO** (opcional) - Desarrollo avanzado
- **Python 3.x** - Análisis de datos
- **MATLAB** (opcional) - Análisis avanzado
- **Git** - Control de versiones

### Librerías Arduino
- `esp_now.h` - Comunicación
- `WiFi.h` - Conectividad
- `Adafruit_MPU6050` - Sensor inercial
- `ESP32Servo` - Control de servos
- `driver/dac.h` - Salida analógica

### Librerías Python
- `numpy` - Cálculos numéricos
- `matplotlib` - Visualización
- `pandas` - Manejo de datos
- `scipy` - Procesamiento de señales

---

## 📞 Información de Contacto

### Equipo de Trabajo
- Integrante 1: _______________
- Integrante 2: _______________
- Integrante 3: _______________

### Instructor
- Laboratorio de Señales
- Universidad Militar Nueva Granada
- Semestre VI - 2025

---

## 📝 Entregables

### Por Sprint
- [ ] Código fuente completo
- [ ] Documentación técnica
- [ ] Tabla de experimentos
- [ ] Análisis de errores
- [ ] Respuestas a preguntas
- [ ] Fotos/videos del montaje
- [ ] Capturas del osciloscopio

### Final
- [ ] Informe completo del proyecto
- [ ] Comparación entre sprints
- [ ] Conclusiones y recomendaciones
- [ ] Código completo documentado

---

## 🔗 Referencias

### Documentación Oficial
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [ESP-NOW Protocol Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [MPU6050 Product Specification](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)

### Tutoriales
- [ESP-NOW Getting Started](https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/)
- [MPU6050 Tutorial](https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/)
- [Kalman Filter Tutorial](https://www.kalmanfilter.net/)

---

## 📅 Calendario

| Semana | Sprint | Actividades |
|--------|--------|-------------|
| 1-2 | Sprint 1 | Montaje, programación, pruebas |
| 3-4 | Sprint 2 | Filtros, experimentos |
| 5-7 | Sprint 3 | Kalman, análisis final |
| 8 | Final | Documentación, entrega |

---

## ⚠️ Notas Importantes

### Asignación Correcta de ESP32
- ✅ **ESP32 WROOM → GUANTE (Transmisor)**
  - Tiene DAC (GPIO25/26) necesario para análisis en Sprint 1
  - I2C estándar: SDA=GPIO21, SCL=GPIO22
  
- ✅ **ESP32-S3 → BRAZO ROBÓTICO (Receptor)**
  - NO tiene DAC (no lo necesita en el brazo)
  - Control de servos: GPIO12 y GPIO13
  - LED: GPIO48
  - I2C opcional: SDA=GPIO8, SCL=GPIO10

### Reglas de GPIO ESP32-S3 (Brazo)
- ✅ **Usar GPIO seguros**: 1-10, 12-15, 21, 45-48
- ❌ **NUNCA usar GPIO 19/20** (USB D-/D+)
- ❌ **GPIO 9 tiene problemas** - usar GPIO 10 para I2C SCL
- ❌ **ESP32-S3 NO tiene DAC**

### GPIO ESP32 WROOM (Guante)
- ✅ I2C estándar: SDA=GPIO21, SCL=GPIO22
- ✅ DAC1=GPIO25, DAC2=GPIO26
- ✅ LED integrado: GPIO2

### Seguridad
- ⚠️ Desconectar alimentación antes de cambiar conexiones
- ⚠️ No exceder 3.3V en pines GPIO
- ⚠️ Servos requieren fuente externa (5V/2A mínimo)
- ⚠️ Verificar polaridad de componentes

---

## 🎓 Resultados de Aprendizaje Esperados

Al completar este proyecto, el estudiante será capaz de:

1. **Diseñar** sistemas de captura de movimiento con sensores inerciales
2. **Implementar** comunicación inalámbrica entre dispositivos embebidos
3. **Desarrollar** algoritmos de filtrado digital de señales
4. **Aplicar** filtros de Kalman para estimación de estado
5. **Analizar** el desempeño de sistemas de control en tiempo real
6. **Documentar** proyectos técnicos de forma profesional

---

## 📊 Estado Actual del Proyecto

```
SPRINT 1: ████████████████████░░ 90% COMPLETADO
  ├─ Código          ██████████████████████ 100%
  ├─ Documentación   ██████████████████████ 100%
  ├─ Hardware        ████████████░░░░░░░░░░  60%
  └─ Experimentos    ░░░░░░░░░░░░░░░░░░░░░░   0%

SPRINT 2: ░░░░░░░░░░░░░░░░░░░░░░  0% PENDIENTE

SPRINT 3: ░░░░░░░░░░░░░░░░░░░░░░  0% PENDIENTE
```

---

## 🚀 ¡Comencemos!

Para iniciar con el Sprint 1:

```bash
cd Sprint1_SinFiltro
# Leer README.md para instrucciones detalladas
```

---

**Última actualización**: Noviembre 10, 2025  
**Versión**: 1.0  
**Estado**: Sprint 1 en desarrollo

---

¡Éxito con el proyecto! 🎉
