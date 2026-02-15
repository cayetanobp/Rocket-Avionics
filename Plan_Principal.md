# Proyecto Cohete - Arquitectura Modular para Simulación en Proteus

## Estructura del Repositorio

```
rocket-avionics/
├── README.md
├── docs/
│   ├── architecture.md
│   ├── requirements.md
│   └── simulation_guide.md
├── 01-power-management/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 02-mcu-core/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 03-sensors/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 04-imu-navigation/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 05-barometer-altimeter/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 06-gnss-gps/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 07-telemetry-radio/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 08-data-logging/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 09-recovery-system/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 10-flight-control/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 11-state-machine/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
├── 12-diagnostics-debug/
│   ├── README.md
│   ├── schematics/
│   ├── simulation/
│   └── code/
└── integration/
    ├── full-system-schematic/
    ├── full-system-simulation/
    ├── pcb-design/
    └── tests/
```

---

## MÓDULO 01: POWER MANAGEMENT (Gestión de Energía)

### Descripción
Sistema de alimentación, regulación y monitorización de energía para todos los subsistemas del cohete.

### Componentes Principales
- **Batería LiPo**: 2S o 3S (7.4V o 11.1V)
- **Reguladores de voltaje**:
  - Buck converter 5V (para servos, sensores de 5V)
  - LDO 3.3V (para MCU, sensores I2C/SPI)
- **Monitor de batería**: INA219 o similar (I2C)
- **Circuito de protección**: Fusibles, diodos de protección
- **Indicadores LED**: Estado de carga, niveles de voltaje

### Funcionalidades
- Conversión de voltaje para diferentes subsistemas
- Monitorización de voltaje y corriente en tiempo real
- Detección de batería baja
- Apagado automático por voltaje crítico
- Protección contra sobrecorriente y cortocircuitos

### Interfaces
- **Salidas**: 
  - 5V para servos y sistemas de potencia
  - 3.3V para MCU y sensores digitales
- **I2C**: Comunicación con MCU para telemetría de potencia
- **GPIO**: Señales de alarma (batería baja, fallo)

### Simulación en Proteus
- Usar potenciómetros para simular descarga de batería
- Virtual Terminal para logs de estado de energía
- LEDs para indicadores visuales
- Gráficos de voltaje/corriente en tiempo real

---

## MÓDULO 02: MCU CORE (Núcleo del Microcontrolador)

### Descripción
Cerebro del sistema. Ejecuta el RTOS y coordina todos los módulos.

### Componentes Principales
- **MCU**: STM32F4 o similar (con soporte VSM en Proteus)
- **Oscilador externo**: 8MHz o 16MHz
- **Circuito de reset**: Botón + resistencia pull-up
- **Programación**: Conector SWD/JTAG
- **Indicador de actividad**: LED de heartbeat

### Funcionalidades
- Ejecución del RTOS (FreeRTOS recomendado)
- Gestión de tareas y prioridades
- Comunicación entre módulos vía buses I2C, SPI, UART
- Watchdog timer para recuperación de fallos
- Gestión de interrupciones

### Interfaces
- **I2C**: Sensores (IMU, barómetro, monitor de batería)
- **SPI**: Memoria flash, radio telemetría
- **UART**: GPS, debug terminal, telemetría
- **GPIO**: Control de servos (PWM), pyro channels, LEDs
- **ADC**: Lecturas analógicas (sensores opcionales)

### Simulación en Proteus
- Virtual Terminal para debug output
- I2C/SPI debugger para verificar comunicaciones
- Osciloscopio en pines PWM
- Timing analysis para verificar determinismo del RTOS

---

## MÓDULO 03: SENSORS (Interfaz General de Sensores)

### Descripción
Módulo de abstracción y multiplexación para múltiples sensores I2C/SPI.

### Componentes Principales
- **Multiplexor I2C**: TCA9548A (si hay conflicto de direcciones)
- **Pull-ups I2C**: 4.7kΩ en SDA y SCL
- **Conectores**: Para cada sensor
- **LEDs de estado**: Indicador de actividad por sensor

### Funcionalidades
- Gestión centralizada de bus I2C
- Detección automática de sensores conectados
- Timeouts y manejo de errores de comunicación
- Calibración y offset de sensores

### Interfaces
- **I2C Master**: Hacia MCU
- **I2C Slaves**: Hacia cada sensor individual
- **GPIO**: Interrupciones de sensores (data ready)

### Simulación en Proteus
- I2C debugger para verificar direcciones y tráfico
- Generadores de señal para simular respuestas de sensores
- Logs de detección automática de dispositivos

---

## MÓDULO 04: IMU NAVIGATION (Unidad de Medición Inercial)

### Descripción
Sistema de navegación inercial con acelerómetro, giroscopio y magnetómetro.

### Componentes Principales
- **IMU**: MPU6050, MPU9250, BNO055 o LSM9DS1
- **Filtros**: Kalman o Madgwick implementado en software
- **Calibración**: EEPROM para almacenar offsets

### Funcionalidades
- Lectura de aceleración en 3 ejes (detección de lanzamiento)
- Lectura de velocidad angular (orientación)
- Cálculo de ángulos de Euler (pitch, roll, yaw)
- Fusión de sensores para estimación de actitud
- Detección de eventos (despegue, apogeo, aterrizaje)

### Datos de Salida
- Aceleración X, Y, Z (m/s²)
- Velocidad angular X, Y, Z (°/s)
- Ángulos de Euler (°)
- Cuaterniones (para evitar gimbal lock)

### Simulación en Proteus
- Potenciómetros para simular cambios de orientación
- Generador de señales para simular perfil de aceleración de vuelo
- Virtual Terminal mostrando ángulos en tiempo real
- Graph mode para verificar filtrado de señales

---

## MÓDULO 05: BAROMETER ALTIMETER (Barómetro/Altímetro)

### Descripción
Medición de presión atmosférica y cálculo de altitud.

### Componentes Principales
- **Sensor barométrico**: BMP280, BMP388, MS5611
- **Conversión**: Fórmula barométrica para altitud
- **Filtro**: Media móvil para suavizar lecturas

### Funcionalidades
- Medición de presión absoluta
- Cálculo de altitud sobre nivel del mar
- Calibración en tierra (presión de referencia)
- Detección de apogeo (cambio de tendencia)
- Tasa de descenso/ascenso (vertical velocity)

### Datos de Salida
- Presión (hPa)
- Altitud MSL (metros)
- Velocidad vertical (m/s)

### Simulación en Proteus
- Potenciómetro para simular cambio de presión (subida/bajada)
- Función rampa para perfil de vuelo realista
- Logs de detección de apogeo
- Gráfica altitud vs tiempo

---

## MÓDULO 06: GNSS GPS (Sistema de Posicionamiento Global)

### Descripción
Receptor GPS para posición geográfica y recuperación post-vuelo.

### Componentes Principales
- **Módulo GPS**: NEO-6M, NEO-M8N, SAM-M8Q
- **Antena**: Cerámica o patch
- **Interfaz**: UART

### Funcionalidades
- Lectura de coordenadas (latitud, longitud)
- Altitud GPS (menos precisa que barómetro)
- Velocidad sobre suelo (groundspeed)
- Número de satélites y HDOP (calidad de señal)
- Parsing de tramas NMEA (GGA, RMC)

### Datos de Salida
- Latitud/Longitud (decimal degrees)
- Altitud GPS (metros)
- Velocidad (km/h)
- Fix status (sin señal, 2D, 3D)

### Simulación en Proteus
- **Limitación**: Proteus no simula GPS directamente
- **Solución**: Usar Arduino/MCU auxiliar que envíe tramas NMEA simuladas por UART
- Virtual Terminal para ver tramas NMEA parseadas
- Logs de adquisición de satélites

---

## MÓDULO 07: TELEMETRY RADIO (Radio Telemetría)

### Descripción
Transmisión inalámbrica de datos en tiempo real a estación terrestre.

### Componentes Principales
- **Radio módulo**: LoRa (SX1276/RFM95), NRF24L01+, o HC-12
- **Antena**: Dipolo sintonizada a frecuencia
- **Interfaz**: SPI (LoRa) o UART (HC-12)

### Funcionalidades
- Envío de telemetría cada N milisegundos
- Protocolo de paquetes con checksum
- Confirmación de recepción (ACK)
- Potencia de transmisión ajustable
- Cifrado básico (opcional)

### Datos Transmitidos
- Timestamp
- Estado del cohete (pad, boost, coast, descent, landed)
- Altitud, velocidad vertical
- Orientación (ángulos)
- Voltaje de batería
- GPS fix y coordenadas

### Simulación en Proteus
- **Limitación**: Proteus no simula RF
- **Solución**: Usar Virtual Terminal o UART para verificar paquetes
- SPI debugger para verificar comandos al módulo LoRa
- Logs de paquetes enviados con timestamps

---

## MÓDULO 08: DATA LOGGING (Registro de Datos)

### Descripción
Almacenamiento en memoria no volátil de todos los datos de vuelo.

### Componentes Principales
- **Memoria Flash**: Tarjeta microSD con módulo SPI
- **Alternativa**: EEPROM I2C o Flash SPI (W25Q128)
- **LED indicador**: Escritura activa

### Funcionalidades
- Logging de alta frecuencia (100Hz o más)
- Formato CSV o binario
- Buffer circular en RAM para no perder datos
- Timestamps precisos
- Marcadores de eventos (lanzamiento, apogeo, aterrizaje)

### Datos Almacenados
- Todos los sensores sincronizados por timestamp
- Estados de la máquina de estados
- Eventos críticos
- Errores y warnings

### Simulación en Proteus
- Virtual Terminal mostrando confirmación de escritura
- Contador de bytes escritos
- Simulación de "archivo" mediante UART logging
- Verificación de timestamps

---

## MÓDULO 09: RECOVERY SYSTEM (Sistema de Recuperación)

### Descripción
Control de paracaídas y cargas pirotécnicas (e-matches).

### Componentes Principales
- **Pyro channels**: 2-4 canales con MOSFETs
- **Continuidad**: Detección de e-match conectado
- **Seguridad**: Arming switch físico
- **Protección**: Optoacopladores, flyback diodes
- **Indicadores LED**: Canal activo, continuidad

### Funcionalidades
- Detección de apogeo (barómetro + IMU)
- Ignición de drogue chute en apogeo
- Ignición de main chute a altitud programada (ej. 300m)
- Modo manual (override por telemetría)
- Logs de eventos de recuperación

### Interfaces
- **GPIO**: Control de MOSFETs
- **ADC**: Detección de continuidad
- **Interruptor físico**: Arming (simulado con botón)

### Simulación en Proteus
- Botones para simular switches de arming
- LEDs para ver activación de canales
- Virtual Terminal con logs: `[RECOVERY] Drogue deployed at apogee`
- Relays en lugar de e-matches reales

---

## MÓDULO 10: FLIGHT CONTROL (Control de Vuelo)

### Descripción
Control activo de alerones/canards para estabilización (opcional, avanzado).

### Componentes Principales
- **Servos**: 3-4 servos para superficies de control
- **Driver**: PWM generado por MCU
- **Sensores**: IMU para feedback en bucle cerrado
- **PID Controller**: Implementado en software

### Funcionalidades
- Estabilización activa de pitch y yaw
- Control PID para mantener orientación deseada
- Límites de deflexión de servos
- Modo "coast" (superficies neutrales)
- Logs de comandos enviados vs orientación real

### Simulación en Proteus
- Osciloscopio para verificar señales PWM (50Hz, pulsos 1000-2000μs)
- Potenciómetros para simular desviaciones de orientación
- Virtual Terminal con logs de correcciones PID
- Graph mode para ver respuesta del controlador

---

## MÓDULO 11: STATE MACHINE (Máquina de Estados)

### Descripción
Lógica de control del ciclo de vida del cohete.

### Estados Principales
1. **PAD_IDLE**: En rampa, sistemas iniciándose
2. **ARMED**: Armado, esperando detección de lanzamiento
3. **BOOST**: Motor encendido, ascenso bajo propulsión
4. **COAST**: Motor apagado, ascenso por inercia
5. **APOGEE**: Punto máximo, despliegue de drogue
6. **DESCENT_DROGUE**: Descenso con drogue chute
7. **MAIN_DEPLOY**: Despliegue de paracaídas principal
8. **DESCENT_MAIN**: Descenso con main chute
9. **LANDED**: Aterrizaje detectado
10. **RECOVERY_MODE**: Señal de localización activa

### Transiciones
- PAD_IDLE → ARMED: Arming switch activado
- ARMED → BOOST: Aceleración > 2g detectada
- BOOST → COAST: Aceleración < 0.5g
- COAST → APOGEE: Velocidad vertical cambia de signo
- APOGEE → DESCENT_DROGUE: Inmediato tras apogeo
- DESCENT_DROGUE → MAIN_DEPLOY: Altitud < threshold
- DESCENT_MAIN → LANDED: Velocidad vertical ≈ 0 por >3s

### Simulación en Proteus
- LEDs para cada estado
- Virtual Terminal con logs de transiciones
- Botones para forzar eventos (testing)
- Diagrama de estados visualizable

---

## MÓDULO 12: DIAGNOSTICS & DEBUG (Diagnósticos)

### Descripción
Sistema de logging, debugging y health monitoring.

### Componentes Principales
- **UART Debug**: Terminal serie para logs en tiempo real
- **LEDs de estado**: RGB para estados globales
- **Buzzer**: Alarmas audibles (simulado)
- **Watchdog**: Reset automático ante bloqueos

### Funcionalidades
- Logs categorizados: INFO, WARN, ERROR, CRITICAL
- Timestamps en todos los logs
- Health checks periódicos:
  - Sensores respondiendo
  - Voltaje de batería OK
  - Memoria flash no llena
  - GPS fix adquirido
- Códigos de error numerados
- Stack overflow detection (si usa RTOS)

### Niveles de Log
```
[INFO] System initialized
[WARN] GPS fix lost
[ERROR] IMU communication timeout
[CRITICAL] Battery voltage critical: 6.2V
```

### Simulación en Proteus
- Virtual Terminal principal para todos los logs
- LEDs RGB mostrando estado de salud
- Botones para inyectar fallos (testing de recovery)
- Contador de errores en pantalla

---

## INTEGRATION (Integración del Sistema Completo)

### Diagrama de Bloques
```
                    ┌─────────────────────┐
                    │   POWER MANAGEMENT  │
                    │   (Batería + Regs)  │
                    └──────────┬──────────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
      ┌───▼───┐          ┌────▼────┐          ┌────▼────┐
      │  5V   │          │  3.3V   │          │  I2C    │
      │ Servos│          │   MCU   │          │ Battery │
      └───────┘          │  CORE   │          │ Monitor │
                         └────┬────┘          └─────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │         │           │           │         │
    ┌───▼───┐ ┌──▼──┐   ┌────▼────┐  ┌──▼──┐  ┌───▼────┐
    │  IMU  │ │ BARO│   │   GPS   │  │LoRa │  │ FLASH  │
    │(I2C)  │ │(I2C)│   │ (UART)  │  │(SPI)│  │  (SPI) │
    └───────┘ └─────┘   └─────────┘  └─────┘  └────────┘
        │         │           │           │         │
        └─────────┴───────────┴───────────┴─────────┘
                              │
                    ┌─────────▼─────────┐
                    │   STATE MACHINE   │
                    │   & FLIGHT CTRL   │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  RECOVERY SYSTEM  │
                    │  (Pyro Channels)  │
                    └───────────────────┘
```

### Buses de Comunicación
- **I2C**: IMU, Barómetro, Battery Monitor, Multiplexor
- **SPI**: LoRa Radio, Flash Memory
- **UART1**: GPS
- **UART2**: Debug Terminal
- **PWM**: Servos (si aplica)
- **GPIO**: Pyro channels, LEDs, Buttons

### Secuencia de Inicialización
1. Power-on reset
2. MCU init (clocks, peripherals)
3. RTOS kernel start
4. Sensor detection & calibration
5. GPS warm-up
6. Flash memory mount
7. Radio init & test
8. System health check
9. Enter PAD_IDLE state
10. Wait for arming

### Tareas del RTOS (Ejemplo con FreeRTOS)
```c
// Prioridades (mayor número = mayor prioridad)
xTaskCreate(vTaskIMU,         "IMU",      256, NULL, 4, NULL);  // 100Hz
xTaskCreate(vTaskBarometer,   "BARO",     256, NULL, 4, NULL);  // 100Hz
xTaskCreate(vTaskGPS,         "GPS",      256, NULL, 3, NULL);  // 10Hz
xTaskCreate(vTaskStateMachine,"FSM",      512, NULL, 5, NULL);  // 50Hz
xTaskCreate(vTaskTelemetry,   "TELEM",    256, NULL, 2, NULL);  // 10Hz
xTaskCreate(vTaskLogging,     "LOG",      512, NULL, 3, NULL);  // 100Hz
xTaskCreate(vTaskRecovery,    "RECOVERY", 256, NULL, 6, NULL);  // Highest
xTaskCreate(vTaskDiagnostics, "DIAG",     256, NULL, 1, NULL);  // 1Hz
```

### Testing en Proteus
1. **Test de inicialización**: Verificar que todos los módulos arrancan
2. **Test de sensores**: Simular lecturas y ver parsing correcto
3. **Test de máquina de estados**: Forzar transiciones con botones
4. **Test de recuperación**: Simular apogeo y ver activación de pyros
5. **Test de batería baja**: Reducir voltaje y ver alarmas
6. **Test de fallos**: Desconectar sensor y ver recovery
7. **Test de telemetría**: Verificar paquetes en UART
8. **Timing analysis**: Verificar que tareas se ejecutan a tiempo

### Métricas a Validar
- **Jitter de tareas críticas**: < 1ms
- **Latencia de detección de apogeo**: < 50ms
- **Tasa de muestreo IMU**: 100Hz sostenida
- **Uptime del RTOS**: Sin crashes en 10 minutos de simulación
- **Throughput de telemetría**: X paquetes/segundo sin pérdidas

---

## Herramientas y Librerías Recomendadas

### Software
- **IDE**: STM32CubeIDE, PlatformIO, Keil
- **RTOS**: FreeRTOS
- **Proteus**: Versión 8.9 o superior (con VSM models)
- **Control de versiones**: Git + GitHub
- **Documentación**: Markdown, Doxygen

### Librerías
- **STM32 HAL**: Drivers de periféricos
- **FreeRTOS**: Kernel tiempo real
- **TinyGPS++**: Parsing NMEA
- **Adafruit_Sensor**: Abstracción de sensores
- **LoRa library**: Para SX1276/RFM95
- **FatFs**: Sistema de archivos para SD
- **MadgwickAHRS**: Fusión de sensores IMU

---

## Checklist de Completitud del Proyecto

### Hardware (Schematics en Proteus)
- [ ] Esquemático de cada módulo individual
- [ ] Esquemático integrado completo
- [ ] Bill of Materials (BOM) generado
- [ ] Design Rule Check (DRC) pasado
- [ ] Componentes con VSM models verificados

### Firmware
- [ ] Código de cada módulo compilando sin errores
- [ ] Drivers de sensores testeados individualmente
- [ ] RTOS configurado y tareas definidas
- [ ] Máquina de estados implementada
- [ ] Sistema de logging funcional
- [ ] Protocolo de telemetría definido

### Simulación
- [ ] Cada módulo simulado independientemente
- [ ] Sistema integrado simulando correctamente
- [ ] Perfil de vuelo completo simulado
- [ ] Casos de fallo testeados
- [ ] Timing analysis completado
- [ ] Video/capturas de pantalla para portfolio

### Documentación
- [ ] README principal del proyecto
- [ ] README de cada módulo
- [ ] Diagramas de flujo y estados
- [ ] Guía de simulación en Proteus
- [ ] Resultados de tests documentados

### PCB (Fase posterior)
- [ ] Layout del PCB diseñado
- [ ] Routing completado
- [ ] DRC y ERC pasados
- [ ] Archivos Gerber generados
- [ ] 3D preview revisado

---

## Próximos Pasos Sugeridos

1. **Fase 1 - Setup**:
   - Crear estructura de carpetas
   - Configurar Git repository
   - Instalar Proteus y verificar librerías STM32

2. **Fase 2 - Desarrollo Modular**:
   - Empezar por Power Management (base de todo)
   - Seguir con MCU Core y debug UART
   - Agregar sensores uno por uno
   - Implementar State Machine
   - Integrar Recovery System

3. **Fase 3 - Integración**:
   - Unir todos los módulos en esquemático completo
   - Configurar RTOS con todas las tareas
   - Testear comunicación inter-módulos

4. **Fase 4 - Simulación**:
   - Crear perfil de vuelo realista
   - Ejecutar simulación completa
   - Grabar video para portfolio
   - Documentar resultados

5. **Fase 5 - PCB** (si la simulación es exitosa):
   - Diseñar layout considerando EMI/EMC
   - Revisar con DRC
   - Generar archivos de fabricación

---

## Contacto y Referencias

### Para Portfolio Clue Aerospace
Este proyecto demuestra:
- ✅ **Embedded Systems**: MCU, periféricos, buses I2C/SPI
- ✅ **RTOS**: FreeRTOS, scheduling, prioridades
- ✅ **Sensores**: Fusión IMU, filtrado Kalman
- ✅ **Control**: PID, state machines
- ✅ **Telemetría**: Protocolos, radio comms
- ✅ **Metodología**: Modular, versionado, simulado antes de HW

### Referencias Útiles
- Documentación STM32: https://www.st.com/
- FreeRTOS: https://www.freertos.org/
- Proteus VSM: https://www.labcenter.com/
- OpenRocket (para perfiles de vuelo): https://openrocket.info/

---

**Autor**: Tu nombre  
**Fecha**: 2026  
**Licencia**: MIT (o la que prefieras)  
**Estado**: En desarrollo - Simulación virtual

---

Este documento es la base. Copia cada sección en un README.md dentro de su carpeta correspondiente y personalízalo con detalles técnicos específicos. ¡Éxito con tu proyecto!