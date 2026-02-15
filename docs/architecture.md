# Arquitectura del Sistema - Rocket Avionics

## Visión General

El sistema de aviónica se estructura como una arquitectura centralizada con MCU único (STM32F401VE) ejecutando FreeRTOS. Todos los subsistemas se conectan al MCU mediante buses estándar (I2C, SPI, UART) y se abstraen en tareas RTOS independientes.

---

## Decisiones Arquitectónicas

### ¿Por qué MCU único vs. múltiples MCUs?
- **Elegido**: MCU único STM32F401VE
- **Razón**: Simplifica la comunicación inter-módulos, reduce peso, elimina problemas de sincronización entre MCUs
- **Trade-off**: Mayor complejidad del firmware, punto único de fallo (mitigado con watchdog)
- **Alternativa descartada**: Arquitectura distribuida con CAN bus - excesiva para cohete amateur

### ¿Por qué STM32F401VE?
- ARM Cortex-M4 a 84 MHz - potencia suficiente para fusión de sensores + PID a 100Hz
- FPU hardware - cálculos de filtro Madgwick/Kalman sin penalización
- Periféricos suficientes: 3x I2C, 4x SPI, 3x USART, 12x ADC, 12x PWM
- **Modelo VSM disponible en Proteus** (STM32F401VE) - requisito crítico para simulación
- Variante VE: LQFP100, 512 KB Flash, 96 KB SRAM
- Ecosistema maduro: HAL, CubeMX, FreeRTOS port oficial

### ¿Por qué FreeRTOS y no bare-metal?
- Determinismo temporal garantizado para tareas críticas (recovery a máxima prioridad)
- Separación limpia de responsabilidades por tarea
- Mecanismos de sincronización (semáforos, colas, mutexes) para datos compartidos
- Stack overflow detection para robustez
- Port oficial para STM32F4 con soporte completo

---

## Mapa de Periféricos del MCU

```
STM32F401VET6
├── I2C1 (PB6=SCL, PB7=SDA) ── 400kHz
│   ├── MPU6050  (addr: 0x68)  ── IMU
│   ├── BMP280   (addr: 0x76)  ── Barómetro
│   └── INA219   (addr: 0x40)  ── Monitor de batería
│
├── SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI)
│   ├── CS=PA4  ── W25Q128 Flash Memory
│   └── CS=PB0  ── RFM95W LoRa Radio
│
├── UART1 (PA9=TX, PA10=RX) ── 9600 baud
│   └── NEO-6M GPS
│
├── UART2 (PA2=TX, PA3=RX) ── 115200 baud
│   └── Debug Terminal / Diagnostics
│
├── TIM2 (PWM)
│   ├── CH1=PA0  ── Servo 1
│   ├── CH2=PA1  ── Servo 2
│   ├── CH3=PB10 ── Servo 3
│   └── CH4=PB11 ── Servo 4
│
├── GPIO (Output)
│   ├── PC0  ── PYRO_DROGUE (MOSFET gate)
│   ├── PC1  ── PYRO_MAIN (MOSFET gate)
│   ├── PC2  ── LED_HEARTBEAT
│   ├── PC3  ── LED_STATUS_R
│   ├── PC4  ── LED_STATUS_G
│   ├── PC5  ── LED_STATUS_B
│   └── PC6  ── BUZZER
│
├── GPIO (Input)
│   ├── PD0  ── ARM_SWITCH
│   ├── PD1  ── PYRO_CONTINUITY_1 (ADC alternativo)
│   └── PD2  ── PYRO_CONTINUITY_2 (ADC alternativo)
│
├── ADC1
│   ├── CH0=PA0  ── (compartido con servo, usar si no hay flight control)
│   ├── IN10=PC0 ── Continuidad Pyro 1 (si se usa ADC)
│   └── IN11=PC1 ── Continuidad Pyro 2 (si se usa ADC)
│
└── IWDG (Independent Watchdog)
    └── Timeout: 1 segundo
```

---

## Modelo de Tareas RTOS

| Tarea              | Prioridad | Frecuencia | Stack   | Descripción                          |
|--------------------|-----------|------------|---------|--------------------------------------|
| `vTaskRecovery`    | 6 (MAX)   | Evento     | 256 W   | Despliegue de paracaídas             |
| `vTaskStateMachine`| 5         | 50 Hz      | 512 W   | FSM del ciclo de vuelo               |
| `vTaskIMU`         | 4         | 100 Hz     | 256 W   | Lectura + filtrado MPU6050           |
| `vTaskBarometer`   | 4         | 100 Hz     | 256 W   | Lectura BMP280 + altitud             |
| `vTaskGPS`         | 3         | 10 Hz      | 256 W   | Parsing NMEA del NEO-6M              |
| `vTaskLogging`     | 3         | 100 Hz     | 512 W   | Escritura en Flash W25Q128           |
| `vTaskTelemetry`   | 2         | 10 Hz      | 256 W   | Envío de paquetes LoRa               |
| `vTaskDiagnostics` | 1 (LOW)   | 1 Hz       | 256 W   | Health checks y logging              |

### Sincronización entre Tareas

```
IMU Task ──┐
           ├──▶ [Queue: sensor_data_q] ──▶ State Machine Task
BARO Task ─┘                                      │
                                                   ├──▶ [Event: deploy_event] ──▶ Recovery Task
GPS Task ──▶ [Queue: gps_data_q] ──────────────────┤
                                                   ├──▶ [Queue: telem_q] ──▶ Telemetry Task
                                                   └──▶ [Queue: log_q] ──▶ Logging Task

[Mutex: i2c_mutex] ── Protege acceso al bus I2C compartido
[Mutex: spi_mutex] ── Protege acceso al bus SPI compartido
```

---

## Flujo de Datos

### Ciclo Principal (cada 10ms = 100Hz)

```
1. IMU Task lee MPU6050 via I2C
   → Aplica filtro Madgwick
   → Publica en sensor_data_q: {accel_x/y/z, gyro_x/y/z, pitch, roll, yaw}

2. BARO Task lee BMP280 via I2C
   → Calcula altitud con fórmula barométrica
   → Calcula velocidad vertical (derivada)
   → Publica en sensor_data_q: {pressure, altitude, vspeed}

3. State Machine Task consume sensor_data_q
   → Evalúa condiciones de transición
   → Si transición: actualiza estado, genera eventos
   → Publica datos para telemetría y logging

4. Recovery Task espera deploy_event
   → Activa MOSFET correspondiente (drogue/main)
   → Log del evento

5. GPS Task (cada 100ms) parsea tramas NMEA
   → Publica en gps_data_q: {lat, lon, alt_gps, speed, fix, sats}

6. Logging Task escribe todos los datos en Flash SPI
   → Formato binario con timestamp

7. Telemetry Task empaqueta datos y envía por LoRa
   → Incluye estado FSM, altitud, GPS, batería

8. Diagnostics Task (cada 1s)
   → Verifica respuesta de sensores
   → Lee voltaje de batería via INA219
   → Comprueba espacio en Flash
   → Reporta por UART debug
```

---

## Gestión de Energía

### Árbol de Alimentación

```
LiPo 3S (11.1V nominal, 9.0-12.6V)
    │
    ├──▶ Buck Converter 5V (2A)
    │       ├── Servos (×4, hasta 500mA cada uno en carga)
    │       └── Reserva para actuadores
    │
    └──▶ LDO 3.3V (500mA)
            ├── STM32F401 (~50mA)
            ├── MPU6050 (~3.6mA)
            ├── BMP280 (~0.7mA)
            ├── NEO-6M (~45mA)
            ├── RFM95W (~120mA TX)
            ├── W25Q128 (~15mA write)
            ├── INA219 (~1mA)
            └── LEDs, pull-ups (~20mA)
            ─────────────────────
            Total 3.3V: ~306mA (margen OK)
```

### Consumo Estimado por Fase

| Fase               | Corriente 3.3V | Corriente 5V | Total    |
|--------------------|-----------------|---------------|----------|
| PAD_IDLE           | ~150 mA         | ~50 mA        | ~200 mA  |
| ARMED              | ~200 mA         | ~50 mA        | ~250 mA  |
| BOOST/COAST        | ~250 mA         | ~500 mA       | ~750 mA  |
| DESCENT            | ~230 mA         | ~100 mA       | ~330 mA  |
| LANDED/RECOVERY    | ~200 mA         | ~50 mA        | ~250 mA  |

---

## Estrategia de Manejo de Errores

### Niveles de Severidad

| Nivel      | Acción                                             | Ejemplo                          |
|------------|----------------------------------------------------|----------------------------------|
| INFO       | Log only                                           | GPS fix adquirido                |
| WARNING    | Log + LED amarillo                                 | GPS fix perdido temporalmente    |
| ERROR      | Log + intento de recovery                          | IMU no responde → reiniciar I2C  |
| CRITICAL   | Log + acción de seguridad                          | Batería crítica → deploy main    |

### Política de Fallos por Sensor

| Sensor     | Si falla...                                              |
|------------|----------------------------------------------------------|
| IMU        | Usar solo barómetro para detección de apogeo             |
| Barómetro  | Usar solo IMU (integración de aceleración) para altitud  |
| GPS        | No afecta al vuelo, solo a recovery post-landing         |
| Flash      | Cambiar a logging solo por telemetría                    |
| Radio      | Vuelo continúa, datos solo en Flash                      |
| Batería    | Sin monitoreo, asumir voltaje OK                         |

---

## Diagrama de Estados Completo

```
                        ┌─────────┐
                 ┌──────│PAD_IDLE │
                 │      └────┬────┘
                 │           │ ARM_SWITCH = ON
            DISARM           │
                 │      ┌────▼────┐
                 └──────│  ARMED  │
                        └────┬────┘
                             │ accel_z > 2g for 100ms
                        ┌────▼────┐
                        │  BOOST  │
                        └────┬────┘
                             │ accel_z < 0.5g
                        ┌────▼────┐
                        │  COAST  │
                        └────┬────┘
                             │ vspeed changes sign (+ → -)
                        ┌────▼────┐
               ┌────────│ APOGEE  │
               │        └────┬────┘
               │             │ IMMEDIATE: fire drogue
               │        ┌────▼──────────┐
               │        │DESCENT_DROGUE │
               │        └────┬──────────┘
               │             │ altitude < MAIN_ALT (300m)
           TIMEOUT      ┌────▼──────────┐
           BACKUP        │ MAIN_DEPLOY  │
               │        └────┬──────────┘
               │             │ fire main chute
               │        ┌────▼────────┐
               │        │DESCENT_MAIN │
               │        └────┬────────┘
               │             │ vspeed ≈ 0 for 3s
               │        ┌────▼────┐
               └───────▶│ LANDED  │
                        └────┬────┘
                             │ AUTO after 10s
                        ┌────▼──────────┐
                        │RECOVERY_MODE  │
                        │(buzzer+GPS TX)│
                        └───────────────┘
```

---

## Consideraciones de Timing

### Presupuesto de Tiempo por Ciclo (10ms)

```
Lectura I2C IMU:        ~1.5 ms (14 bytes @ 400kHz + overhead)
Filtro Madgwick:        ~0.6 ms (con FPU a 84 MHz)
Lectura I2C BMP280:     ~1.0 ms (6 bytes @ 400kHz)
Cálculo de altitud:     ~0.2 ms
State Machine eval:     ~0.4 ms
Flash SPI write:        ~0.5 ms (page write, non-blocking con DMA ideal)
LoRa packet TX:         ~5.0 ms (pero solo cada 100ms)
Context switch RTOS:    ~0.02 ms per switch
───────────────────────────────────────
Total (sin LoRa):       ~4.2 ms → Margen: 5.8 ms (58% libre)
```

---

*Documento vivo - se actualiza con cada módulo completado*
