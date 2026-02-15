# Integración del Sistema Completo

## Visión General

Este directorio contiene los recursos de integración donde todos los 12 módulos convergen en un sistema unificado. Aquí se valida que los subsistemas cooperan correctamente: los buses comparten el medio sin conflictos, las tareas RTOS cumplen sus deadlines, y los datos fluyen coherentemente desde los sensores hasta la telemetría y el almacenamiento.

```
integration/
├── full-system-schematic/    ← Esquemático Proteus con TODOS los componentes
├── full-system-simulation/   ← Simulación end-to-end con datos de vuelo reales
├── pcb-design/               ← Diseño PCB (futuro)
└── tests/                    ← Tests de integración automatizados
```

---

## 1. Esquemático Completo del Sistema

### 1.1 Arquitectura de Conexiones

```
                    ┌─────────────────────────────┐
                    │       STM32F401VET6          │
                    │                               │
    I2C1 (400kHz)   │  PB6 (SCL) ──────┬──────────│──→ MPU6050 (0x68)
                    │  PB7 (SDA) ──────┤          │──→ BMP280  (0x76)
                    │                   └──────────│──→ INA219  (0x40)
                    │                               │
    SPI1 (8MHz)     │  PA5 (SCK)  ─────────────────│──→ W25Q128 (CS: PA4)
                    │  PA6 (MISO) ─────────────────│──→ RFM95W  (CS: PB0)
                    │  PA7 (MOSI) ─────────────────│
                    │                               │
    UART1 (9600)    │  PA9  (TX) ──────────────────│──→ NEO-6M GPS
                    │  PA10 (RX) ──────────────────│
                    │                               │
    UART2 (115200)  │  PA2 (TX) ───────────────────│──→ Debug Console
                    │  PA3 (RX) ───────────────────│
                    │                               │
    GPIO            │  PC0 ────────────────────────│──→ Drogue MOSFET (IRLZ44N)
                    │  PC1 ────────────────────────│──→ Main MOSFET (IRLZ44N)
                    │  PC2 ────────────────────────│──→ Arm Switch Input
                    │  PC3 ────────────────────────│──→ Continuity Drogue
                    │  PC4 ────────────────────────│──→ Continuity Main
                    │  PC13 ───────────────────────│──→ Heartbeat LED
                    │                               │
    PWM (TIM2)      │  PA0 ────────────────────────│──→ Servo Canard 1
                    │  PA1 ────────────────────────│──→ Servo Canard 2
                    │                               │
    Interrupts      │  PB1 (EXTI1) ────────────────│──→ RFM95W DIO0
                    │  PB2 (EXTI2) ────────────────│──→ MPU6050 INT
                    │                               │
    Buzzer          │  PB10 (TIM2_CH3) ────────────│──→ Buzzer Piezoeléctrico
                    │                               │
    Power           │  3.3V ───────────────────────│──→ Todos los ICs
                    │  5V   ───────────────────────│──→ Servos, GPS
                    └─────────────────────────────┘
```

### 1.2 Pull-ups y Componentes Pasivos

| Bus/Señal | Componente | Valor | Nota |
|-----------|-----------|-------|------|
| I2C1 SCL | R_SCL | 4.7kΩ | Pull-up a 3.3V |
| I2C1 SDA | R_SDA | 4.7kΩ | Pull-up a 3.3V |
| SPI1 CS (Flash) | R_CS1 | 10kΩ | Pull-up a 3.3V (inactivo HIGH) |
| SPI1 CS (Radio) | R_CS2 | 10kΩ | Pull-up a 3.3V (inactivo HIGH) |
| Arm Switch | R_ARM | 10kΩ | Pull-down (activo = HIGH) |
| Continuity | R_CONT | 1kΩ | Serie en circuito de prueba |
| Reset | C_RST | 100nF | Filtro en NRST |
| Cada IC | C_BYPASS | 100nF | Desacoplo por cada IC |
| VDD | C_BULK | 10µF | Capacitor bulk en rail de 3.3V |

### 1.3 Construcción del Esquemático en Proteus

**Paso 1: Crear Proyecto**
```
File → New Project → "RocketAvionics_FullSystem"
├── Schematic: Default
├── PCB Layout: None (por ahora)
└── Firmware: STM32F401VET6
```

**Paso 2: Colocar Componentes por Subsistema**
Organizar el esquemático en bloques visuales:
```
┌─────────────┐ ┌──────────────┐ ┌──────────────┐
│   POWER     │ │   MCU CORE   │ │   SENSORS    │
│ LiPo→Buck   │ │ STM32F401    │ │ I2C Scanner  │
│ →LDO→INA219│ │ HSE 8MHz     │ │ Bus Manager  │
└─────────────┘ └──────────────┘ └──────────────┘

┌─────────────┐ ┌──────────────┐ ┌──────────────┐
│   IMU       │ │  BAROMETER   │ │    GPS       │
│ MPU6050     │ │  BMP280      │ │  NEO-6M      │
│ Madgwick    │ │  Altitude    │ │  NMEA Parse  │
└─────────────┘ └──────────────┘ └──────────────┘

┌─────────────┐ ┌──────────────┐ ┌──────────────┐
│  TELEMETRY  │ │  DATA LOG    │ │  RECOVERY    │
│  RFM95W     │ │  W25Q128     │ │  Dual Deploy │
│  LoRa SPI   │ │  Flash SPI   │ │  MOSFETs     │
└─────────────┘ └──────────────┘ └──────────────┘
```

**Paso 3: Conectar Buses**
- Usar Net Labels para buses compartidos (I2C1_SCL, I2C1_SDA, SPI1_SCK, etc.)
- NO tirar cables largos entre bloques - usar labels
- Cada bloque debe ser autocontenido visualmente

---

## 2. Simulación Full-System

### 2.1 Pipeline de Simulación End-to-End

```
OpenRocket CSV ──→ Python Converter ──→ Proteus Stimulus Files
                                        ├── mpu6050_stimulus.sdf
                                        ├── bmp280_stimulus.sdf
                                        ├── gps_nmea_stream.txt
                                        └── power_profile.sdf
                                              │
                                              ▼
                                    Proteus Full-System Sim
                                              │
                    ┌─────────────────────────┼────────────────────┐
                    ▼                         ▼                    ▼
              Telemetry Output         Flash Log Dump        Debug Console
              (Virtual Terminal)       (Memory View)         (UART Output)
```

### 2.2 Escenarios de Simulación Integrada

#### Escenario A: Vuelo Nominal Completo
```
Tiempo  | Fase          | Eventos Esperados
--------|---------------|------------------------------------------
T-5s    | PAD_IDLE      | Health check OK, sensores calibrados
T+0s    | ARMED         | Switch activado, continuidad verificada
T+0.1s  | BOOST         | Aceleración >3g, PID activo, logging 100Hz
T+3.5s  | COAST         | Aceleración <0.5g, PID sigue activo
T+12s   | APOGEE        | Vel vertical ≈ 0, presión mínima
T+12.1s | DROGUE_DEPLOY | MOSFET drogue activo 2s, vel descendente
T+45s   | MAIN_DEPLOY   | Altitud <150m, MOSFET main activo 2s
T+120s  | RECOVERY_MODE | Beacon GPS cada 5s, buzzer activo
```

#### Escenario B: Fallo de Sensor IMU
```
T+5s: MPU6050 deja de responder (simular desconexión I2C)
Esperado:
  - health_check() detecta fallo IMU
  - LOG_WARNING: "IMU communication lost"
  - Navegación degrada a solo barométrica
  - Recovery system usa backup timer (15s desde launch)
  - Telemetría reporta sensor_status con flag IMU_FAIL
```

#### Escenario C: Pérdida de Telemetría
```
T+8s: RFM95W no responde a comandos SPI
Esperado:
  - health_check() detecta fallo radio
  - LOG_WARNING: "Radio offline"
  - Data logging continúa normalmente
  - Recovery y FSM operan independientemente
  - Post-vuelo: datos recuperables de flash
```

#### Escenario D: Fallo de Paracaídas Drogue
```
T+12s: Apogeo detectado, continuity check falla para drogue
Esperado:
  - DROGUE_DEPLOY intenta disparo → no detecta despliegue
  - LOG_CRITICAL: "Drogue deploy failure"
  - Backup timer activo (15s desde launch detect)
  - Si sigue fallando → emergency_deploy() activa ambos canales
  - Telemetría envía alerta crítica
```

### 2.3 Verificación de Coherencia de Datos

La simulación integrada debe validar que todos los datos son físicamente coherentes:

```c
// Tabla de correlación en cada instante t:
//
// altitude_baro(t) ≈ altitude_gps(t)     (±10m)
// velocity_imu(t)  ≈ d(altitude_baro)/dt (±2 m/s)
// accel_imu(t)     ≈ d(velocity)/dt      (±0.5 g)
//
// En apogeo (t_apogee):
//   velocity_vertical ≈ 0
//   pressure = mínima del vuelo
//   altitude = máxima del vuelo
//   accel_z ≈ -1g (solo gravedad)
//
// En despliegue drogue (t_apogee + delta):
//   altitude decreasing
//   velocity < 0 (descendiendo)
//   accel_z spike positivo (frenado)
```

### 2.4 Archivos de Simulación

```
full-system-simulation/
├── RocketAvionics_FullSystem.pdsprj   ← Proyecto Proteus principal
├── stimulus/
│   ├── flight_nominal.zip             ← Todos los stimulus para vuelo nominal
│   ├── flight_failure_imu.zip         ← Stimulus con fallo IMU a T+5s
│   └── flight_failure_chute.zip       ← Stimulus con fallo paracaídas
├── firmware/
│   └── rocket_avionics.hex            ← Firmware compilado para simulación
├── results/
│   ├── nominal_telemetry.log          ← Log de telemetría capturado
│   ├── nominal_flash_dump.bin         ← Dump de flash post-vuelo
│   └── nominal_debug_console.log      ← Salida UART de debug
└── scripts/
    └── validate_simulation.py         ← Script que verifica coherencia
```

---

## 3. Diseño PCB (Futuro)

### 3.1 Consideraciones de Diseño

```
Factor de Forma: Cilíndrico (diámetro interno del cohete)
├── Diámetro útil: ~35mm (cohete de 38mm)
├── Formato: PCBs circulares apiladas con conectores
└── Capas: 4 capas mínimo (señal, GND, power, señal)

Stack-up propuesto:
┌──────────────┐
│  Top Signal   │  MCU, conectores, LEDs
├──────────────┤
│  GND Plane    │  Plano de tierra continuo
├──────────────┤
│  Power Plane  │  3.3V, 5V con splits
├──────────────┤
│  Bottom Signal│  Sensores, radio, flash
└──────────────┘
```

### 3.2 Reglas de Ruteo

| Señal | Ancho Pista | Separación | Impedancia |
|-------|------------|------------|------------|
| Power (3.3V, 5V) | 0.5mm | 0.2mm | N/A |
| I2C | 0.2mm | 0.15mm | N/A |
| SPI | 0.2mm | 0.15mm | ~50Ω |
| RF (LoRa) | 0.2mm | 0.3mm | 50Ω controlada |
| GPIO | 0.15mm | 0.15mm | N/A |

### 3.3 Consideraciones EMC

- Plano de tierra continuo bajo MCU y radio
- Bypass capacitors (100nF) lo más cerca posible de cada IC
- Separación analógico/digital
- Antena LoRa en borde de PCB, lejos de líneas digitales
- Guard ring alrededor de cristal HSE

---

## 4. Tests de Integración

### 4.1 Estructura de Tests

```
tests/
├── test_bus_arbitration.c      ← Verifica que I2C y SPI no colisionan
├── test_task_timing.c          ← Mide tiempos reales de cada tarea RTOS
├── test_data_flow.c            ← Datos fluyen sensor→log→telemetry
├── test_state_transitions.c   ← FSM recorre todos los estados
├── test_recovery_sequence.c   ← Secuencia completa de recuperación
├── test_power_modes.c          ← Transiciones de consumo
└── test_full_flight.c          ← Simulación de vuelo completo
```

### 4.2 Test: Arbitraje de Bus I2C

```c
/**
 * @file test_bus_arbitration.c
 * @brief Verifica que 3 dispositivos I2C coexisten sin conflictos
 * 
 * Procedimiento:
 * 1. Inicializar I2C1 con MPU6050, BMP280, INA219
 * 2. Crear 3 tareas que leen sus sensores continuamente
 * 3. Ejecutar durante 10 segundos
 * 4. Verificar: 0 errores NACK, 0 timeouts, 0 datos corruptos
 */

#include "unity.h"
#include "sensors_hal.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t imu_reads = 0;
static volatile uint32_t baro_reads = 0;
static volatile uint32_t power_reads = 0;
static volatile uint32_t errors = 0;

void task_read_imu(void* params) {
    imu_data_t data;
    while (1) {
        if (mpu6050_read_all(&data) == HAL_OK) {
            imu_reads++;
            // Verificar rango válido
            TEST_ASSERT_FLOAT_WITHIN(16.0f, 0.0f, data.accel_x);
            TEST_ASSERT_FLOAT_WITHIN(16.0f, 0.0f, data.accel_y);
            TEST_ASSERT_FLOAT_WITHIN(16.0f, 0.0f, data.accel_z);
        } else {
            errors++;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

void task_read_baro(void* params) {
    baro_data_t data;
    while (1) {
        if (bmp280_read(&data) == HAL_OK) {
            baro_reads++;
            TEST_ASSERT_FLOAT_WITHIN(200.0f, 1013.25f, data.pressure_hpa);
            TEST_ASSERT_FLOAT_WITHIN(100.0f, 25.0f, data.temperature_c);
        } else {
            errors++;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

void task_read_power(void* params) {
    power_data_t data;
    while (1) {
        if (ina219_read(&data) == HAL_OK) {
            power_reads++;
            TEST_ASSERT_FLOAT_WITHIN(15.0f, 11.0f, data.voltage_v);
            TEST_ASSERT_TRUE(data.current_ma >= 0);
        } else {
            errors++;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

void test_bus_arbitration_no_conflicts(void) {
    // Crear tareas
    xTaskCreate(task_read_imu,   "IMU",   256, NULL, 3, NULL);
    xTaskCreate(task_read_baro,  "BARO",  256, NULL, 3, NULL);
    xTaskCreate(task_read_power, "PWR",   256, NULL, 2, NULL);
    
    // Ejecutar 10 segundos
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    // Verificar resultados
    printf("IMU reads:   %lu\n", imu_reads);
    printf("Baro reads:  %lu\n", baro_reads);
    printf("Power reads: %lu\n", power_reads);
    printf("Errors:      %lu\n", errors);
    
    TEST_ASSERT_EQUAL_UINT32(0, errors);
    TEST_ASSERT_TRUE(imu_reads > 900);   // ~1000 esperados
    TEST_ASSERT_TRUE(baro_reads > 450);  // ~500 esperados
    TEST_ASSERT_TRUE(power_reads > 90);  // ~100 esperados
}
```

### 4.3 Test: Timing de Tareas RTOS

```c
/**
 * @file test_task_timing.c
 * @brief Mide el tiempo real de ejecución de cada tarea
 * 
 * Criterio de aceptación:
 * - Suma de worst-case < 10ms (período del tick)
 * - Ninguna tarea excede su budget individual
 * 
 * Budget por tarea (de architecture.md):
 *   IMU:       1.2ms
 *   Baro:      0.8ms
 *   GPS:       0.3ms
 *   Logging:   0.5ms
 *   Telemetry: 0.4ms
 *   FSM:       0.2ms
 *   Recovery:  0.1ms
 *   Diag:      0.1ms
 *   TOTAL:     3.6ms (margen 64%)
 */

#include "unity.h"
#include "dwt_delay.h"  // DWT cycle counter

typedef struct {
    const char* name;
    uint32_t budget_us;
    uint32_t worst_case_us;
    uint32_t avg_us;
    uint32_t samples;
} task_timing_t;

static task_timing_t timings[] = {
    {"IMU",       1200, 0, 0, 0},
    {"Baro",       800, 0, 0, 0},
    {"GPS",        300, 0, 0, 0},
    {"Logging",    500, 0, 0, 0},
    {"Telemetry",  400, 0, 0, 0},
    {"FSM",        200, 0, 0, 0},
    {"Recovery",   100, 0, 0, 0},
    {"Diag",       100, 0, 0, 0},
};

void test_all_tasks_within_budget(void) {
    // Después de 10s de ejecución, verificar timings
    for (int i = 0; i < 8; i++) {
        printf("%-10s: avg=%lu us, worst=%lu us, budget=%lu us %s\n",
               timings[i].name,
               timings[i].avg_us,
               timings[i].worst_case_us,
               timings[i].budget_us,
               timings[i].worst_case_us <= timings[i].budget_us ? "OK" : "FAIL");
        
        TEST_ASSERT_TRUE_MESSAGE(
            timings[i].worst_case_us <= timings[i].budget_us,
            timings[i].name
        );
    }
    
    // Verificar suma total
    uint32_t total = 0;
    for (int i = 0; i < 8; i++) {
        total += timings[i].worst_case_us;
    }
    printf("Total worst-case: %lu us / 10000 us\n", total);
    TEST_ASSERT_TRUE(total < 10000); // Debe caber en 10ms
}
```

### 4.4 Test: Flujo de Datos Completo

```c
/**
 * @file test_data_flow.c
 * @brief Verifica que los datos fluyen: sensor → processing → log + telemetry
 * 
 * Procedimiento:
 * 1. Inyectar datos conocidos en sensor HAL (mock)
 * 2. Esperar un ciclo completo
 * 3. Verificar que flash contiene los datos
 * 4. Verificar que telemetría transmitió los datos
 */

void test_data_flows_from_sensor_to_storage(void) {
    // Inyectar datos conocidos
    imu_data_t mock_imu = {
        .accel_x = 0.0f, .accel_y = 0.0f, .accel_z = 9.81f,
        .gyro_x = 0.0f, .gyro_y = 0.0f, .gyro_z = 0.0f,
        .timestamp_ms = 1000
    };
    sensor_inject_mock_imu(&mock_imu);
    
    // Esperar procesamiento
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Verificar flash
    log_record_t last_record;
    datalog_read_last(&last_record);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 9.81f, last_record.accel_z);
    
    // Verificar telemetría (leer buffer de salida SPI)
    telemetry_packet_t last_packet;
    telemetry_get_last_sent(&last_packet);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 9.81f, last_packet.accel_z);
}
```

---

## 5. Checklist de Integración

### Fase 1: Bus Validation
- [ ] I2C1: MPU6050 responde al scanner (0x68)
- [ ] I2C1: BMP280 responde al scanner (0x76)
- [ ] I2C1: INA219 responde al scanner (0x40)
- [ ] SPI1: W25Q128 devuelve JEDEC ID correcto
- [ ] SPI1: RFM95W devuelve versión de chip correcta
- [ ] UART1: GPS genera datos NMEA
- [ ] UART2: Debug console funcional

### Fase 2: Data Flow
- [ ] IMU → Madgwick → flight_data.orientation actualizado
- [ ] Baro → Altitude → flight_data.altitude actualizado
- [ ] GPS → Parse → flight_data.gps actualizado
- [ ] flight_data → DataLog → Flash escrito correctamente
- [ ] flight_data → Telemetry → Paquete LoRa transmitido
- [ ] INA219 → power_data → Health check usa datos

### Fase 3: State Machine
- [ ] PAD_IDLE → ARMED (arm switch)
- [ ] ARMED → BOOST (accel > 3g)
- [ ] BOOST → COAST (accel < 0.5g)
- [ ] COAST → APOGEE (velocity ≈ 0)
- [ ] APOGEE → DROGUE_DEPLOY (automático)
- [ ] DROGUE_DEPLOY → DESCENT_DROGUE (2s timer)
- [ ] DESCENT_DROGUE → MAIN_DEPLOY (alt < 150m)
- [ ] MAIN_DEPLOY → DESCENT_MAIN (2s timer)
- [ ] DESCENT_MAIN → RECOVERY_MODE (vel < 2 m/s)
- [ ] Backup timers funcionan si sensores fallan

### Fase 4: Recovery
- [ ] Continuity check detecta e-matches conectados
- [ ] Drogue dispara en apogeo
- [ ] Main dispara a 150m AGL
- [ ] Emergency deploy si falla primario
- [ ] Buzzer y beacon GPS en recovery

### Fase 5: Full Flight
- [ ] Simulación completa con datos reales de OpenRocket
- [ ] Todos los datos son físicamente coherentes
- [ ] Log de flash recuperable post-vuelo
- [ ] Telemetría recibida durante todo el vuelo
- [ ] Tiempo total de procesamiento < 10ms/ciclo

---

## 6. Script de Validación de Simulación

```python
#!/usr/bin/env python3
"""
validate_simulation.py
Valida la coherencia de datos entre sensores después de una simulación.

Uso:
    python validate_simulation.py --telemetry results/nominal_telemetry.log
                                  --flash results/nominal_flash_dump.bin
"""

import argparse
import numpy as np
import struct

def parse_telemetry_log(filepath):
    """Parsea log de telemetría del virtual terminal."""
    records = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('T:'):
                parts = line.strip().split(',')
                record = {
                    'timestamp_ms': int(parts[0].split(':')[1]),
                    'state': int(parts[1].split(':')[1]),
                    'altitude_m': float(parts[2].split(':')[1]),
                    'velocity_ms': float(parts[3].split(':')[1]),
                    'accel_z_g': float(parts[4].split(':')[1]),
                    'pressure_hpa': float(parts[5].split(':')[1]),
                    'lat': float(parts[6].split(':')[1]),
                    'lon': float(parts[7].split(':')[1]),
                }
                records.append(record)
    return records

def validate_apogee_coherence(records):
    """Verifica que en el apogeo: velocidad ≈ 0, presión mínima, altitud máxima."""
    altitudes = [r['altitude_m'] for r in records]
    max_alt_idx = np.argmax(altitudes)
    apogee = records[max_alt_idx]
    
    print(f"\n=== APOGEE VALIDATION (t={apogee['timestamp_ms']}ms) ===")
    print(f"  Altitude: {apogee['altitude_m']:.1f} m (max)")
    print(f"  Velocity: {apogee['velocity_ms']:.2f} m/s (should be ≈ 0)")
    print(f"  Pressure: {apogee['pressure_hpa']:.1f} hPa (should be min)")
    
    # Velocity should be near zero at apogee
    assert abs(apogee['velocity_ms']) < 5.0, \
        f"Velocity at apogee too high: {apogee['velocity_ms']}"
    
    # Pressure should be minimum at apogee
    pressures = [r['pressure_hpa'] for r in records]
    min_pressure_idx = np.argmin(pressures)
    assert abs(max_alt_idx - min_pressure_idx) <= 5, \
        f"Pressure minimum not at apogee (alt_idx={max_alt_idx}, p_idx={min_pressure_idx})"
    
    print("  ✓ Apogee coherence PASSED")

def validate_state_sequence(records):
    """Verifica que los estados siguen la secuencia esperada."""
    expected = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]  # PAD_IDLE to RECOVERY
    states_seen = []
    
    for r in records:
        s = r['state']
        if not states_seen or states_seen[-1] != s:
            states_seen.append(s)
    
    print(f"\n=== STATE SEQUENCE VALIDATION ===")
    print(f"  States seen: {states_seen}")
    print(f"  Expected:    {expected}")
    
    assert states_seen == expected, \
        f"State sequence mismatch: {states_seen} != {expected}"
    
    print("  ✓ State sequence PASSED")

def validate_altitude_pressure_correlation(records):
    """Verifica correlación inversa entre altitud y presión."""
    print(f"\n=== ALTITUDE-PRESSURE CORRELATION ===")
    
    for i in range(0, len(records), max(1, len(records)//10)):
        r = records[i]
        # Barometric formula: P = P0 * (1 - 0.0000225577 * h)^5.25588
        expected_p = 1013.25 * (1 - 0.0000225577 * r['altitude_m']) ** 5.25588
        error = abs(r['pressure_hpa'] - expected_p)
        
        print(f"  t={r['timestamp_ms']:6d}ms: alt={r['altitude_m']:7.1f}m, "
              f"P={r['pressure_hpa']:7.1f}hPa, expected={expected_p:7.1f}hPa, "
              f"err={error:.1f}hPa {'OK' if error < 5 else 'WARN'}")
    
    print("  ✓ Altitude-Pressure correlation checked")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Validate simulation coherence')
    parser.add_argument('--telemetry', required=True, help='Telemetry log file')
    args = parser.parse_args()
    
    records = parse_telemetry_log(args.telemetry)
    print(f"Loaded {len(records)} telemetry records")
    
    validate_apogee_coherence(records)
    validate_state_sequence(records)
    validate_altitude_pressure_correlation(records)
    
    print("\n" + "="*50)
    print("ALL VALIDATIONS PASSED")
    print("="*50)
```

---

## Siguiente Paso

> Comenzar con `full-system-schematic/` ensamblando los subcircuitos de cada módulo en un esquemático unificado en Proteus, utilizando Net Labels para las conexiones entre bloques.
