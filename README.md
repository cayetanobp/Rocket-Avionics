# 🚀 Rocket Avionics - Full Avionics Simulation Platform

<p align="center">
  <img src="docs/assets/banner-placeholder.png" alt="Rocket Avionics Banner" width="800"/>
</p>

<p align="center">
  <strong>Plataforma modular de aviónica para cohetes con simulación completa en Proteus VSM</strong>
</p>

<p align="center">
  <a href="#módulos">Módulos</a> •
  <a href="#arquitectura">Arquitectura</a> •
  <a href="#simulación-con-datos-reales">Simulación</a> •
  <a href="#cómo-empezar">Comenzar</a> •
  <a href="#roadmap">Roadmap</a>
</p>

---

## Descripción General

**Rocket Avionics** es un sistema completo de aviónica para cohetes sounding/amateur, diseñado con arquitectura modular, desarrollado y validado íntegramente en simulación antes de fabricar hardware. Cada subsistema se desarrolla, prueba y documenta de forma independiente antes de integrarse en el sistema completo.

### Características Destacadas

- **12 módulos independientes** con interfaces bien definidas
- **RTOS (FreeRTOS)** para gestión determinista de tareas en tiempo real
- **Datos de vuelo reales** (CSV de OpenRocket/vuelos reales) inyectados en la simulación para validación física coherente
- **Correlación de datos multi-sensor**: presión, aceleración, altitud, velocidad y GPS sincronizados temporalmente
- **Máquina de estados completa** con 10 estados de vuelo
- **Sistema de recuperación dual** (drogue + main) con detección redundante de apogeo
- **Simulación en Proteus VSM** con instrumentación virtual completa
- **Documentación exhaustiva** de cada decisión técnica

---

## Arquitectura

```
                        ┌─────────────────────────┐
                        │    POWER MANAGEMENT      │
                        │   LiPo 3S → 5V / 3.3V   │
                        └────────────┬────────────┘
                                     │
            ┌────────────────────────┼────────────────────────┐
            │                        │                        │
        ┌───▼───┐              ┌─────▼─────┐            ┌────▼────┐
        │  5V   │              │  STM32F4  │            │  INA219 │
        │ Rail  │              │  MCU CORE │            │ Battery │
        └───┬───┘              │  FreeRTOS │            │ Monitor │
            │                  └─────┬─────┘            └─────────┘
            │                        │
            │    ┌───────┬───────┬───┴───┬───────┬───────┐
            │    │       │       │       │       │       │
        ┌───▼┐ ┌─▼──┐ ┌─▼──┐ ┌─▼──┐ ┌──▼─┐ ┌──▼──┐ ┌──▼──┐
        │Srv │ │IMU │ │BARO│ │GPS │ │LoRa│ │Flash│ │Diag │
        │PWM │ │I2C │ │I2C │ │UART│ │SPI │ │SPI  │ │UART │
        └────┘ └──┬─┘ └──┬─┘ └──┬─┘ └──┬─┘ └──┬──┘ └──┬──┘
                  │      │      │      │      │       │
                  └──────┴──────┴──────┴──────┴───────┘
                                     │
                        ┌────────────▼────────────┐
                        │     STATE MACHINE       │
                        │  PAD → BOOST → APOGEE   │
                        │  → DESCENT → LANDED     │
                        └────────────┬────────────┘
                                     │
                        ┌────────────▼────────────┐
                        │   RECOVERY SYSTEM       │
                        │  Drogue + Main Chute    │
                        └─────────────────────────┘
```

### Buses de Comunicación

| Bus   | Dispositivos                              | Velocidad     |
|-------|-------------------------------------------|---------------|
| I2C1  | MPU6050, BMP280, INA219                   | 400 kHz       |
| SPI1  | W25Q128 Flash, RFM95W LoRa               | 8 MHz         |
| UART1 | NEO-6M GPS                                | 9600 baud     |
| UART2 | Debug Terminal                            | 115200 baud   |
| PWM   | Servos (TIM2/TIM3)                        | 50 Hz         |
| GPIO  | Pyro channels, LEDs, Arming switch        | -             |

---

## Módulos

| #  | Módulo                  | Descripción                                  | Estado |
|----|-------------------------|----------------------------------------------|--------|
| 01 | [Power Management](01-power-management/)    | Regulación, monitorización y protección      | 🔴     |
| 02 | [MCU Core](02-mcu-core/)                    | STM32F4 + FreeRTOS, cerebro del sistema      | 🔴     |
| 03 | [Sensors Interface](03-sensors/)             | Abstracción I2C/SPI, detección automática    | 🔴     |
| 04 | [IMU Navigation](04-imu-navigation/)         | MPU6050 + Madgwick, orientación 3D           | 🔴     |
| 05 | [Barometer Altimeter](05-barometer-altimeter/)| BMP280, altitud + detección de apogeo        | 🔴     |
| 06 | [GNSS GPS](06-gnss-gps/)                    | NEO-6M, posicionamiento + recovery           | 🔴     |
| 07 | [Telemetry Radio](07-telemetry-radio/)       | LoRa RFM95W, downlink en tiempo real         | 🔴     |
| 08 | [Data Logging](08-data-logging/)             | Flash SPI W25Q128, 100Hz+ logging            | 🔴     |
| 09 | [Recovery System](09-recovery-system/)       | Pyro channels, dual deploy                   | 🔴     |
| 10 | [Flight Control](10-flight-control/)         | PID + servos, estabilización activa          | 🔴     |
| 11 | [State Machine](11-state-machine/)           | FSM de 10 estados, ciclo de vuelo            | 🔴     |
| 12 | [Diagnostics](12-diagnostics-debug/)         | Logging, health checks, watchdog             | 🔴     |

**Estado**: 🔴 No iniciado | 🟡 En desarrollo | 🟢 Completado | ✅ Validado en simulación

---

## Simulación con Datos Reales

### Filosofía de Simulación

A diferencia de simulaciones con valores arbitrarios, este proyecto utiliza **datos de vuelo reales** (exportados de OpenRocket o registros de vuelos) para alimentar la simulación en Proteus. Esto garantiza:

1. **Coherencia física**: Cuando la presión es mínima, la velocidad vertical es 0 (apogeo)
2. **Perfiles realistas**: Aceleración, velocidad y altitud siguen curvas físicamente posibles
3. **Correlación temporal**: Todos los sensores reciben datos del mismo instante de vuelo
4. **Validación end-to-end**: La máquina de estados transiciona correctamente con datos reales

### Pipeline de Datos

```
OpenRocket / Vuelo Real
        │
        ▼
   CSV con datos crudos
   (t, alt, vel, accel, pressure, ...)
        │
        ▼
   Script de conversión (Python)
   ├── Genera valores I2C para BMP280
   ├── Genera valores I2C para MPU6050
   ├── Genera tramas NMEA para GPS
   └── Calcula estados esperados
        │
        ▼
   Archivos de estímulo para Proteus
   (señales analógicas / digital patterns)
        │
        ▼
   Simulación con datos correlacionados
```

Ver [flight-data/README.md](flight-data/) y [docs/simulation_guide.md](docs/simulation_guide.md) para detalles completos.

---

## Cómo Empezar

### Requisitos

| Herramienta        | Versión          | Uso                                    |
|--------------------|------------------|----------------------------------------|
| Proteus            | 8.9+             | Simulación de circuitos y firmware     |
| STM32CubeIDE       | 1.12+            | Desarrollo y compilación de firmware   |
| Python             | 3.10+            | Scripts de conversión de datos         |
| Git                | 2.x              | Control de versiones                   |
| OpenRocket         | 23.09+           | Generación de perfiles de vuelo        |

### Inicio Rápido

```bash
# Clonar el repositorio
git clone https://github.com/tu-usuario/rocket-avionics.git
cd rocket-avionics

# Instalar dependencias de Python (para scripts de datos)
pip install -r tools/requirements.txt

# Abrir el módulo que quieras trabajar en Proteus
# Ej: 01-power-management/simulation/power_management.pdsprj
```

### Orden de Desarrollo Recomendado

```
01-Power → 02-MCU → 12-Diagnostics → 03-Sensors → 04-IMU
    → 05-Baro → 06-GPS → 08-DataLog → 07-Telemetry
    → 11-StateMachine → 09-Recovery → 10-FlightControl
    → Integration
```

---

## Roadmap

### Fase 1 - Fundación ✳️ En curso
- [x] Definir arquitectura y plan del proyecto
- [x] Crear estructura de repositorio
- [ ] Configurar entorno de desarrollo (Proteus + STM32CubeIDE)
- [ ] Preparar datos de vuelo de referencia

### Fase 2 - Desarrollo Modular
- [ ] Módulo 01: Power Management
- [ ] Módulo 02: MCU Core + FreeRTOS
- [ ] Módulo 12: Diagnostics (debug temprano)
- [ ] Módulo 03: Sensor Interface
- [ ] Módulo 04: IMU Navigation
- [ ] Módulo 05: Barometer Altimeter
- [ ] Módulo 06: GNSS GPS
- [ ] Módulo 08: Data Logging
- [ ] Módulo 07: Telemetry Radio

### Fase 3 - Lógica de Vuelo
- [ ] Módulo 11: State Machine
- [ ] Módulo 09: Recovery System
- [ ] Módulo 10: Flight Control (PID)

### Fase 4 - Integración y Validación
- [ ] Integración completa en esquemático único
- [ ] Simulación de vuelo completo con datos reales
- [ ] Análisis de timing y rendimiento RTOS
- [ ] Video demo para portfolio

### Fase 5 - Hardware (Post-simulación)
- [ ] Diseño PCB
- [ ] Fabricación y ensamblaje
- [ ] Testing en banco
- [ ] Vuelo de prueba

---

## Estructura del Repositorio

```
rocket-avionics/
├── README.md                          ← Este archivo
├── Plan_Principal.md                  ← Plan original del proyecto
├── docs/                              ← Documentación global
│   ├── architecture.md                ← Arquitectura detallada
│   ├── requirements.md                ← Requisitos del sistema
│   └── simulation_guide.md            ← Guía de simulación en Proteus
├── flight-data/                       ← Datos de vuelo para simulación
│   ├── raw/                           ← CSVs originales
│   ├── processed/                     ← Datos convertidos para Proteus
│   └── profiles/                      ← Perfiles de vuelo predefinidos
├── tools/                             ← Scripts y utilidades
│   ├── data-converter/                ← Conversor CSV → estímulos Proteus
│   └── proteus-scripts/               ← Macros y scripts de Proteus
├── 01-power-management/               ← Módulo de gestión de energía
│   ├── README.md
│   ├── schematics/                    ← Esquemáticos Proteus
│   ├── simulation/                    ← Proyecto de simulación
│   └── code/                          ← Firmware del módulo
├── 02-mcu-core/                       ← MCU + RTOS
├── ...                                ← Módulos 03-12
├── integration/                       ← Sistema integrado
│   ├── full-system-schematic/
│   ├── full-system-simulation/
│   ├── pcb-design/
│   └── tests/
└── .gitignore
```

---

## Tecnologías Clave

| Categoría          | Tecnología                              |
|--------------------|-----------------------------------------|
| MCU                | STM32F407VGT6 (ARM Cortex-M4, 168 MHz) |
| RTOS               | FreeRTOS v10.x                          |
| IMU                | MPU6050 (6-DOF)                         |
| Barómetro          | BMP280                                  |
| GPS                | NEO-6M (NMEA 0183)                      |
| Radio              | RFM95W (LoRa, 433/868 MHz)             |
| Flash              | W25Q128 (16 MB SPI Flash)               |
| Battery Monitor    | INA219                                  |
| Simulación         | Proteus 8.9+ VSM                        |
| Filtro IMU         | Madgwick AHRS                           |
| Control            | PID discreto                            |

---

## Para Portfolio

Este proyecto demuestra competencias en:

- **Sistemas Embebidos**: ARM Cortex-M4, periféricos, buses I2C/SPI/UART
- **RTOS**: FreeRTOS, scheduling, prioridades, sincronización
- **Sensores**: Fusión IMU, filtrado Madgwick, correlación multi-sensor
- **Control**: PID discreto, máquinas de estado, sistemas de recuperación
- **Telemetría**: Protocolos LoRa, empaquetado de datos, CRC
- **Metodología**: Diseño modular, simulación antes de hardware, documentación técnica
- **Datos reales**: Validación con perfiles de vuelo físicamente coherentes

---

## Licencia

MIT License - Ver [LICENSE](LICENSE)

---

<p align="center">
  <sub>Desarrollado como proyecto de portfolio para ingeniería aeroespacial y sistemas embebidos</sub>
</p>
