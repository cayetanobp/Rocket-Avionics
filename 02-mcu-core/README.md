# Módulo 02: MCU Core - Núcleo del Microcontrolador

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Segundo módulo a desarrollar  
> **Dependencias**: 01-Power Management  
> **Dependientes**: Todos los módulos de sensores y lógica

---

## Resumen

Cerebro del sistema de aviónica. El STM32F407VGT6 ejecuta FreeRTOS y coordina la lectura de sensores, ejecución de la máquina de estados, logging, telemetría y control de recuperación. Este módulo establece la base sobre la cual se construyen todos los demás.

---

## Especificaciones Técnicas

### Microcontrolador
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| MCU                | STM32F407VGT6                   |
| Core               | ARM Cortex-M4 con FPU           |
| Frecuencia         | 168 MHz                         |
| Flash              | 1 MB                            |
| SRAM               | 192 KB                          |
| Package            | LQFP100                         |
| Voltaje operación  | 1.8V - 3.6V                     |
| Modelo Proteus     | ✅ VSM disponible               |

### Oscilador
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| HSE (externo)      | 8 MHz cristal                   |
| PLL configuración  | 8 MHz → 168 MHz                 |
| LSE (RTC)          | 32.768 kHz (opcional)           |

### FreeRTOS
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Versión            | v10.4.x (CMSIS-RTOS v2 wrapper) |
| Tick rate          | 1000 Hz (1ms tick)              |
| Heap scheme        | heap_4 (coalescente)            |
| Heap size          | 32 KB                           |
| Max priorities     | 7                               |
| Stack overflow     | Check method 2 (pattern fill)   |
| Timer task         | Habilitado                      |

---

## Decisiones Técnicas

### ¿Por qué STM32F407 y no STM32F103?
- F407 tiene **FPU hardware**: filtro Madgwick/Kalman ~10× más rápido que F103
- 168 MHz vs 72 MHz: margen de procesamiento para todas las tareas a 100Hz
- 192 KB SRAM vs 20 KB: espacio para buffers de logging y stacks de 8 tareas RTOS
- Más periféricos (3xI2C, 3xSPI, 6xUART) para no compartir buses
- **Ambos tienen modelo VSM en Proteus** - F407 es la elección superior

### ¿Por qué FreeRTOS y no bare-metal con timer interrupts?
- **Bare-metal**: Más simple para 2-3 tareas, pero escala mal a 8+ tareas
- **FreeRTOS**: 
  - Prioridades por tarea: Recovery (máx) nunca se bloquea por Telemetry (mín)
  - Colas (queues) para pasar datos entre sensores y state machine
  - Mutexes para buses compartidos (I2C, SPI) - evita corrupción de datos
  - Stack overflow detection - critical para sistemas de vuelo
  - Overhead mínimo: ~2% CPU para scheduler a 1kHz tick

### ¿Por qué `heap_4` y no `heap_1`?
- `heap_1`: Solo alloca, nunca libera - eficiente pero no permite borrar tareas/colas
- `heap_4`: Alloca y libera con coalescencia - flexibilidad para crear/destruir recursos dinámicamente
- En un sistema embebido de vuelo, la mayoría de allocations son al boot, pero `heap_4` da margen

### ¿Por qué 1ms tick rate?
- 1ms = resolución temporal suficiente para detectar apogeo (50ms budget)
- Tareas a 100Hz necesitan `vTaskDelayUntil` con período de 10 ticks → baja granularidad de error
- 10ms tick significaría período de 1 tick para 100Hz → jitter de hasta 100%

---

## Configuración de Clocks

```
HSE (8 MHz) ──▶ PLL ──▶ SYSCLK = 168 MHz
                           │
                           ├── AHB = 168 MHz (HCLK)
                           │     ├── Cortex System Timer (SysTick) = 168 MHz
                           │     ├── AHB1: GPIO, DMA
                           │     └── AHB2: USB (no usado)
                           │
                           ├── APB1 = 42 MHz (máx)
                           │     ├── I2C1 (sensores)
                           │     ├── UART2 (debug)
                           │     ├── TIM2-7 (PWM servos, timing)
                           │     └── IWDG, WWDG
                           │
                           └── APB2 = 84 MHz (máx)
                                 ├── SPI1 (Flash, LoRa)
                                 ├── UART1 (GPS)
                                 ├── ADC1
                                 └── TIM1, TIM8
```

---

## Diseño del Circuito

### Diagrama de Bloques

```
                              ┌─────────────────────┐
    3.3V ───────────────────▶│    STM32F407VGT6     │
                              │                     │
    8MHz XTAL ──┬── OSC_IN   │  PB6/PB7 ── I2C1    │── SDA/SCL → Sensores
                └── OSC_OUT  │  PA5-7   ── SPI1    │── SCK/MISO/MOSI → Flash/LoRa
                              │  PA9/10  ── UART1   │── TX/RX → GPS
    32.768kHz ──┬── OSC32_IN │  PA2/3   ── UART2   │── TX/RX → Debug
                └── OSC32_OUT│  PA0-1   ── TIM2    │── PWM → Servos
                              │  PC0-1   ── GPIO    │── PYRO channels
    NRST ── 10kΩ → 3.3V     │  PC2-6   ── GPIO    │── LEDs, Buzzer
    + 100nF → GND            │  PD0-2   ── GPIO    │── Switches, Continuity
    + Button → GND            │                     │
                              │  BOOT0 ── GND       │
    SWD ── SWDIO/SWCLK       │  VDD ── 3.3V (×5)   │
                              │  VSS ── GND (×5)     │
                              │  VDDA ── 3.3V (ferrita) │
                              └─────────────────────┘
```

### Componentes del Esquemático

| Ref  | Componente              | Valor/Tipo        | Notas                |
|------|-------------------------|-------------------|----------------------|
| U1   | STM32F407VGT6           | LQFP100           | MCU principal        |
| Y1   | Cristal                 | 8 MHz             | 20pF load caps       |
| Y2   | Cristal                 | 32.768 kHz        | 6.8pF load caps      |
| C1-C2| Caps de cristal HSE     | 20pF              |                      |
| C3-C4| Caps de cristal LSE     | 6.8pF             |                      |
| C5-C9| Bypass caps VDD         | 100nF (×5)        | Uno por pin VDD      |
| C10  | Bypass VDDA             | 1µF + 100nF       | Rail analógico       |
| R1   | Pull-up NRST            | 10kΩ              |                      |
| C11  | Debounce NRST           | 100nF              |                      |
| SW1  | Reset button            | Momentary          |                      |
| J1   | Conector SWD            | 2×5 header         | SWDIO, SWCLK, GND    |
| LED1 | Heartbeat               | Verde 3mm          | PC2 → 330Ω → LED    |

---

## Desarrollo Step-by-Step

### Paso 1: Esquemático Mínimo del MCU
1. Crear proyecto Proteus: `02-mcu-core/simulation/mcu_core.pdsprj`
2. Colocar STM32F407VGT6
3. Conectar cristal de 8 MHz con caps de carga (20pF)
4. Conectar todos los pines VDD a 3.3V con bypass caps (100nF cada uno)
5. Conectar todos los pines VSS a GND
6. Circuito de reset: 10kΩ pull-up + 100nF + botón
7. BOOT0 a GND (boot desde flash)
8. **Test**: Verificar que el MCU arranca (LED heartbeat o debug UART)

### Paso 2: Configurar STM32CubeMX
1. Crear proyecto CubeMX para STM32F407VGT6
2. Configurar clocks: HSE 8MHz → PLL → 168MHz
3. Habilitar periféricos:
   - I2C1 (PB6/PB7) @ 400kHz
   - SPI1 (PA5/PA6/PA7) @ 8MHz
   - UART1 (PA9/PA10) @ 9600 baud
   - UART2 (PA2/PA3) @ 115200 baud
   - TIM2 (PWM, 4 canales)
   - GPIO outputs (PC0-PC6)
   - GPIO inputs (PD0-PD2)
   - IWDG (1s timeout)
4. Habilitar FreeRTOS (CMSIS-RTOS v2)
5. Generar código

### Paso 3: FreeRTOS Bootstrap
1. Verificar que el kernel arranca correctamente
2. Crear tarea `vTaskHeartbeat`:
   ```c
   void vTaskHeartbeat(void *pvParameters) {
       for (;;) {
           HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
           vTaskDelay(pdMS_TO_TICKS(500));
       }
   }
   ```
3. Compilar y cargar en Proteus
4. **Test**: LED heartbeat debe parpadear a 1Hz (500ms ON, 500ms OFF)

### Paso 4: Debug UART
1. Implementar `_write()` para redirigir `printf()` a UART2:
   ```c
   int _write(int file, char *ptr, int len) {
       HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
       return len;
   }
   ```
2. Abrir Virtual Terminal en Proteus conectado a UART2
3. **Test**: Enviar "System Boot OK" por UART al arrancar

### Paso 5: Watchdog
1. Configurar IWDG con timeout de 1 segundo
2. Refrescar watchdog en tarea de máxima prioridad o en idle hook
3. **Test**: Bloquear una tarea → MCU debe reiniciarse en ~1s

### Paso 6: Creación de Todas las Tareas Placeholder
1. Crear todas las tareas RTOS (vacías, solo con delay):
   ```c
   xTaskCreate(vTaskIMU,         "IMU",      256, NULL, 4, NULL);
   xTaskCreate(vTaskBarometer,   "BARO",     256, NULL, 4, NULL);
   xTaskCreate(vTaskGPS,         "GPS",      256, NULL, 3, NULL);
   xTaskCreate(vTaskStateMachine,"FSM",      512, NULL, 5, NULL);
   xTaskCreate(vTaskTelemetry,   "TELEM",    256, NULL, 2, NULL);
   xTaskCreate(vTaskLogging,     "LOG",      512, NULL, 3, NULL);
   xTaskCreate(vTaskRecovery,    "RECOVERY", 256, NULL, 6, NULL);
   xTaskCreate(vTaskDiagnostics, "DIAG",     256, NULL, 1, NULL);
   ```
2. Cada tarea imprime su nombre al arrancar
3. **Test**: Verificar en Virtual Terminal que todas las tareas arrancan

### Paso 7: Mecanismos de Sincronización
1. Crear mutexes para buses compartidos:
   ```c
   SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();
   SemaphoreHandle_t spi_mutex = xSemaphoreCreateMutex();
   ```
2. Crear colas para transferencia de datos:
   ```c
   QueueHandle_t sensor_data_q = xQueueCreate(10, sizeof(sensor_data_t));
   QueueHandle_t gps_data_q    = xQueueCreate(5, sizeof(gps_data_t));
   QueueHandle_t telem_q       = xQueueCreate(10, sizeof(telem_packet_t));
   QueueHandle_t log_q         = xQueueCreate(20, sizeof(log_entry_t));
   ```
3. Crear event group para eventos:
   ```c
   EventGroupHandle_t flight_events = xEventGroupCreate();
   #define EVT_DEPLOY_DROGUE (1 << 0)
   #define EVT_DEPLOY_MAIN   (1 << 1)
   #define EVT_LANDED         (1 << 2)
   ```
4. **Test**: Enviar dato por cola y verificar recepción en otra tarea

---

## Interfaz con Otros Módulos

### Estructura de Datos Global
```c
// shared_data.h - Tipos compartidos entre todos los módulos

typedef struct {
    float accel_x, accel_y, accel_z;  // m/s²
    float gyro_x, gyro_y, gyro_z;     // °/s
    float pitch, roll, yaw;           // ° (Euler angles)
    float q0, q1, q2, q3;            // Quaternion
} imu_data_t;

typedef struct {
    float pressure;     // hPa
    float temperature;  // °C
    float altitude;     // m (AGL)
    float vspeed;       // m/s (vertical velocity)
} baro_data_t;

typedef struct {
    float latitude;     // decimal degrees
    float longitude;    // decimal degrees
    float altitude_gps; // m
    float speed;        // km/h
    uint8_t fix;        // 0=none, 1=2D, 2=3D
    uint8_t satellites; // count
} gps_data_t;

typedef struct {
    uint32_t timestamp_ms;
    imu_data_t imu;
    baro_data_t baro;
    gps_data_t gps;
    power_data_t power;
    uint8_t flight_state;
} flight_data_t;
```

---

## Criterios de Aceptación

- [ ] MCU arranca correctamente a 168 MHz (verificar con osciloscopio en pin MCO)
- [ ] FreeRTOS kernel inicia y todas las tareas se ejecutan
- [ ] LED heartbeat a 1 Hz estable
- [ ] Debug UART funcional (printf a Virtual Terminal)
- [ ] Watchdog reinicia el MCU ante bloqueo de tarea
- [ ] Mutexes protegen buses I2C y SPI correctamente
- [ ] Colas transfieren datos entre tareas sin pérdida
- [ ] Stack overflow detection funciona (forzar overflow → callback ejecuta)
- [ ] Boot sequence completo en < 500ms

---

## Simulación en Proteus

### Instrumentos a Utilizar
- **Virtual Terminal**: Conectado a UART2 (PA2/PA3) para debug
- **Oscilloscope**: En pin PC2 para verificar heartbeat timing
- **I2C Debugger**: En PB6/PB7 para verificar inicialización del bus
- **Logic Analyzer**: En pines GPIO para timing de tareas

### Escenarios de Test

| # | Escenario                         | Resultado Esperado                         |
|---|-----------------------------------|--------------------------------------------|
| 1 | Power-on boot                     | Boot message en UART, heartbeat arranca    |
| 2 | Todas las tareas running          | 8 mensajes de "Task started" en UART       |
| 3 | Watchdog timeout                  | MCU reinicia, boot message repetido        |
| 4 | Stack overflow                    | Callback ejecuta, error reportado          |
| 5 | Queue overflow                    | Dato más antiguo descartado, log warning   |
| 6 | I2C bus sin dispositivos          | Timeout detectado, error reportado         |

---

## Referencias

- [STM32F407 Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [STM32F407 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [FreeRTOS API Reference](https://www.freertos.org/a00106.html)
- [FreeRTOS STM32 Port](https://www.freertos.org/RTOS-Cortex-M3-M4.html)

---

*Módulo 02 - El cerebro. Todo depende de que este módulo funcione correctamente.*
