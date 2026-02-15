# Módulo 11: State Machine - Máquina de Estados del Vuelo

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 3 - Desarrollar después de sensores, antes de recovery  
> **Dependencias**: 02-MCU Core, 04-IMU, 05-Barometer, 09-Recovery  
> **Dependientes**: Todos los módulos consultan el estado actual

---

## Resumen

La FSM (Finite State Machine) es la **lógica central de vuelo**: decide en qué fase del vuelo se encuentra el cohete y coordina las acciones de cada módulo. Define 10 estados con transiciones basadas en datos de sensores, y es el único componente autorizado para disparar eventos como el despliegue de paracaídas.

---

## Especificaciones Técnicas

### Estados
| ID | Estado            | Descripción                                | LED Color |
|----|-------------------|--------------------------------------------|-----------|
| 0  | `PAD_IDLE`        | En rampa, sistemas iniciando               | Blanco    |
| 1  | `ARMED`           | Armado, esperando lanzamiento              | Verde fijo|
| 2  | `BOOST`           | Motor encendido, ascenso propulsado        | Rojo fijo |
| 3  | `COAST`           | Motor apagado, ascenso inercial            | Amarillo  |
| 4  | `APOGEE`          | Punto máximo, despliegue de drogue         | Azul flash|
| 5  | `DESCENT_DROGUE`  | Descenso con drogue chute                  | Azul fijo |
| 6  | `MAIN_DEPLOY`     | Despliegue de paracaídas principal         | Magenta   |
| 7  | `DESCENT_MAIN`    | Descenso con main chute                    | Cian      |
| 8  | `LANDED`          | Aterrizaje detectado                       | Verde fijo|
| 9  | `RECOVERY_MODE`   | Señal de localización activa               | Verde parpadeo|

### Transiciones
| Desde             | Hacia              | Condición                                   | Timeout Backup |
|--------------------|--------------------|--------------------------------------------|----------------|
| `PAD_IDLE`        | `ARMED`            | Arming switch ON + continuity OK            | -              |
| `ARMED`           | `PAD_IDLE`         | Arming switch OFF                           | -              |
| `ARMED`           | `BOOST`            | accel_z > 2g sostenida 100 ms              | -              |
| `BOOST`           | `COAST`            | accel_z < 0.5g (motor burnout)             | 5 s timeout    |
| `COAST`           | `APOGEE`           | vspeed cambia de + a – (3 muestras)         | 15 s timeout   |
| `APOGEE`          | `DESCENT_DROGUE`   | Inmediato tras fire drogue                  | 0.5 s          |
| `DESCENT_DROGUE`  | `MAIN_DEPLOY`      | altitud < MAIN_ALT (300 m)                 | 60 s timeout   |
| `MAIN_DEPLOY`     | `DESCENT_MAIN`     | Inmediato tras fire main                    | 0.5 s          |
| `DESCENT_MAIN`    | `LANDED`           | |vspeed| < 0.5 m/s durante 3 s             | 120 s timeout  |
| `LANDED`          | `RECOVERY_MODE`    | Automático tras 10 s                        | -              |

---

## Decisiones Técnicas

### ¿Por qué FSM y no un sistema de reglas continuo?
- **FSM**: Estados discretos, transiciones claras, fácil de debuggear y verificar
- **Reglas continuas**: Más flexible pero difícil de razonar sobre seguridad
- Las competiciones de cohetes requieren poder demostrar la lógica de deploy - una FSM visual es perfecta
- Los logs de transiciones son legibles: `[FSM] COAST → APOGEE @ t=12345ms, alt=800.5m`

### ¿Por qué 100 ms de sostenimiento para detección de launch?
- Un bump o vibración puede causar un pico de 2g momentáneo (< 50 ms)
- 100 ms de aceleración sostenida solo puede ser un motor encendido
- No afecta al timing: el cohete sigue en la rampa durante los primeros 100 ms de motor

### ¿Por qué 3 muestras consecutivas para detección de apogeo?
- Una sola muestra con vspeed < 0 puede ser ruido
- 3 muestras consecutivas a 100 Hz = 30 ms → estadísticamente robusto
- Latencia total: 30 ms + latencia del IIR filter (~30 ms) = ~60 ms → dentro del budget de 50 ms post-filter

### ¿Por qué timeouts de backup?
- Si un sensor falla, la transición nunca ocurriría → cohete podría caer sin paracaídas
- Timeout: sube un "floor" de seguridad - si no transiciona naturalmente, fuerza la transición
- Ejemplo: si COAST no detecta apogeo en 15s, fuerza deploy de drogue (mejor temprano que nunca)

### ¿Por qué la FSM tiene la prioridad más alta después de Recovery?
- La FSM evalúa condiciones de deploy - delay aquí = delay en abrir paracaídas
- Prioridad 5 (debajo de Recovery a 6): que la FSM decida rápido, y Recovery actúe inmediatamente

---

## Diagrama de Estados

```
                            ┌──────────────┐
                     ┌──────│  PAD_IDLE    │
                     │      │ (init, cal)  │
                     │      └──────┬───────┘
                     │             │ ARM switch + continuity
                DISARM             │
                     │      ┌──────▼───────┐
                     └──────│   ARMED      │
                            │ (waiting)    │
                            └──────┬───────┘
                                   │ accel_z > 2g for 100ms
                            ┌──────▼───────┐
                            │   BOOST      │──── Timeout 5s
                            │ (propulsión) │     → force COAST
                            └──────┬───────┘
                                   │ accel_z < 0.5g
                            ┌──────▼───────┐
                            │   COAST      │──── Timeout 15s
                            │ (inercia)    │     → force APOGEE
                            └──────┬───────┘
                                   │ vspeed: + → -
                            ┌──────▼───────┐     
                            │   APOGEE     │──── Fire drogue
                            │ (máximo)     │     
                            └──────┬───────┘
                                   │ immediate
                            ┌──────▼──────────┐
                            │ DESCENT_DROGUE  │──── Timeout 60s
                            │ (~15 m/s)       │     → force MAIN
                            └──────┬──────────┘
                                   │ alt < 300m
                            ┌──────▼──────────┐
                            │  MAIN_DEPLOY    │──── Fire main
                            │                 │
                            └──────┬──────────┘
                                   │ immediate
                            ┌──────▼──────────┐
                            │  DESCENT_MAIN   │──── Timeout 120s
                            │ (~5 m/s)        │     → force LANDED
                            └──────┬──────────┘
                                   │ |vspeed| < 0.5 for 3s
                            ┌──────▼───────┐
                            │   LANDED     │
                            │              │
                            └──────┬───────┘
                                   │ after 10s
                            ┌──────▼──────────┐
                            │ RECOVERY_MODE   │
                            │ (buzzer + GPS)  │
                            └─────────────────┘
```

---

## Desarrollo Step-by-Step

### Paso 1: Definición de Estados y Transiciones
```c
// state_machine.h

typedef enum {
    STATE_PAD_IDLE = 0,
    STATE_ARMED,
    STATE_BOOST,
    STATE_COAST,
    STATE_APOGEE,
    STATE_DESCENT_DROGUE,
    STATE_MAIN_DEPLOY,
    STATE_DESCENT_MAIN,
    STATE_LANDED,
    STATE_RECOVERY_MODE,
    STATE_COUNT
} flight_state_t;

// String names for logging
static const char *state_names[] = {
    "PAD_IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
    "DESCENT_DROGUE", "MAIN_DEPLOY", "DESCENT_MAIN",
    "LANDED", "RECOVERY_MODE"
};
```

### Paso 2: Estructura de la FSM
```c
typedef struct {
    flight_state_t current_state;
    uint32_t state_entry_time;     // Tick de entrada al estado actual
    uint32_t launch_time;          // Tick de detección de launch
    
    // Contadores de condición
    uint16_t launch_accel_count;   // Muestras consecutivas > 2g
    uint8_t  apogee_vspeed_count;  // Muestras consecutivas vspeed < 0
    uint16_t landed_still_count;   // Muestras consecutivas vspeed ≈ 0
    
    // Timeouts configurables
    uint32_t boost_timeout_ms;     // 5000 ms
    uint32_t coast_timeout_ms;     // 15000 ms
    uint32_t drogue_timeout_ms;    // 60000 ms
    uint32_t main_timeout_ms;      // 120000 ms
    float    main_deploy_alt;      // 300.0 m
    
    // Datos del vuelo
    float    max_altitude;
    float    max_speed;
    float    max_accel;
} fsm_context_t;

static fsm_context_t fsm;
```

### Paso 3: Inicialización
```c
void fsm_init(void) {
    fsm.current_state = STATE_PAD_IDLE;
    fsm.state_entry_time = HAL_GetTick();
    fsm.launch_time = 0;
    fsm.launch_accel_count = 0;
    fsm.apogee_vspeed_count = 0;
    fsm.landed_still_count = 0;
    
    // Timeouts
    fsm.boost_timeout_ms = 5000;
    fsm.coast_timeout_ms = 15000;
    fsm.drogue_timeout_ms = 60000;
    fsm.main_timeout_ms = 120000;
    fsm.main_deploy_alt = 300.0f;
    
    fsm.max_altitude = 0;
    fsm.max_speed = 0;
    fsm.max_accel = 0;
    
    printf("[FSM] Initialized → PAD_IDLE\n");
}
```

### Paso 4: Función de Transición
```c
static void fsm_transition(flight_state_t new_state) {
    uint32_t now = HAL_GetTick();
    uint32_t time_in_state = now - fsm.state_entry_time;
    
    printf("[FSM] %s → %s @ t=%lu ms (was in state for %lu ms)\n",
           state_names[fsm.current_state], state_names[new_state],
           now, time_in_state);
    
    fsm.current_state = new_state;
    fsm.state_entry_time = now;
    
    // Acciones al entrar al nuevo estado
    switch (new_state) {
        case STATE_BOOST:
            fsm.launch_time = now;
            recovery_start_backup_timers();
            printf("[FSM] Launch detected! Backup timers started.\n");
            break;
            
        case STATE_APOGEE:
            printf("[FSM] APOGEE at alt=%.1fm, max_speed=%.1fm/s\n",
                   fsm.max_altitude, fsm.max_speed);
            xEventGroupSetBits(flight_events, EVT_DEPLOY_DROGUE);
            break;
            
        case STATE_MAIN_DEPLOY:
            xEventGroupSetBits(flight_events, EVT_DEPLOY_MAIN);
            break;
            
        case STATE_LANDED:
            logger_flush();
            printf("[FSM] LANDED. Max alt=%.1fm, max_accel=%.1fg\n",
                   fsm.max_altitude, fsm.max_accel / 9.81f);
            break;
            
        case STATE_RECOVERY_MODE:
            printf("[FSM] RECOVERY MODE: Buzzer ON, GPS tracking active\n");
            // Activar buzzer, cambiar telemetría a GPS-only mode
            break;
            
        default:
            break;
    }
}
```

### Paso 5: Evaluación de Transiciones
```c
void fsm_update(const imu_data_t *imu, const baro_data_t *baro) {
    uint32_t now = HAL_GetTick();
    uint32_t time_in_state = now - fsm.state_entry_time;
    float total_accel = sqrtf(imu->accel_x * imu->accel_x + 
                               imu->accel_y * imu->accel_y + 
                               imu->accel_z * imu->accel_z);
    
    // Actualizar máximos
    if (baro->altitude > fsm.max_altitude) fsm.max_altitude = baro->altitude;
    if (baro->vspeed > fsm.max_speed) fsm.max_speed = baro->vspeed;
    if (total_accel > fsm.max_accel) fsm.max_accel = total_accel;
    
    switch (fsm.current_state) {
        
        case STATE_PAD_IDLE:
            if (recovery_is_armed()) {
                continuity_t cont = recovery_check_continuity();
                if (cont.channel1_ok && cont.channel2_ok) {
                    fsm_transition(STATE_ARMED);
                } else {
                    printf("[FSM] Cannot arm: continuity check failed\n");
                }
            }
            break;
            
        case STATE_ARMED:
            if (!recovery_is_armed()) {
                fsm_transition(STATE_PAD_IDLE);
                break;
            }
            // Detección de lanzamiento: accel > 2g sostenida 100ms
            if (total_accel > 2.0f * 9.81f) {
                fsm.launch_accel_count++;
                if (fsm.launch_accel_count >= 10) {  // 10 × 10ms = 100ms
                    fsm_transition(STATE_BOOST);
                }
            } else {
                fsm.launch_accel_count = 0;
            }
            break;
            
        case STATE_BOOST:
            // Motor burnout: accel cae a < 0.5g
            if (total_accel < 0.5f * 9.81f) {
                fsm_transition(STATE_COAST);
            }
            // Timeout backup
            if (time_in_state > fsm.boost_timeout_ms) {
                printf("[FSM] BOOST timeout! Forcing COAST\n");
                fsm_transition(STATE_COAST);
            }
            break;
            
        case STATE_COAST:
            // Detección de apogeo: vspeed cambia de + a -
            if (baro->vspeed < -0.5f) {
                fsm.apogee_vspeed_count++;
                if (fsm.apogee_vspeed_count >= 3) {  // 3 muestras
                    fsm_transition(STATE_APOGEE);
                }
            } else {
                fsm.apogee_vspeed_count = 0;
            }
            // Timeout backup
            if (time_in_state > fsm.coast_timeout_ms) {
                printf("[FSM] COAST timeout! Forcing APOGEE\n");
                fsm_transition(STATE_APOGEE);
            }
            break;
            
        case STATE_APOGEE:
            // Transición inmediata a descent
            fsm_transition(STATE_DESCENT_DROGUE);
            break;
            
        case STATE_DESCENT_DROGUE:
            // Detección de altitud de main deploy
            if (baro->altitude < fsm.main_deploy_alt) {
                fsm_transition(STATE_MAIN_DEPLOY);
            }
            // Timeout
            if (time_in_state > fsm.drogue_timeout_ms) {
                printf("[FSM] DROGUE DESCENT timeout! Forcing MAIN\n");
                fsm_transition(STATE_MAIN_DEPLOY);
            }
            break;
            
        case STATE_MAIN_DEPLOY:
            // Transición inmediata a descent main
            fsm_transition(STATE_DESCENT_MAIN);
            break;
            
        case STATE_DESCENT_MAIN:
            // Aterrizaje: velocidad vertical ≈ 0 durante 3 segundos
            if (fabsf(baro->vspeed) < 0.5f) {
                fsm.landed_still_count++;
                if (fsm.landed_still_count >= 300) {  // 300 × 10ms = 3s
                    fsm_transition(STATE_LANDED);
                }
            } else {
                fsm.landed_still_count = 0;
            }
            // Timeout
            if (time_in_state > fsm.main_timeout_ms) {
                printf("[FSM] MAIN DESCENT timeout! Forcing LANDED\n");
                fsm_transition(STATE_LANDED);
            }
            break;
            
        case STATE_LANDED:
            // Después de 10 segundos, entrar en recovery mode
            if (time_in_state > 10000) {
                fsm_transition(STATE_RECOVERY_MODE);
            }
            break;
            
        case STATE_RECOVERY_MODE:
            // Estado final - esperar recuperación
            // Buzzer intermitente + GPS tracking
            break;
    }
}
```

### Paso 6: Tarea RTOS
```c
void vTaskStateMachine(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  // 50 Hz
    
    fsm_init();
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        imu_data_t imu;
        baro_data_t baro;
        xQueuePeek(imu_data_q, &imu, 0);
        xQueuePeek(baro_data_q, &baro, 0);
        
        fsm_update(&imu, &baro);
    }
}
```

---

## Correlación con Datos de Vuelo Simulados

### Eventos Esperados con Perfil Nominal

| Tiempo (s) | Accel Z (g) | V. Vert. (m/s) | Altitud (m) | Estado Esperado   |
|------------|-------------|-----------------|-------------|-------------------|
| 0.0        | 1.0         | 0.0             | 0           | PAD_IDLE          |
| 0.5        | 1.0         | 0.0             | 0           | ARMED (manual)    |
| 2.0        | 3.5         | 12.0            | 6           | BOOST (t+100ms)   |
| 4.5        | 0.3         | 120.0           | 250         | COAST             |
| 12.0       | -1.0        | +0.2→-0.2       | 800         | APOGEE            |
| 12.1       | -1.0        | -2.0            | 799         | DESCENT_DROGUE    |
| 80.0       | 0.0         | -15.0           | 299         | MAIN_DEPLOY       |
| 80.1       | 0.0         | -15.0→-5.0      | 298         | DESCENT_MAIN      |
| 115.0      | 1.0         | 0.0             | 0           | LANDED            |
| 125.0      | 1.0         | 0.0             | 0           | RECOVERY_MODE     |

---

## API del Módulo

```c
// state_machine.h

void fsm_init(void);
void fsm_update(const imu_data_t *imu, const baro_data_t *baro);
flight_state_t fsm_get_state(void);
const char* fsm_get_state_name(void);
uint32_t fsm_get_time_in_state(void);

// Estadísticas de vuelo
float fsm_get_max_altitude(void);
float fsm_get_max_speed(void);
float fsm_get_max_accel(void);
uint32_t fsm_get_flight_duration(void);

// Configuración
void fsm_set_main_deploy_altitude(float alt_m);
void fsm_set_launch_threshold(float accel_g);
```

---

## Criterios de Aceptación

- [ ] Todas las transiciones ocurren correctamente con datos de vuelo simulados
- [ ] ARMED → BOOST solo con accel > 2g sostenida 100 ms
- [ ] COAST → APOGEE con latencia < 50 ms desde cruce de vspeed
- [ ] Drogue deploy event se genera en APOGEE
- [ ] Main deploy event se genera cuando altitude < 300 m
- [ ] LANDED detectado cuando vspeed ≈ 0 por 3 s
- [ ] Todos los timeouts de backup funcionan
- [ ] No hay transiciones falsas con datos ruidosos
- [ ] Logs de cada transición con timestamp
- [ ] LEDs de estado cambian correctamente

---

## Simulación en Proteus

### Escenarios de Test

| # | Escenario                        | Resultado Esperado                              |
|---|----------------------------------|-------------------------------------------------|
| 1 | Perfil nominal completo (CSV)    | Todas las transiciones en orden y a tiempo       |
| 2 | Apogeo no detectado              | Timeout a 15s fuerza APOGEE                     |
| 3 | Main alt no cruzada              | Timeout a 60s fuerza MAIN_DEPLOY                |
| 4 | Vibración en PAD (pico 2g)       | NO transiciona a BOOST (< 100ms)               |
| 5 | Desarmado durante ARMED          | Vuelve a PAD_IDLE                               |
| 6 | Boost largo (motor lento)        | Timeout a 5s fuerza COAST                       |
| 7 | Aterrizaje suave                 | LANDED después de 3s de quietud                 |

---

## Referencias

- [UML State Machine Diagrams](https://www.uml-diagrams.org/state-machine-diagrams.html)
- [Flight Computer State Machines](https://bps.space/documentation)
- [FSM Design Patterns for Embedded Systems](https://www.embedded.com/design-patterns-for-embedded-state-machines/)

---

*Módulo 11 - El cerebro racional del vuelo. Sabe exactamente qué está pasando y qué hacer.*
