# Módulo 09: Recovery System - Sistema de Recuperación

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 3 - Décimo módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core, 05-Barometer, 04-IMU, 11-State Machine  
> **Dependientes**: Ninguno (módulo terminal de seguridad)

---

## Resumen

Sistema de despliegue dual de paracaídas (drogue + main) mediante canales pirotécnicos controlados por MOSFETs. Es el módulo **más crítico de seguridad**: un fallo aquí significa pérdida del cohete o, peor, un cohete cayendo sin paracaídas. Implementa redundancia, safety interlocks, continuity checking y backup timers.

---

## Especificaciones Técnicas

### Canales Pirotécnicos
| Parámetro                | Canal 1 (Drogue) | Canal 2 (Main)  |
|--------------------------|-------------------|------------------|
| Pin GPIO                 | PC0               | PC1              |
| Función                  | Drogue chute      | Main chute       |
| Trigger condition        | Apogeo detectado  | Altitud < 300m   |
| Corriente de ignición    | 1-2 A (e-match)   | 1-2 A (e-match)  |
| Duración de pulso        | 2 segundos         | 2 segundos       |
| MOSFET                   | IRLZ44N (N-ch)    | IRLZ44N (N-ch)   |
| Voltaje de gate          | 3.3V (logic level)| 3.3V (logic level)|
| Protección flyback       | Diodo 1N4007       | Diodo 1N4007     |

### Arming Switch
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Pin GPIO                 | PD0 (input, pull-up interno)    |
| Tipo                     | Interruptor físico (a GND)      |
| Estado desarmado         | PD0 = HIGH (pull-up)            |
| Estado armado            | PD0 = LOW (switch cerrado)      |
| Debounce                 | 50 ms software                  |

### Continuity Check
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Pines ADC                | PD1 (canal 1), PD2 (canal 2)   |
| Circuito                 | Divisor resistivo + e-match     |
| E-match OK               | Resistencia < 5Ω → ADC alto    |
| E-match desconectado     | Resistencia ∞ → ADC bajo       |
| Frecuencia de check      | 1 Hz (solo en PAD_IDLE/ARMED)  |

---

## Decisiones Técnicas

### ¿Por qué dual deploy (drogue + main)?
- **Single deploy en apogeo**: Paracaídas grande - viento arrastra el cohete kilómetros
- **Dual deploy**: 
  - Drogue pequeño en apogeo → descenso rápido pero controlado (~15 m/s)
  - Main grande a 300m AGL → descenso suave (~5 m/s), aterriza cerca
- **Ventaja**: Recuperación más fácil, menor daño por viento

### ¿Por qué IRLZ44N y no un MOSFET más pequeño?
- IRLZ44N: **logic-level gate** - se activa completamente con 3.3V
- RDS(on) = 0.022Ω → disipa apenas 44 mW con 2A de e-match
- ID máx = 47A → margen enorme para 1-2A de e-match
- SOA (Safe Operating Area) muy amplio
- Alternativa: 2N7000 - solo 200mA, insuficiente para e-match

### ¿Por qué 2 segundos de pulso?
- E-matches típicos ignicionan en 10-50 ms
- 2 segundos da margen masivo para asegurar la ignición
- Si el primer intento falla, los 2 segundos permiten re-intento térmico
- Después de 2s: MOSFET OFF automáticamente (seguridad)

### Redundancia de Detección de Apogeo
```
Método 1 (primario): Barómetro - vspeed cambia de signo
Método 2 (secundario): IMU - aceleración cae a ~-1g (caída libre)
Método 3 (backup timer): Si Método 1 y 2 fallan, deploy automático 
                          N segundos después de BOOST

Lógica: deploy si (Método 1 OR Método 2)
Backup: deploy siempre a T_boost + T_max_coast (timeout de seguridad)
```

### ¿Por qué arming switch hardware?
- **Safety**: No se puede armar solo por software - requiere acción física
- **Regulación**: Muchas competiciones de cohetes exigen arming switch accesible
- **Prevención de accidentes**: Un bug de software no puede activar pyros accidentalmente si no está armado
- En simulación: se reemplaza por un botón en Proteus

---

## Diseño del Circuito

### Esquemático - Canal Pirotécnico

```
                    Battery 5V (Rail servo)
                         │
                    ┌────┴────┐
                    │ E-match │ (simulado con resistencia + LED)
                    │  1-5Ω   │
                    └────┬────┘
                         │
                    ┌────┴────┐
                    │ 1N4007  │ (flyback diode, protección inductiva)
                    └────┬────┘
                         │
              ┌──────────┴──────────┐
              │     IRLZ44N         │
    PC0 ──────┤ Gate   Drain        │
              │        Source ──── GND
    10kΩ ─────┤ Gate ──── GND       │ (pull-down: desarmado por default)
              └─────────────────────┘

    Continuity check:
    5V ── 10kΩ ── ● ── E-match ── GND
                  │
                  └── PD1 (ADC / digital input)
                      Si e-match OK: V ≈ 4.5V (low R divider)
                      Si e-match NO: V ≈ 5V (open circuit, todo en pull-up)
```

### Esquemático - Arming Switch

```
    3.3V
     │
   10kΩ (pull-up interno STM32)
     │
    PD0 ──── Switch ──── GND
     │
   (Desarmado: PD0 = HIGH)
   (Armado: PD0 = LOW, switch cerrado)
```

### Componentes

| Ref  | Componente     | Valor/Tipo     | Notas                          |
|------|----------------|----------------|--------------------------------|
| Q1   | IRLZ44N        | TO-220         | MOSFET canal drogue            |
| Q2   | IRLZ44N        | TO-220         | MOSFET canal main              |
| D3   | 1N4007         | Axial          | Flyback protection drogue      |
| D4   | 1N4007         | Axial          | Flyback protection main        |
| R5   | 10kΩ           | 1/4W           | Pull-down gate Q1              |
| R6   | 10kΩ           | 1/4W           | Pull-down gate Q2              |
| R7   | 10kΩ           | 1/4W           | Continuity pull-up CH1         |
| R8   | 10kΩ           | 1/4W           | Continuity pull-up CH2         |
| SW2  | Switch         | Momentary      | Arming switch (toggle en real)  |
| LED2 | LED rojo       | 3mm            | Simula e-match drogue           |
| LED3 | LED rojo       | 3mm            | Simula e-match main             |
| R9   | 4.7Ω           | 1W             | Simula resistencia e-match      |
| R10  | 4.7Ω           | 1W             | Simula resistencia e-match      |

---

## Desarrollo Step-by-Step

### Paso 1: GPIO Setup
```c
// Pyro outputs (PC0, PC1) - iniciar como LOW (desarmado)
void recovery_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Pyro channels: output, push-pull, initially LOW
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    
    // Arming switch: input, pull-up
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    printf("[RECOVERY] GPIO initialized, pyro channels OFF\n");
}
```

### Paso 2: Arming Logic
```c
typedef enum {
    RECOVERY_DISARMED,
    RECOVERY_ARMED,
    RECOVERY_FIRED_DROGUE,
    RECOVERY_FIRED_MAIN,
    RECOVERY_COMPLETE
} recovery_state_t;

static recovery_state_t recovery_state = RECOVERY_DISARMED;

bool recovery_is_armed(void) {
    // Debounce: leer 3 veces con 10ms entre lecturas
    uint8_t count = 0;
    for (int i = 0; i < 3; i++) {
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET) {
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (count >= 2);  // 2 de 3 = armado
}
```

### Paso 3: Continuity Check
```c
typedef struct {
    bool channel1_ok;  // Drogue e-match connected
    bool channel2_ok;  // Main e-match connected
} continuity_t;

continuity_t recovery_check_continuity(void) {
    continuity_t cont;
    
    // Leer ADC o digital input para continuidad
    // Si e-match está conectado: voltaje cae por divisor resistivo
    // Simplificado: leer como digital (HIGH = desconectado, LOW = conectado)
    cont.channel1_ok = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET);
    cont.channel2_ok = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET);
    
    return cont;
}
```

### Paso 4: Funciones de Deploy
```c
// Deploy drogue (canal 1)
void recovery_fire_drogue(void) {
    if (recovery_state != RECOVERY_ARMED) {
        printf("[RECOVERY] BLOCKED: system not armed!\n");
        return;
    }
    
    printf("[RECOVERY] *** DROGUE DEPLOY *** at t=%lu ms\n", HAL_GetTick());
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   // FIRE
    vTaskDelay(pdMS_TO_TICKS(2000));                        // 2 second pulse
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // OFF
    
    recovery_state = RECOVERY_FIRED_DROGUE;
    logger_mark_event(LOG_FLAG_DROGUE);
}

// Deploy main (canal 2)
void recovery_fire_main(void) {
    if (recovery_state < RECOVERY_FIRED_DROGUE) {
        printf("[RECOVERY] BLOCKED: drogue not fired yet!\n");
        return;
    }
    
    printf("[RECOVERY] *** MAIN DEPLOY *** at alt=%.1fm, t=%lu ms\n", 
           baro_get_altitude(), HAL_GetTick());
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2000));
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    
    recovery_state = RECOVERY_FIRED_MAIN;
    logger_mark_event(LOG_FLAG_MAIN);
}
```

### Paso 5: Backup Timer
```c
static TimerHandle_t backup_drogue_timer;
static TimerHandle_t backup_main_timer;

// Se inicia cuando se detecta BOOST
void recovery_start_backup_timers(void) {
    // Backup drogue: Tboost_max + Tcoast_max = 3s + 12s = 15s
    backup_drogue_timer = xTimerCreate("BkpDrogue", 
                                         pdMS_TO_TICKS(15000),
                                         pdFALSE,  // One-shot
                                         NULL,
                                         backup_drogue_callback);
    xTimerStart(backup_drogue_timer, 0);
    
    // Backup main: 60s después del launch (si drogue falló, al menos main deploy)
    backup_main_timer = xTimerCreate("BkpMain",
                                       pdMS_TO_TICKS(60000),
                                       pdFALSE,
                                       NULL,
                                       backup_main_callback);
    xTimerStart(backup_main_timer, 0);
}

void backup_drogue_callback(TimerHandle_t xTimer) {
    if (recovery_state == RECOVERY_ARMED) {
        printf("[RECOVERY] BACKUP TIMER: Forcing drogue deploy!\n");
        recovery_fire_drogue();
    }
}

void backup_main_callback(TimerHandle_t xTimer) {
    if (recovery_state == RECOVERY_FIRED_DROGUE) {
        printf("[RECOVERY] BACKUP TIMER: Forcing main deploy!\n");
        recovery_fire_main();
    }
}
```

### Paso 6: Tarea RTOS
```c
void vTaskRecovery(void *pvParameters) {
    recovery_gpio_init();
    
    for (;;) {
        // Esperar evento de deploy
        EventBits_t bits = xEventGroupWaitBits(
            flight_events,
            EVT_DEPLOY_DROGUE | EVT_DEPLOY_MAIN,
            pdTRUE,   // Clear bits after reading
            pdFALSE,  // Wait for ANY bit
            pdMS_TO_TICKS(100)  // Timeout 100ms (para health checks)
        );
        
        if (bits & EVT_DEPLOY_DROGUE) {
            recovery_fire_drogue();
        }
        
        if (bits & EVT_DEPLOY_MAIN) {
            recovery_fire_main();
        }
        
        // Health check periódico (cuando no hay eventos)
        if (recovery_state == RECOVERY_DISARMED || 
            recovery_state == RECOVERY_ARMED) {
            continuity_t cont = recovery_check_continuity();
            if (!cont.channel1_ok) {
                printf("[RECOVERY] WARNING: Drogue e-match not connected!\n");
            }
            if (!cont.channel2_ok) {
                printf("[RECOVERY] WARNING: Main e-match not connected!\n");
            }
        }
    }
}
```

---

## Secuencia Completa de Recuperación

```
Timeline del vuelo:

t=0     PAD_IDLE → Continuity check: "Drogue OK, Main OK"
t=1     ARM switch ON → ARMED → "System armed, ready for launch"
t=2     LAUNCH detected → BOOST → Start backup timers
t=4.5   Burnout → COAST
t=12    APOGEE detected (baro vspeed < 0) → EVT_DEPLOY_DROGUE
t=12.01 *** DROGUE FIRE *** (PC0 = HIGH)
t=14.01 DROGUE pulse complete (PC0 = LOW)
t=12-80 DESCENT_DROGUE (~15 m/s)
t=80    Altitude < 300m → EVT_DEPLOY_MAIN
t=80.01 *** MAIN FIRE *** (PC1 = HIGH)
t=82.01 MAIN pulse complete (PC1 = LOW)
t=80-115 DESCENT_MAIN (~5 m/s)
t=115   LANDED (vspeed ≈ 0 for 3s)
t=125   RECOVERY_MODE → Buzzer + GPS tracking
```

---

## API del Módulo

```c
// recovery_system.h

// Inicialización
void recovery_init(void);

// Estado
recovery_state_t recovery_get_state(void);
bool recovery_is_armed(void);
continuity_t recovery_check_continuity(void);

// Deploy (llamado por State Machine)
void recovery_fire_drogue(void);
void recovery_fire_main(void);

// Backup timers
void recovery_start_backup_timers(void);
void recovery_cancel_backup_timers(void);

// Emergency
void recovery_emergency_deploy_all(void);  // Fire both channels immediately
```

---

## Criterios de Aceptación

- [ ] Pyro channels OFF al arrancar (pull-down hardware verificado)
- [ ] Arming switch: no se puede fire sin armar primero
- [ ] Continuity check detecta e-match presente/ausente
- [ ] Drogue deploys en apogeo (latencia < 50 ms desde detección)
- [ ] Main deploys al cruzar altitud configurada (300m ±5m)
- [ ] Pulso de 2 segundos exactos en cada canal
- [ ] Backup timer drogue fire si detección primaria falla
- [ ] Backup timer main fire si detección primaria falla
- [ ] Emergency deploy all funciona
- [ ] No hay activación accidental en ningún estado (test de seguridad)
- [ ] Logs detallados de cada acción de recovery

---

## Simulación en Proteus

### Escenarios de Test

| # | Escenario                        | Resultado Esperado                              |
|---|----------------------------------|-------------------------------------------------|
| 1 | Boot sin arming switch           | Pyros desactivados, "DISARMED" en logs          |
| 2 | Armar sistema                    | Continuity check, "ARMED" en logs               |
| 3 | Apogeo normal                    | Drogue LED enciende 2s, log correcto            |
| 4 | Altitud < 300m                   | Main LED enciende 2s, log correcto              |
| 5 | Fallo de detección de apogeo     | Backup timer fire drogue a T+15s                |
| 6 | E-match desconectado             | Warning en logs, continuity check falla         |
| 7 | Intento de fire sin armar        | BLOCKED en logs, pyro permanece OFF             |
| 8 | Batería crítica en vuelo         | Emergency deploy all                            |

---

## Referencias

- [IRLZ44N Datasheet](https://www.infineon.com/dgdl/irlz44npbf.pdf)
- [NAR Safety Code](https://www.nar.org/safety-information/high-power-rocket-safety-code/)
- [E-match Specifications](https://www.apogeerockets.com/Building_Supplies/Launch_Accessories/Electric-Matches)

---

*Módulo 09 - El salvavidas. Sin él, el cohete no vuelve a casa.*
