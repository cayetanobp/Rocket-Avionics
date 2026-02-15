# Módulo 12: Diagnostics & Debug - Diagnósticos y Depuración

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Tercer módulo a desarrollar (debug temprano)  
> **Dependencias**: 01-Power, 02-MCU Core  
> **Dependientes**: Todos los módulos reportan a Diagnostics

---

## Resumen

Sistema centralizado de logging, health monitoring y debug. Proporciona la infraestructura de observabilidad del sistema: logs categorizados por severidad con timestamps, health checks periódicos de todos los subsistemas, indicadores LED de estado, buzzer de alarma y watchdog. Es el **tercer módulo a implementar** porque sin diagnósticos, debuggear los demás módulos es extremadamente difícil.

---

## Especificaciones Técnicas

### UART Debug
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Periférico         | UART2 (PA2=TX, PA3=RX)         |
| Baud rate          | 115200                          |
| Configuración      | 8N1                             |
| Dirección          | TX (principalmente), RX (comandos)|
| Conexión Proteus   | Virtual Terminal                 |

### LEDs de Estado
| LED    | Pin  | Color   | Función                          |
|--------|------|---------|----------------------------------|
| LED1   | PC2  | Verde   | Heartbeat (500ms toggle)         |
| LED2   | PC3  | Rojo    | Error/alarma activa              |
| LED3   | PC4  | Verde   | Status G (componente RGB)        |
| LED4   | PC5  | Azul    | Status B (componente RGB)        |

### Buzzer
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Pin                | PC6 (GPIO output o PWM)         |
| Tipo               | Activo (3.3V drive)             |
| Patrones           | Beep corto (info), largo (warn), continuo (error) |
| En RECOVERY_MODE   | Intermitente 1s ON / 1s OFF     |

### Watchdog (IWDG)
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Tipo               | Independent Watchdog (IWDG)     |
| Timeout            | 1000 ms                         |
| Refresh location   | En tarea de diagnóstico o idle hook |
| Acción al timeout  | Reset completo del MCU           |

---

## Decisiones Técnicas

### ¿Por qué desarrollar diagnósticos tan temprano (3er módulo)?
- Sin logs → debuggear I2C, SPI, sensores es ciego
- Sin health checks → no sabemos si un sensor falló hasta que el cohete cae
- Sin heartbeat → no sabemos si el MCU está corriendo o colgado
- **Inversión temprana**: 2 días de desarrollo ahora ahorran semanas de debugging después

### ¿Por qué printf sobre UART y no SWD/ITM trace?
- **printf/UART**: Visible en Virtual Terminal de Proteus, fácil de leer
- **ITM Trace**: Más eficiente pero requiere configuración de Proteus y herramientas
- **Trade-off**: printf usa ~5% CPU por log - aceptable para simulación
- En producción: se reemplazaría por ITM o se reduciría el nivel de log

### ¿Por qué IWDG y no WWDG?
- **IWDG (Independent)**: Clock independiente del sistema (LSI ~32 kHz), funciona incluso si el clock principal falla
- **WWDG (Window)**: Más preciso pero depende del clock APB1
- En un cohete, si el oscilador principal falla, queremos que el watchdog siga funcionando → IWDG

### Sistema de Log Levels
```c
// Solo se imprimen logs con nivel ≥ al nivel configurado
// Default: LOG_INFO en simulación, LOG_WARN en vuelo
typedef enum {
    LOG_DEBUG = 0,    // Detalles extremos (solo desarrollo)
    LOG_INFO  = 1,    // Información general
    LOG_WARN  = 2,    // Advertencias (degradación acceptable)
    LOG_ERROR = 3,    // Errores (algo falló, recovery automático)
    LOG_CRITICAL = 4  // Crítico (acción de seguridad requerida)
} log_level_t;
```

---

## Desarrollo Step-by-Step

### Paso 1: Sistema de Logging
```c
// diagnostics.h

#include <stdio.h>
#include <stdarg.h>

static log_level_t current_log_level = LOG_INFO;
static uint32_t error_count = 0;
static uint32_t warning_count = 0;

void diag_log(log_level_t level, const char *module, const char *fmt, ...) {
    if (level < current_log_level) return;
    
    static const char *level_tags[] = {
        "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"
    };
    
    // Timestamp + level + module + message
    printf("[%lu][%s][%s] ", HAL_GetTick(), level_tags[level], module);
    
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    
    printf("\n");
    
    // Contadores
    if (level == LOG_WARN) warning_count++;
    if (level >= LOG_ERROR) error_count++;
    
    // Acciones automáticas por nivel
    if (level == LOG_ERROR) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  // LED rojo ON
    }
    if (level == LOG_CRITICAL) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        diag_buzzer_pattern(BUZZER_CONTINUOUS);
    }
}

// Macros de conveniencia
#define LOG_D(module, fmt, ...) diag_log(LOG_DEBUG, module, fmt, ##__VA_ARGS__)
#define LOG_I(module, fmt, ...) diag_log(LOG_INFO, module, fmt, ##__VA_ARGS__)
#define LOG_W(module, fmt, ...) diag_log(LOG_WARN, module, fmt, ##__VA_ARGS__)
#define LOG_E(module, fmt, ...) diag_log(LOG_ERROR, module, fmt, ##__VA_ARGS__)
#define LOG_C(module, fmt, ...) diag_log(LOG_CRITICAL, module, fmt, ##__VA_ARGS__)
```

### Paso 2: Uso en Otros Módulos
```c
// Ejemplo de uso en el módulo IMU:
void mpu6050_init(void) {
    uint8_t who_am_i = mpu6050_read_reg(0x75);
    if (who_am_i == 0x68) {
        LOG_I("IMU", "MPU6050 detected, WHO_AM_I=0x%02X", who_am_i);
    } else {
        LOG_E("IMU", "MPU6050 not found! WHO_AM_I=0x%02X (expected 0x68)", who_am_i);
    }
}

// Ejemplo en Power Management:
if (voltage < 9.0f) {
    LOG_C("PWR", "Battery CRITICAL: %.2fV - Emergency procedures!", voltage);
}
```

### Paso 3: Health Checks
```c
typedef struct {
    bool imu_ok;
    bool baro_ok;
    bool gps_fix;
    bool flash_ok;
    bool radio_ok;
    bool battery_ok;
    float battery_voltage;
    uint8_t gps_satellites;
    uint32_t flash_bytes_free;
    uint32_t uptime_ms;
    uint32_t error_count;
    uint32_t warning_count;
    float cpu_usage_pct;
} system_health_t;

system_health_t diag_run_health_check(void) {
    system_health_t health;
    
    // IMU: intentar leer WHO_AM_I
    health.imu_ok = sensors_is_device_ready(0x68);
    
    // Barómetro: intentar leer chip_id
    health.baro_ok = sensors_is_device_ready(0x76);
    
    // GPS: tiene fix?
    gps_data_t gps = gps_get_data();
    health.gps_fix = (gps.fix > 0);
    health.gps_satellites = gps.satellites;
    
    // Flash: leer ID
    health.flash_ok = (flash_read_id() >> 16) == 0xEF;
    health.flash_bytes_free = logger_get_bytes_free();
    
    // Radio: leer version register
    health.radio_ok = (lora_read_reg(0x42) == 0x12);
    
    // Batería
    power_data_t pwr = power_read();
    health.battery_voltage = pwr.bus_voltage;
    health.battery_ok = (pwr.bus_voltage > 10.0f);
    
    // Sistema
    health.uptime_ms = HAL_GetTick();
    health.error_count = error_count;
    health.warning_count = warning_count;
    
    return health;
}

void diag_print_health(const system_health_t *h) {
    printf("\n╔══════════════════════════════════════╗\n");
    printf("║       SYSTEM HEALTH REPORT           ║\n");
    printf("╠══════════════════════════════════════╣\n");
    printf("║ Uptime:      %10lu ms           ║\n", h->uptime_ms);
    printf("║ IMU:         %s                    ║\n", h->imu_ok ? "OK" : "FAIL");
    printf("║ Barometer:   %s                    ║\n", h->baro_ok ? "OK" : "FAIL");
    printf("║ GPS:         %s (Sats: %2d)        ║\n", h->gps_fix ? "FIX" : "NO FIX", h->gps_satellites);
    printf("║ Flash:       %s (%lu KB free)      ║\n", h->flash_ok ? "OK" : "FAIL", h->flash_bytes_free/1024);
    printf("║ Radio:       %s                    ║\n", h->radio_ok ? "OK" : "FAIL");
    printf("║ Battery:     %.2fV %s             ║\n", h->battery_voltage, h->battery_ok ? "OK" : "LOW!");
    printf("║ Errors:      %lu                    ║\n", h->error_count);
    printf("║ Warnings:    %lu                    ║\n", h->warning_count);
    printf("╚══════════════════════════════════════╝\n\n");
}
```

### Paso 4: Heartbeat LED
```c
// Simple LED toggle en tarea de diagnóstico
static void diag_heartbeat(void) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
}
```

### Paso 5: Buzzer Patterns
```c
typedef enum {
    BUZZER_OFF,
    BUZZER_BEEP_SHORT,    // 100ms beep
    BUZZER_BEEP_LONG,     // 500ms beep
    BUZZER_CONTINUOUS,    // ON permanente
    BUZZER_RECOVERY       // 1s ON, 1s OFF (para localización)
} buzzer_pattern_t;

static buzzer_pattern_t current_buzzer_pattern = BUZZER_OFF;

void diag_buzzer_update(void) {
    static uint32_t buzzer_timer = 0;
    uint32_t now = HAL_GetTick();
    
    switch (current_buzzer_pattern) {
        case BUZZER_OFF:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
            break;
        case BUZZER_BEEP_SHORT:
            if (now - buzzer_timer < 100)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
            else {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
                current_buzzer_pattern = BUZZER_OFF;
            }
            break;
        case BUZZER_CONTINUOUS:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
            break;
        case BUZZER_RECOVERY:
            // 1s ON, 1s OFF
            if ((now / 1000) % 2 == 0)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

void diag_buzzer_pattern(buzzer_pattern_t pattern) {
    current_buzzer_pattern = pattern;
}
```

### Paso 6: Watchdog
```c
void diag_watchdog_init(void) {
    // IWDG con timeout de ~1s
    // LSI = ~32 kHz
    // Prescaler = 64 → 32000/64 = 500 Hz
    // Reload = 500 → 500/500 = 1s timeout
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 500;
    HAL_IWDG_Init(&hiwdg);
    
    LOG_I("DIAG", "Watchdog initialized, timeout=1s");
}

void diag_watchdog_refresh(void) {
    HAL_IWDG_Refresh(&hiwdg);
}
```

### Paso 7: Stack Overflow Hook (FreeRTOS)
```c
// FreeRTOS callback: llamado cuando se detecta stack overflow
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // CRITICAL: No usar printf aquí (podría requerir stack)
    // Encender LED rojo directamente
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    
    // Intentar logging mínimo
    // En la práctica, aquí se podría escribir a un registro de crash dump
    printf("[CRITICAL] Stack overflow in task: %s\n", pcTaskName);
    
    // Reset
    NVIC_SystemReset();
}
```

### Paso 8: Tarea RTOS
```c
void vTaskDiagnostics(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);  // 1 Hz
    
    diag_watchdog_init();
    LOG_I("DIAG", "Diagnostics task started");
    
    // Boot health check
    system_health_t health = diag_run_health_check();
    diag_print_health(&health);
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        // Heartbeat
        diag_heartbeat();
        
        // Watchdog refresh
        diag_watchdog_refresh();
        
        // Health check periódico
        health = diag_run_health_check();
        
        // Alertas por estado
        if (!health.imu_ok) LOG_E("DIAG", "IMU not responding!");
        if (!health.baro_ok) LOG_E("DIAG", "Barometer not responding!");
        if (!health.battery_ok) LOG_W("DIAG", "Battery low: %.2fV", health.battery_voltage);
        if (health.flash_bytes_free < 1024*100) LOG_W("DIAG", "Flash almost full! %lu KB free", health.flash_bytes_free/1024);
        
        // Buzzer update
        diag_buzzer_update();
        
        // Report resumido cada 10s
        static uint8_t report_counter = 0;
        if (++report_counter >= 10) {
            report_counter = 0;
            printf("[DIAG] Uptime=%lus State=%s Errs=%lu Bat=%.1fV\n",
                   health.uptime_ms / 1000,
                   fsm_get_state_name(),
                   health.error_count,
                   health.battery_voltage);
        }
    }
}
```

### Paso 9: CLI de Debug (Opcional, para Simulación)
```c
// Recibir comandos por UART para testing
// Ejemplo de comandos:
//   "status"  → imprimir health report
//   "arm"     → simular arming switch
//   "launch"  → inyectar aceleración simulada
//   "reset"   → reset del MCU

void diag_process_command(const char *cmd) {
    if (strcmp(cmd, "status") == 0) {
        system_health_t h = diag_run_health_check();
        diag_print_health(&h);
    } else if (strcmp(cmd, "errors") == 0) {
        printf("[DIAG] Total errors: %lu, warnings: %lu\n", 
               error_count, warning_count);
    } else if (strcmp(cmd, "reset") == 0) {
        printf("[DIAG] Resetting MCU...\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    } else if (strcmp(cmd, "loglevel") == 0) {
        // Cycle through log levels
        current_log_level = (current_log_level + 1) % 5;
        printf("[DIAG] Log level set to %d\n", current_log_level);
    } else {
        printf("[DIAG] Unknown command: %s\n", cmd);
    }
}
```

---

## Boot Sequence Log Esperado

```
[0][INFO][SYS] ============================
[0][INFO][SYS] Rocket Avionics - Boot v1.0
[0][INFO][SYS] STM32F407 @ 168 MHz
[0][INFO][SYS] FreeRTOS v10.4
[0][INFO][SYS] ============================
[1][INFO][PWR] INA219 initialized, Vbat=11.82V
[2][INFO][DIAG] Watchdog initialized, timeout=1s
[3][INFO][DIAG] Diagnostics task started
[5][INFO][SEN] I2C scan: found 3 devices
[5][INFO][SEN]   0x40 (INA219)
[5][INFO][SEN]   0x68 (MPU6050)
[5][INFO][SEN]   0x76 (BMP280)
[10][INFO][IMU] MPU6050 initialized, WHO_AM_I=0x68
[15][INFO][IMU] Calibration started (1000 samples)...
[25][INFO][IMU] Calibration done. Offsets: ax=12 ay=-8 az=45 gx=3 gy=-2 gz=1
[30][INFO][BARO] BMP280 initialized, chip_id=0x58
[31][INFO][BARO] Ground pressure calibrated: 101325.00 Pa
[35][INFO][GPS] UART DMA started, waiting for fix...
[40][INFO][LOG] W25Q128 detected, ID=0xEF4018
[45][INFO][TELEM] LoRa initialized: 433MHz, SF7, BW125, +17dBm
[50][INFO][CTRL] Flight control initialized, servos neutral
[51][INFO][RECOV] GPIO initialized, pyro channels OFF
[55][INFO][FSM] Initialized → PAD_IDLE

╔══════════════════════════════════════╗
║       SYSTEM HEALTH REPORT           ║
╠══════════════════════════════════════╣
║ Uptime:             55 ms           ║
║ IMU:         OK                      ║
║ Barometer:   OK                      ║
║ GPS:         NO FIX (Sats:  0)       ║
║ Flash:       OK (16384 KB free)      ║
║ Radio:       OK                      ║
║ Battery:     11.82V OK               ║
║ Errors:      0                       ║
║ Warnings:    0                       ║
╚══════════════════════════════════════╝

[56][INFO][SYS] All systems GO. Waiting for arming...
```

---

## API del Módulo

```c
// diagnostics.h

// Logging
void diag_log(log_level_t level, const char *module, const char *fmt, ...);
void diag_set_log_level(log_level_t level);

// Health
system_health_t diag_run_health_check(void);
void diag_print_health(const system_health_t *health);

// Indicadores
void diag_buzzer_pattern(buzzer_pattern_t pattern);
void diag_set_status_led(uint8_t r, uint8_t g, uint8_t b);

// Watchdog
void diag_watchdog_init(void);
void diag_watchdog_refresh(void);

// Estadísticas
uint32_t diag_get_error_count(void);
uint32_t diag_get_warning_count(void);
uint32_t diag_get_uptime_ms(void);
```

---

## Criterios de Aceptación

- [ ] Logs se imprimen correctamente en Virtual Terminal con formato esperado
- [ ] Filtro de log levels funciona (DEBUG no se imprime si level=INFO)
- [ ] Health check detecta correctamente sensores presentes/ausentes
- [ ] LED heartbeat parpadea a 1 Hz estable
- [ ] LED de error se enciende ante LOG_ERROR
- [ ] Buzzer suena con patrones correctos
- [ ] Watchdog reinicia MCU si tarea se bloquea > 1s
- [ ] Stack overflow hook se ejecuta ante overflow forzado
- [ ] Boot sequence log es completo y legible
- [ ] Error count y warning count son precisos

---

## Referencias

- [STM32 IWDG Configuration](https://www.st.com/resource/en/application_note/dm00042534.pdf)
- [FreeRTOS Debug & Trace](https://www.freertos.org/rtos-trace-macros.html)
- [Embedded Logging Best Practices](https://interrupt.memfault.com/blog/logging-best-practices)

---

*Módulo 12 - El médico del sistema. Diagnostica, reporta y mantiene todo funcionando.*
