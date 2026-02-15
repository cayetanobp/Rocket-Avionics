# Módulo 10: Flight Control - Control de Vuelo Activo

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 3 - Último módulo funcional (avanzado/opcional)  
> **Dependencias**: 01-Power, 02-MCU Core, 04-IMU Navigation  
> **Dependientes**: Ninguno (módulo opcional avanzado)

---

## Resumen

Sistema de estabilización activa del cohete mediante superficies de control (canards o alerones) accionadas por servos. Implementa un controlador PID discreto que usa la orientación de la IMU como feedback para mantener el cohete vertical durante las fases BOOST y COAST. Este es el módulo más avanzado y **opcional** - un cohete puede volar sin control activo si es aerodinámicamente estable.

---

## Especificaciones Técnicas

### Servos
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Cantidad                 | 4 (configuración X o +)         |
| Tipo                     | Micro servo (SG90 o similar)    |
| Voltaje                  | 5V (desde rail de potencia)     |
| Señal PWM                | 50 Hz, pulso 1000-2000 µs      |
| Rango de movimiento      | 0° - 180° (usamos ±15°)        |
| Torque                   | 1.8 kgf·cm @ 4.8V              |
| Velocidad                | 0.12 s/60° (suficiente)         |
| Pines PWM                | TIM2 CH1-CH4 (PA0, PA1, PB10, PB11)|

### Controlador PID
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Loop rate                | 50 Hz (cada 20 ms)             |
| Ejes controlados         | Pitch, Yaw (2 PIDs independientes)|
| Roll                     | No controlado (rotación axial aceptable)|
| Input                    | Error de orientación (°) vs vertical |
| Output                   | Deflexión de servos (°)         |
| Saturación               | ±15° (protección mecánica)      |

### Ganancias PID Iniciales (tuning requerido)
| Ganancia | Valor Inicial | Descripción                     |
|----------|---------------|---------------------------------|
| Kp       | 2.0           | Respuesta proporcional al error |
| Ki       | 0.1           | Elimina error estacionario      |
| Kd       | 0.5           | Amortiguación de oscilaciones   |
| dt       | 0.02 s        | Período de ejecución (50 Hz)    |

---

## Decisiones Técnicas

### ¿Por qué PID y no algo más avanzado (LQR, MPC)?
| Criterio              | PID              | LQR              | MPC              |
|-----------------------|------------------|-------------------|-------------------|
| Complejidad           | Baja             | Media (matrices)  | Alta (optimización)|
| Tuning                | 3 params         | Q, R matrices     | Muchos params     |
| CPU necesario         | ~0.05 ms         | ~0.5 ms           | ~5 ms             |
| Fácil de entender     | ✅               | ⚠️               | ❌               |
| Suficiente para esto  | ✅               | ✅               | Overkill          |

**Decisión**: PID - suficiente para estabilización de un cohete a baja velocidad/altitud. La no-linealidad del sistema aéreo se maneja con anti-windup y saturación.

### ¿Por qué 50 Hz y no 100 Hz?
- Los servos aceptan señales PWM a 50 Hz - no pueden responder más rápido
- Actualizar el PID a 100 Hz generaría comandos que el servo no puede seguir
- 50 Hz da 20 ms por ciclo - tiempo suficiente para cálculo PID + PWM update
- La IMU a 100 Hz sobremuestra el PID → los datos están siempre frescos

### ¿Por qué no controlar Roll?
- Un cohete aerodinámicamente estable tiene spin natural por ángulo de aletas fijas
- El spin no afecta la trayectoria - solo la orientación axial
- Controlar roll requiere un servo dedicado o mixing complejo
- En cohetes reales, el roll-rate es incluso deseable (estabilización giroscópica)

### ¿Cuándo está activo el control?
```
PAD_IDLE: Servos en posición neutral (1500 µs)
ARMED:    Servos en posición neutral
BOOST:    PID ACTIVO - estabilización contra viento/thrust offset
COAST:    PID ACTIVO - mantener orientación para apogeo predecible
APOGEE+:  PID OFF - servos neutros (no hay efecto aerodinámico en descenso lento)
```

---

## Diseño del Circuito

### Conexión de Servos

```
5V Rail (Buck converter)
  │
  ├── Servo 1 (Canard +X) ── Signal → PA0 (TIM2_CH1)
  ├── Servo 2 (Canard -X) ── Signal → PA1 (TIM2_CH2)
  ├── Servo 3 (Canard +Y) ── Signal → PB10 (TIM2_CH3)
  └── Servo 4 (Canard -Y) ── Signal → PB11 (TIM2_CH4)
      │
      └── GND (común)
```

### Configuración PWM (TIM2)
```c
// TIM2 a 50 Hz:
// APB1 = 42 MHz × 2 (timer multiplier) = 84 MHz
// Prescaler = 84 → Timer clock = 1 MHz
// ARR = 20000 → Período = 20 ms = 50 Hz
// CCR = 1000-2000 → Pulso de 1-2 ms

#define PWM_FREQ_HZ     50
#define PWM_PRESCALER    84     // 84 MHz / 84 = 1 MHz
#define PWM_PERIOD       20000  // 1 MHz / 20000 = 50 Hz
#define SERVO_MIN_US     1000   // 0° / full deflection left
#define SERVO_CENTER_US  1500   // 90° / neutral
#define SERVO_MAX_US     2000   // 180° / full deflection right
#define SERVO_RANGE_DEG  30.0f  // ±15° useful range
```

---

## Desarrollo Step-by-Step

### Paso 1: PWM Setup
```c
void flight_control_init(void) {
    // TIM2 ya configurado por CubeMX
    // Iniciar todos los canales PWM en posición neutral
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, SERVO_CENTER_US);
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    
    printf("[CTRL] Flight control initialized, servos neutral\n");
}
```

### Paso 2: Mapeo Ángulo → PWM
```c
uint16_t servo_angle_to_pwm(float angle_deg) {
    // angle_deg: -15 a +15 (rango útil)
    // Clamp
    if (angle_deg < -15.0f) angle_deg = -15.0f;
    if (angle_deg > 15.0f) angle_deg = 15.0f;
    
    // Mapear: -15° → 1000µs, 0° → 1500µs, +15° → 2000µs
    float us = SERVO_CENTER_US + (angle_deg / 15.0f) * 500.0f;
    return (uint16_t)us;
}

void servo_set_deflection(uint8_t channel, float angle_deg) {
    uint16_t pwm = servo_angle_to_pwm(angle_deg);
    switch (channel) {
        case 0: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm); break;
        case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm); break;
        case 3: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm); break;
    }
}
```

### Paso 3: Controlador PID
```c
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output_min, output_max;
    float integral_min, integral_max;  // Anti-windup
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -15.0f;  // Max deflection °
    pid->output_max =  15.0f;
    pid->integral_min = -50.0f;  // Anti-windup limits
    pid->integral_max =  50.0f;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Proportional
    float P = pid->Kp * error;
    
    // Integral with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    float I = pid->Ki * pid->integral;
    
    // Derivative (filtered)
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;
    
    // Total output with saturation
    float output = P + I + D;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
```

### Paso 4: Mixing de Canards (Configuración +)
```c
// Configuración +: 
// Servo 1 (+X): controla pitch
// Servo 2 (-X): controla pitch (invertido)
// Servo 3 (+Y): controla yaw
// Servo 4 (-Y): controla yaw (invertido)

void flight_control_apply(float pitch_correction, float yaw_correction) {
    servo_set_deflection(0, +pitch_correction);  // Servo 1: +pitch
    servo_set_deflection(1, -pitch_correction);  // Servo 2: -pitch (opuesto)
    servo_set_deflection(2, +yaw_correction);    // Servo 3: +yaw
    servo_set_deflection(3, -yaw_correction);    // Servo 4: -yaw (opuesto)
}
```

### Paso 5: Loop de Control (50 Hz)
```c
static pid_controller_t pitch_pid, yaw_pid;

void vTaskFlightControl(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  // 50 Hz
    const float dt = 0.02f;
    
    // Inicializar PIDs
    pid_init(&pitch_pid, 2.0f, 0.1f, 0.5f);
    pid_init(&yaw_pid,   2.0f, 0.1f, 0.5f);
    
    flight_control_init();
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        uint8_t state = fsm_get_state();
        
        if (state == STATE_BOOST || state == STATE_COAST) {
            // PID activo
            imu_data_t imu;
            xQueuePeek(imu_data_q, &imu, 0);
            
            // Setpoint: 0° (vertical)
            float pitch_cmd = pid_update(&pitch_pid, 0.0f, imu.pitch, dt);
            float yaw_cmd   = pid_update(&yaw_pid,   0.0f, imu.yaw, dt);
            
            flight_control_apply(pitch_cmd, yaw_cmd);
            
            // Log cada 0.5s
            static uint8_t log_cnt = 0;
            if (++log_cnt >= 25) {
                log_cnt = 0;
                printf("[CTRL] P:%.1f° Y:%.1f° → Pcmd:%.1f° Ycmd:%.1f°\n",
                       imu.pitch, imu.yaw, pitch_cmd, yaw_cmd);
            }
        } else {
            // Servos neutros
            flight_control_apply(0.0f, 0.0f);
            pid_reset(&pitch_pid);
            pid_reset(&yaw_pid);
        }
    }
}
```

---

## Tuning del PID en Simulación

### Método: Ziegler-Nichols Adaptado

1. **Ki = 0, Kd = 0**: Solo proporcional
2. Subir Kp hasta que el sistema oscile (Ku = ganancia última)
3. Medir período de oscilación (Tu)
4. Calcular ganancias:
   - Kp = 0.6 × Ku
   - Ki = 2 × Kp / Tu
   - Kd = Kp × Tu / 8

### Simulación de Perturbaciones
```c
// En modo SIMULATION, inyectar perturbaciones de orientación
#ifdef SIMULATION_MODE
void sim_inject_disturbance(imu_data_t *imu, uint32_t tick) {
    // Ráfaga de viento a T=5s: pitch se desvía +5°
    if (tick > 5000 && tick < 5500) {
        imu->pitch += 5.0f;
    }
    // Thrust offset: yaw drift constante de +0.5°/s
    if (fsm_get_state() == STATE_BOOST) {
        imu->yaw += 0.01f;  // Acumulativo
    }
}
#endif
```

---

## API del Módulo

```c
// flight_control.h

// Inicialización
void fctrl_init(void);

// Control
void fctrl_enable(void);
void fctrl_disable(void);      // Servos a neutro
bool fctrl_is_active(void);

// PID tuning (para ajuste en tiempo real por telemetría)
void fctrl_set_pid_gains(float kp, float ki, float kd);
void fctrl_get_pid_gains(float *kp, float *ki, float *kd);
void fctrl_reset_pid(void);

// Diagnóstico
float fctrl_get_pitch_error(void);
float fctrl_get_yaw_error(void);
float fctrl_get_pitch_output(void);
float fctrl_get_yaw_output(void);
```

---

## Criterios de Aceptación

- [ ] PWM a 50 Hz verificado en osciloscopio (período = 20 ms ±1%)
- [ ] Rango de pulso 1000-2000 µs verificado
- [ ] Servos en posición neutral al arrancar
- [ ] PID corrige perturbación de ±5° a 0° en < 2 segundos
- [ ] No hay overshoot > 50% de la perturbación
- [ ] Anti-windup previene acumulación excesiva de integral
- [ ] Saturación de ±15° respetada (no supera límite mecánico)
- [ ] PID se desactiva en estados de descenso
- [ ] PID se resetea al cambiar de estado

---

## Referencias

- [PID Controller Theory](https://en.wikipedia.org/wiki/PID_controller)
- [Servo PWM Standard](https://www.servocity.com/how-does-a-servo-work)
- [Rocket Stability and Control](https://www.nakka-rocketry.net/fins.html)

---

*Módulo 10 - Las manos del cohete. Corrige su rumbo cuando el viento intenta desviarlo.*
