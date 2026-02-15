# Módulo 04: IMU Navigation - Unidad de Medición Inercial

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Quinto módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core, 03-Sensors Interface  
> **Dependientes**: 10-Flight Control, 11-State Machine, 09-Recovery

---

## Resumen

Sistema de navegación inercial basado en el MPU6050 (acelerómetro + giroscopio 6-DOF). Proporciona orientación 3D en tiempo real mediante el filtro Madgwick AHRS, y es crítico para la detección de eventos de vuelo: lanzamiento (aceleración > 2g), motor burnout (aceleración < 0.5g), y contribución a la detección de apogeo.

---

## Especificaciones Técnicas

### Sensor MPU6050
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Interfaz                 | I2C (dirección 0x68)            |
| Voltaje operación        | 2.375V - 3.46V                  |
| Consumo                  | 3.6 mA típico                   |
| Acelerómetro - Rango     | ±2g, ±4g, **±8g**, ±16g        |
| Acelerómetro - Resolución| 16 bits (4096 LSB/g @ ±8g)     |
| Acelerómetro - Ruido     | 400 µg/√Hz                     |
| Giroscopio - Rango       | ±250, **±500**, ±1000, ±2000 °/s|
| Giroscopio - Resolución  | 16 bits (65.5 LSB/°/s @ ±500)  |
| Giroscopio - Ruido       | 0.005 °/s/√Hz                  |
| Output Data Rate         | Programable hasta 1 kHz         |
| DLPF (filtro digital)    | Configurable 5-260 Hz           |
| Modelo Proteus           | ✅ Disponible (I2C slave)       |

### Filtro Madgwick AHRS
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Tipo                     | Gradient Descent IMU fusion     |
| Frecuencia de ejecución  | 100 Hz (mismo rate que lectura) |
| Parámetro beta           | 0.1 (balance velocidad/suavidad)|
| Convergencia             | < 2 segundos desde arranque     |
| Salida                   | Cuaternión + Euler angles       |
| Complejidad              | ~250 FLOPs por iteración        |
| Tiempo de ejecución      | ~0.3 ms con FPU hardware        |

---

## Decisiones Técnicas

### ¿Por qué MPU6050 y no BNO055 o LSM9DS1?

| Criterio              | MPU6050      | BNO055       | LSM9DS1      |
|-----------------------|--------------|--------------|--------------|
| **Modelo Proteus**    | ✅ Sí        | ❌ No        | ❌ No        |
| Precio                | ~$2          | ~$25         | ~$10         |
| Fusión en hardware    | No           | Sí           | No           |
| DOF                   | 6 (Acc+Gyro) | 9 (Acc+Gyro+Mag)| 9         |
| Rango acelerómetro    | ±16g         | ±16g         | ±16g         |
| I2C                   | 400 kHz      | 400 kHz      | 400 kHz      |

**Decisión**: MPU6050 - el **modelo VSM disponible en Proteus** es el factor decisivo. Para vuelo real, se migra a BNO055 o ICM-20948 cambiando solo el driver.

### ¿Por qué Madgwick y no Kalman?

| Criterio                | Madgwick        | Kalman (EKF)    |
|-------------------------|-----------------|-----------------|
| Complejidad implementación | Baja          | Alta            |
| Complejidad computacional  | O(1) por step | O(n³) matrices  |
| Tuning de parámetros      | 1 (beta)      | Q, R matrices   |
| Calidad con 6-DOF         | Buena         | Excelente       |
| Uso con FPU STM32F4       | ~0.3 ms       | ~1.5 ms         |

**Decisión**: Madgwick - suficiente para 6-DOF, mucho más simple de implementar y debuggear, y deja más CPU libre para otras tareas.

### ¿Por qué ±8g y no ±16g para el acelerómetro?
- En vuelo, el pico de aceleración típico es ~8-10g (motor H/I class)
- ±8g da resolución de 4096 LSB/g → **0.24 mg** por LSB
- ±16g daría 2048 LSB/g → 0.49 mg por LSB (la mitad de resolución)
- Si algún vuelo supera ±8g, se puede reconfigurar por software sin cambiar hardware

### ¿Por qué ±500°/s para el giroscopio?
- Un cohete estable rara vez supera 200°/s de rotación
- ±500°/s da margen para ráfagas de viento y spin
- Mayor resolución que ±2000°/s (65.5 vs 16.4 LSB/°/s)

---

## Diseño del Circuito

### Esquemático

```
                    3.3V
                     │
                   100nF (bypass, lo más cerca posible del chip)
                     │
              ┌──────┴──────┐
              │   MPU6050   │
    SDA ──────┤ SDA     VCC ├──── 3.3V
    SCL ──────┤ SCL     GND ├──── GND
              │ INT     AD0 ├──── GND (addr = 0x68)
              │         XDA ├──── NC (no DMP externo)
              │         XCL ├──── NC
              │         AUX ├──── NC
              └─────────────┘
                 │
                 ▼
            INT → PB1 (GPIO EXTI, rising edge)
            (Data Ready interrupt)
```

### ¿Por qué INT (Data Ready)?
- Sin INT: La tarea hace polling → puede leer datos a destiempo
- Con INT: El MPU6050 avisa cuando hay datos nuevos → lectura sincronizada, sin jitter
- EXTI en PB1 con priority mayor que RTOS → notificación inmediata a la tarea

### Componentes

| Ref  | Componente     | Valor        | Notas                          |
|------|----------------|--------------|--------------------------------|
| U4   | MPU6050        | QFN-24       | I2C addr 0x68 (AD0=GND)       |
| C12  | Bypass cap     | 100nF        | Cerámico, lo más cerca de VCC  |
| R4   | Pull-up INT    | 10kΩ         | Si INT es open-drain           |

> **Nota**: Pull-ups I2C (4.7kΩ) ya están en el módulo 03-Sensors.

---

## Desarrollo Step-by-Step

### Paso 1: Verificar Comunicación I2C
1. Añadir MPU6050 al esquemático de simulación
2. Ejecutar I2C scan → debe detectar 0x68
3. Leer registro `WHO_AM_I` (0x75) → debe retornar 0x68
4. **Test**: `[IMU] MPU6050 detected, WHO_AM_I = 0x68`

### Paso 2: Configurar Registros del MPU6050
```c
// Configuración inicial
#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_INT_ENABLE    0x38

void mpu6050_init(void) {
    uint8_t data;
    
    // Despertar al MPU (sale de sleep)
    data = 0x00;
    i2c_write(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &data, 1);
    
    // Sample rate = 1kHz / (1 + SMPLRT_DIV)
    // SMPLRT_DIV = 9 → 100 Hz
    data = 0x09;
    i2c_write(MPU6050_ADDR, MPU6050_SMPLRT_DIV, &data, 1);
    
    // DLPF = 44Hz (reduce ruido, suficiente ancho de banda)
    data = 0x03;
    i2c_write(MPU6050_ADDR, MPU6050_CONFIG, &data, 1);
    
    // Gyro: ±500°/s (FS_SEL = 1)
    data = 0x08;
    i2c_write(MPU6050_ADDR, MPU6050_GYRO_CONFIG, &data, 1);
    
    // Accel: ±8g (AFS_SEL = 2)
    data = 0x10;
    i2c_write(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, &data, 1);
    
    // Habilitar Data Ready interrupt
    data = 0x01;
    i2c_write(MPU6050_ADDR, MPU6050_INT_ENABLE, &data, 1);
}
```
**Test**: Leer registros de configuración y verificar valores escritos.

### Paso 3: Lectura de Datos Raw
```c
#define MPU6050_ACCEL_XOUT_H  0x3B  // 14 bytes: accel(6) + temp(2) + gyro(6)

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t temperature;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_raw_t;

mpu6050_raw_t mpu6050_read_raw(void) {
    uint8_t buffer[14];
    mpu6050_raw_t raw;
    
    i2c_read(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14);
    
    raw.accel_x = (buffer[0] << 8) | buffer[1];
    raw.accel_y = (buffer[2] << 8) | buffer[3];
    raw.accel_z = (buffer[4] << 8) | buffer[5];
    raw.temperature = (buffer[6] << 8) | buffer[7];
    raw.gyro_x = (buffer[8] << 8) | buffer[9];
    raw.gyro_y = (buffer[10] << 8) | buffer[11];
    raw.gyro_z = (buffer[12] << 8) | buffer[13];
    
    return raw;
}
```
**Test**: Verificar que los datos raw corresponden a los valores inyectados por la simulación.

### Paso 4: Conversión a Unidades Físicas
```c
#define ACCEL_SCALE  (9.81f / 4096.0f)   // ±8g → m/s²
#define GYRO_SCALE   (1.0f / 65.5f)       // ±500°/s → °/s
#define DEG_TO_RAD   (M_PI / 180.0f)

imu_data_t mpu6050_read(void) {
    mpu6050_raw_t raw = mpu6050_read_raw();
    imu_data_t data;
    
    data.accel_x = raw.accel_x * ACCEL_SCALE;
    data.accel_y = raw.accel_y * ACCEL_SCALE;
    data.accel_z = raw.accel_z * ACCEL_SCALE;
    
    data.gyro_x = raw.gyro_x * GYRO_SCALE;
    data.gyro_y = raw.gyro_y * GYRO_SCALE;
    data.gyro_z = raw.gyro_z * GYRO_SCALE;
    
    return data;
}
```

### Paso 5: Implementar Filtro Madgwick
```c
// Variables de estado del filtro (cuaternión)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static const float beta = 0.1f;
static const float sampleFreq = 100.0f;

void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Convertir gyro a rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2; _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1;
        q2q2 = q2 * q2; q3q3 = q3 * q3;

        // Gradient descent corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

// Extraer ángulos de Euler del cuaternión
void madgwick_get_euler(float *pitch, float *roll, float *yaw) {
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f/M_PI);
    *pitch = asinf(2.0f*(q0*q2 - q3*q1)) * (180.0f/M_PI);
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f/M_PI);
}
```
**Test**: Con acelerómetro apuntando +Z (cohete vertical), pitch ≈ 0°, roll ≈ 0°.

### Paso 6: Tarea RTOS
```c
void vTaskIMU(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 100 Hz
    
    mpu6050_init();
    printf("[IMU] MPU6050 initialized, 100Hz sampling\n");
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        imu_data_t data = mpu6050_read();
        
        madgwick_update(data.gyro_x, data.gyro_y, data.gyro_z,
                        data.accel_x, data.accel_y, data.accel_z);
        
        madgwick_get_euler(&data.pitch, &data.roll, &data.yaw);
        data.q0 = q0; data.q1 = q1; data.q2 = q2; data.q3 = q3;
        
        // Publicar datos en cola para State Machine
        xQueueOverwrite(imu_data_q, &data);
    }
}
```

### Paso 7: Calibración
```c
// Calibración en tierra: promediar 1000 muestras con sensor estático
typedef struct {
    int16_t accel_offset_x, accel_offset_y, accel_offset_z;
    int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
} mpu6050_calibration_t;

mpu6050_calibration_t mpu6050_calibrate(uint16_t samples) {
    mpu6050_calibration_t cal = {0};
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;
    
    for (uint16_t i = 0; i < samples; i++) {
        mpu6050_raw_t raw = mpu6050_read_raw();
        sum_ax += raw.accel_x; sum_ay += raw.accel_y;
        sum_az += raw.accel_z;  // No restar 1g aquí, se hace después
        sum_gx += raw.gyro_x; sum_gy += raw.gyro_y;
        sum_gz += raw.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    cal.accel_offset_x = sum_ax / samples;
    cal.accel_offset_y = sum_ay / samples;
    cal.accel_offset_z = (sum_az / samples) - 4096;  // Restar 1g (±8g scale)
    cal.gyro_offset_x = sum_gx / samples;
    cal.gyro_offset_y = sum_gy / samples;
    cal.gyro_offset_z = sum_gz / samples;
    
    return cal;
}
```

---

## Correlación de Datos en Simulación

### Relación IMU con Datos de Vuelo

| Fase de Vuelo  | Accel Z (m/s²) | Accel X/Y     | Gyro          |
|----------------|-----------------|----------------|---------------|
| PAD_IDLE       | +9.81 (1g)      | ≈ 0            | ≈ 0           |
| BOOST          | +20 a +80       | Pequeñas pert. | Según viento  |
| COAST          | -9.81 a 0       | Decreciente    | Decreciente   |
| APOGEE         | -9.81 (caída)   | ≈ 0            | ≈ 0           |
| DESCENT        | ~0 (drag=1g)    | Según viento   | Según rotación|

### Conversión de Datos de Vuelo → Registros MPU6050

```python
# En tools/data-converter/flight_to_proteus.py

def accel_to_mpu6050_raw(accel_ms2, scale_g=8):
    """Convierte aceleración en m/s² a valor raw del MPU6050"""
    lsb_per_g = 32768.0 / scale_g  # 4096 para ±8g
    accel_g = accel_ms2 / 9.81
    raw = int(accel_g * lsb_per_g)
    return max(-32768, min(32767, raw))  # Clamp a int16

def gyro_to_mpu6050_raw(gyro_dps, scale_dps=500):
    """Convierte velocidad angular en °/s a valor raw del MPU6050"""
    lsb_per_dps = 32768.0 / scale_dps  # 65.5 para ±500°/s
    raw = int(gyro_dps * lsb_per_dps)
    return max(-32768, min(32767, raw))
```

---

## API del Módulo

```c
// imu_navigation.h

// Inicialización
HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c);

// Calibración (llamar en PAD_IDLE, sensor estático)
void imu_calibrate(uint16_t samples);

// Lectura de datos procesados (accel + gyro + euler + quaternion)
imu_data_t imu_read(void);

// Detección de eventos
bool imu_detect_launch(float threshold_g, uint16_t duration_ms);
bool imu_detect_burnout(float threshold_g);
float imu_get_total_acceleration(void);  // Magnitud del vector accel

// Reset del filtro (para re-inicialización)
void imu_reset_filter(void);
```

---

## Criterios de Aceptación

- [ ] MPU6050 detectado en I2C scan (WHO_AM_I = 0x68)
- [ ] Lectura de aceleración coherente con datos inyectados (±0.5 m/s²)
- [ ] Lectura de giroscopio coherente con datos inyectados (±1°/s)
- [ ] Filtro Madgwick converge en < 2 segundos
- [ ] Ángulos de Euler correctos en orientación estática (pitch ≈ 0° vertical)
- [ ] Detección de lanzamiento (accel > 2g sostenida 100ms) funcional
- [ ] Detección de burnout (accel < 0.5g) funcional
- [ ] Tarea RTOS a 100Hz sin jitter > 1ms
- [ ] Calibración en tierra compensa offsets correctamente
- [ ] Datos publicados en cola sin pérdida

---

## Simulación en Proteus

### Instrumentos
- **I2C Debugger**: Verificar lectura de 14 bytes cada 10ms
- **Virtual Terminal**: Logs de orientación (pitch, roll, yaw)
- **Graph (Analogue)**: Aceleración Z vs tiempo (perfil de vuelo)
- **Logic Analyzer**: Verificar timing de INT pin

### Escenarios de Test

| # | Escenario                        | Resultado Esperado                              |
|---|----------------------------------|--------------------------------------------------|
| 1 | Sensor estático (1g arriba)      | accel_z ≈ 9.81, pitch ≈ 0°, roll ≈ 0°           |
| 2 | Simulación de lanzamiento (8g)   | Detección de launch en < 200ms                   |
| 3 | Motor burnout (accel < 0.5g)     | Detección de burnout inmediata                   |
| 4 | Rotación simulada (gyro)         | Ángulos de Euler cambian coherentemente          |
| 5 | Perfil de vuelo completo (CSV)   | Aceleración sigue curva del CSV inyectado        |
| 6 | Desconexión del sensor           | Error I2C reportado, datos anteriores mantenidos |

---

## Referencias

- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [Madgwick Filter Paper](https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf)
- [Sebastian Madgwick's Implementation](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

---

*Módulo 04 - Los ojos del cohete. Sabe dónde está y hacia dónde apunta en todo momento.*
