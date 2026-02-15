# Módulo 05: Barometer Altimeter - Barómetro y Altímetro

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Sexto módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core, 03-Sensors Interface  
> **Dependientes**: 09-Recovery System, 11-State Machine

---

## Resumen

Sensor de presión barométrica BMP280 para cálculo de altitud AGL (Above Ground Level) y velocidad vertical. Es el sensor **más crítico del sistema**: la detección de apogeo depende principalmente de la velocidad vertical derivada de la altitud barométrica, y el despliegue del paracaídas principal se basa en el cruce de un umbral de altitud.

---

## Especificaciones Técnicas

### Sensor BMP280
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Interfaz                 | I2C (dirección 0x76, SDO=GND)  |
| Voltaje operación        | 1.71V - 3.6V                   |
| Consumo                  | 2.7 µA @ 1 Hz, 720 µA en medición |
| Rango de presión         | 300 - 1100 hPa                 |
| Resolución presión       | 0.16 Pa (modo ultra-high res)  |
| Precisión absoluta       | ±1 hPa (±8.4 m a nivel del mar)|
| Precisión relativa       | ±0.12 hPa (±1 m, key spec!)   |
| Rango de temperatura     | -40°C a +85°C                  |
| Resolución temperatura   | 0.01°C                         |
| Output Data Rate         | Hasta 157 Hz (oversampling ×1) |
| Modelo Proteus           | ✅ Disponible (I2C slave)       |

### Configuración Elegida
| Parámetro                | Valor                    | Razón                   |
|--------------------------|--------------------------|-------------------------|
| Oversampling presión     | ×16 (ultra-high res)     | Máxima resolución       |
| Oversampling temperatura | ×2                        | Suficiente precisión    |
| IIR Filter coeff.        | 16                        | Suavizado de lecturas   |
| Modo                     | Normal (continuo)         | Lectura periódica       |
| Standby time             | 0.5 ms                   | ODR ≈ 26 Hz con ×16    |

> **Nota**: Con oversampling ×16 + IIR 16, la resolución efectiva es ~0.16 Pa ≈ **0.013 m** - mucho mejor que lo necesario.

---

## Decisiones Técnicas

### ¿Por qué BMP280 y no MS5611 o BMP388?

| Criterio              | BMP280       | MS5611       | BMP388       |
|-----------------------|--------------|--------------|--------------|
| **Modelo Proteus**    | ✅ Sí        | ❌ No        | ❌ No        |
| Precisión relativa    | ±0.12 hPa   | ±0.012 hPa  | ±0.08 hPa   |
| Precio                | ~$2          | ~$8          | ~$5          |
| I2C                   | Sí           | Sí           | Sí           |
| Resolución altitud    | ~1 m         | ~0.1 m       | ~0.5 m       |

**Decisión**: BMP280 - modelo VSM disponible en Proteus. Para vuelo real, se migra a BMP388 o MS5611 cambiando solo el driver. La interfaz I2C y la API del módulo serían idénticas.

### ¿Por qué altitud AGL y no MSL?
- **MSL (Mean Sea Level)**: Requiere conocer la presión a nivel del mar (QNH) - varía con el clima
- **AGL (Above Ground Level)**: Se calibra en tierra antes del vuelo → altitud = 0 en la rampa
- Para las decisiones de vuelo (apogeo, deploy main), solo importa AGL
- GPS proporciona MSL si se necesita para referencia

### ¿Por qué IIR filter coefficient = 16?
- Sin IIR: Lecturas ruidosas, velocidad vertical imprecisa
- IIR 16: Suavizado pronunciado, ~3 muestras de settling time
- El ruido en velocidad vertical causaría falsas detecciones de apogeo sin IIR
- Con IIR 16, la resolución de altitud mejora a ~0.25 m RMS
- **Trade-off**: Mayor latencia (~60ms) - aceptable para nuestro 50ms apogee budget total

### ¿Cómo se calcula la velocidad vertical?
- Velocidad vertical = Δaltitud / Δt (derivada numérica)
- Derivada directa es ruidosa → aplicar media móvil de 5 muestras
- A 100Hz, ventana de 5 muestras = 50ms → resolución temporal aceptable
- **Alternativa**: Kalman filter 1D (presión + aceleración IMU) - para versión avanzada

---

## Diseño del Circuito

### Esquemático

```
                    3.3V
                     │
                   100nF (bypass)
                     │
              ┌──────┴──────┐
              │   BMP280    │
    SDA ──────┤ SDA     VCC ├──── 3.3V
    SCL ──────┤ SCL     GND ├──── GND
              │ CSB     SDO ├──── GND (addr = 0x76)
              │         VCC ├──── 3.3V (CSB pull-up: I2C mode)
              └─────────────┘
```

### Componentes

| Ref  | Componente     | Valor        | Notas                          |
|------|----------------|--------------|--------------------------------|
| U5   | BMP280         | LGA-8        | I2C addr 0x76 (SDO=GND)       |
| C13  | Bypass cap     | 100nF        | Cerámico, junto a VCC          |

> **Nota**: CSB debe estar HIGH para modo I2C (conectado a VCC). SDO=GND → addr 0x76.

---

## Desarrollo Step-by-Step

### Paso 1: Verificar Comunicación I2C
1. Añadir BMP280 al esquemático
2. I2C scan → debe detectar 0x76
3. Leer registro `id` (0xD0) → debe retornar 0x58 (BMP280)
4. **Test**: `[BARO] BMP280 detected, chip_id = 0x58`

### Paso 2: Leer Coeficientes de Calibración
```c
// El BMP280 tiene coeficientes de calibración únicos por chip
// Almacenados en registros 0x88-0xA1 (26 bytes)
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

void bmp280_read_calibration(bmp280_calib_t *calib) {
    uint8_t buffer[26];
    i2c_read(BMP280_ADDR, 0x88, buffer, 26);
    
    calib->dig_T1 = (buffer[1] << 8) | buffer[0];
    calib->dig_T2 = (buffer[3] << 8) | buffer[2];
    calib->dig_T3 = (buffer[5] << 8) | buffer[4];
    calib->dig_P1 = (buffer[7] << 8) | buffer[6];
    // ... (continuar para P2-P9)
}
```

### Paso 3: Configuración del Sensor
```c
#define BMP280_ADDR          0x76
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5

void bmp280_init(void) {
    uint8_t data;
    
    // Config: standby=0.5ms, IIR_coeff=16, SPI off
    data = (0x00 << 5) | (0x04 << 2) | 0x00;  // t_sb=0.5ms, filter=16
    i2c_write(BMP280_ADDR, BMP280_REG_CONFIG, &data, 1);
    
    // Ctrl_meas: osrs_t=×2, osrs_p=×16, mode=normal
    data = (0x02 << 5) | (0x05 << 2) | 0x03;  // temp×2, press×16, normal
    i2c_write(BMP280_ADDR, BMP280_REG_CTRL_MEAS, &data, 1);
}
```

### Paso 4: Lectura y Compensación
```c
// Compensación de temperatura (del datasheet, Bosch code)
int32_t bmp280_compensate_temperature(int32_t adc_T, bmp280_calib_t *cal) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)cal->dig_T1 << 1))) * 
            ((int32_t)cal->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)cal->dig_T1)) * 
              ((adc_T >> 4) - ((int32_t)cal->dig_T1))) >> 12) * 
            ((int32_t)cal->dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;  // Temp en 0.01 °C
    return T;
}

// Compensación de presión (del datasheet)
uint32_t bmp280_compensate_pressure(int32_t adc_P, bmp280_calib_t *cal) {
    // Implementación completa del datasheet (64-bit fixed point)
    // Retorna presión en Pa con 256 de resolución fraccional
    // ... (ver datasheet sección 4.2.3)
}
```

### Paso 5: Cálculo de Altitud
```c
static float ground_pressure = 101325.0f;  // Se calibra en tierra

// Fórmula barométrica internacional
float bmp280_calculate_altitude(float pressure_pa) {
    // h = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure_pa / ground_pressure, 0.190295f));
}

// Calibración: guardar presión en tierra como referencia
void bmp280_calibrate_ground(void) {
    float sum = 0.0f;
    for (int i = 0; i < 100; i++) {
        sum += bmp280_read_pressure();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ground_pressure = sum / 100.0f;
    printf("[BARO] Ground pressure calibrated: %.2f Pa\n", ground_pressure);
}
```

### Paso 6: Velocidad Vertical
```c
#define VSPEED_WINDOW 5  // Media móvil de 5 muestras

static float altitude_history[VSPEED_WINDOW];
static uint8_t alt_idx = 0;

float bmp280_calculate_vspeed(float current_altitude) {
    // Guardar altitud actual
    uint8_t oldest = (alt_idx + 1) % VSPEED_WINDOW;
    float oldest_alt = altitude_history[oldest];
    altitude_history[alt_idx] = current_altitude;
    alt_idx = (alt_idx + 1) % VSPEED_WINDOW;
    
    // Δalt / Δt (ventana de 5 muestras a 100Hz = 50ms)
    float dt = VSPEED_WINDOW * 0.01f;  // 50ms
    return (current_altitude - oldest_alt) / dt;
}
```

### Paso 7: Detección de Apogeo
```c
typedef struct {
    bool detected;
    float altitude_at_apogee;
    uint32_t timestamp_ms;
} apogee_info_t;

static float prev_vspeed = 0.0f;
static uint8_t apogee_confirm_count = 0;
#define APOGEE_CONFIRM_SAMPLES 3  // 3 muestras consecutivas con vspeed < 0

apogee_info_t baro_check_apogee(float vspeed, float altitude) {
    apogee_info_t info = {.detected = false};
    
    // Detección: velocidad vertical cambia de positiva a negativa
    // Confirmar con 3 muestras consecutivas para evitar falsos positivos
    if (vspeed < -0.5f && prev_vspeed >= 0.0f) {
        apogee_confirm_count = 1;
    } else if (vspeed < -0.5f && apogee_confirm_count > 0) {
        apogee_confirm_count++;
    } else if (vspeed >= 0.0f) {
        apogee_confirm_count = 0;
    }
    
    if (apogee_confirm_count >= APOGEE_CONFIRM_SAMPLES) {
        info.detected = true;
        info.altitude_at_apogee = altitude;
        info.timestamp_ms = HAL_GetTick();
        apogee_confirm_count = 0;  // Reset para no re-detectar
    }
    
    prev_vspeed = vspeed;
    return info;
}
```

### Paso 8: Tarea RTOS
```c
void vTaskBarometer(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 100 Hz
    
    bmp280_init();
    bmp280_calibrate_ground();
    printf("[BARO] BMP280 initialized, ground alt calibrated\n");
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        baro_data_t data;
        data.pressure = bmp280_read_pressure();
        data.temperature = bmp280_read_temperature();
        data.altitude = bmp280_calculate_altitude(data.pressure);
        data.vspeed = bmp280_calculate_vspeed(data.altitude);
        
        // Publicar para State Machine
        xQueueOverwrite(baro_data_q, &data);
    }
}
```

---

## Correlación de Datos en Simulación

### Relación Presión-Altitud-Velocidad

| Fase         | Presión (Pa) | Altitud (m) | V. Vertical (m/s) | Coherencia               |
|--------------|--------------|-------------|-------------------|---------------------------|
| PAD_IDLE     | 101325       | 0           | 0                 | Estable                   |
| BOOST        | Decreciente  | Creciente   | +50 a +180        | ΔP ∝ -Δalt               |
| COAST        | Decreciente  | Creciente   | +180 → 0          | Desacelera                |
| **APOGEE**   | **Mínima**   | **Máxima**  | **≈ 0**           | **Punto de inflexión**    |
| DESCENT      | Creciente    | Decreciente | -15 (drogue)      | ΔP ∝ -Δalt               |
| MAIN_DEPLOY  | Creciente    | Decreciente | -15 → -5          | Frenado por main chute    |
| LANDED       | 101325       | 0           | 0                 | Igual que PAD             |

### Conversión Presión → Registros BMP280
```python
def pressure_to_bmp280_raw(pressure_pa, temperature_c, calib_coeffs):
    """
    Inversión de la fórmula de compensación del BMP280.
    Dado un valor de presión objetivo, calcula el ADC raw que 
    el BMP280 debería producir para que, tras compensación, 
    se obtenga esa presión.
    """
    # Esto requiere invertir numéricamente las fórmulas de Bosch
    # Se implementa con búsqueda binaria o Newton-Raphson
    # Ver tools/data-converter/bmp280_inverse.py
    pass
```

---

## API del Módulo

```c
// barometer_altimeter.h

// Inicialización y calibración
HAL_StatusTypeDef baro_init(I2C_HandleTypeDef *hi2c);
void baro_calibrate_ground(void);

// Lectura
baro_data_t baro_read(void);
float baro_get_altitude(void);
float baro_get_vspeed(void);

// Detección de eventos
apogee_info_t baro_check_apogee(void);
bool baro_altitude_below(float threshold_m);  // Para main deploy

// Configuración
void baro_set_ground_pressure(float pressure_pa);
void baro_set_main_deploy_altitude(float altitude_m);
```

---

## Criterios de Aceptación

- [ ] BMP280 detectado en I2C (chip_id = 0x58)
- [ ] Coeficientes de calibración leídos correctamente
- [ ] Presión leída con error < 1 hPa vs dato inyectado
- [ ] Altitud AGL precisa a ±1 m (tras calibración en tierra)
- [ ] Velocidad vertical precisa a ±0.5 m/s
- [ ] Detección de apogeo con latencia < 50 ms
- [ ] Detección de altitud de main deploy correcta (±5 m)
- [ ] Calibración en tierra estable (100 muestras promediadas)
- [ ] IIR filter suaviza adecuadamente sin latencia excesiva
- [ ] Tarea a 100 Hz sin jitter > 1 ms

---

## Simulación en Proteus

### Escenarios de Test

| # | Escenario                          | Resultado Esperado                              |
|---|------------------------------------|-------------------------------------------------|
| 1 | Presión constante (101325 Pa)      | Altitud ≈ 0 m, vspeed ≈ 0 m/s                  |
| 2 | Rampa de presión decreciente       | Altitud creciente, vspeed > 0                   |
| 3 | Presión mínima (apogeo)            | Detección de apogeo, vspeed cruza por 0         |
| 4 | Rampa de presión creciente         | Altitud decreciente, vspeed < 0                 |
| 5 | Cruce de umbral de altitud (300m)  | Trigger de main deploy                          |
| 6 | Perfil de vuelo completo (CSV)     | Toda la secuencia coherente                     |
| 7 | Sensor desconectado                | Error I2C, datos anteriores mantenidos          |

---

## Referencias

- [BMP280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [International Barometric Formula](https://en.wikipedia.org/wiki/Barometric_formula)
- [BMP280 Compensation Formulas (Bosch)](https://github.com/BoschSensortec/BMP280_driver)

---

*Módulo 05 - El juez del apogeo. Su veredicto decide cuándo se abre el paracaídas.*
