# Guía de Simulación en Proteus - Rocket Avionics

## Filosofía: Simulación con Datos Reales

### El Problema de las Simulaciones Tradicionales
En una simulación típica de Proteus, los sensores se alimentan con potenciómetros o valores fijos. Esto genera datos **no correlacionados**:
- El barómetro puede indicar "subiendo" mientras la IMU dice "bajando"
- El GPS puede indicar una posición que no corresponde con la altitud barométrica
- La máquina de estados puede transicionar incorrectamente por datos incoherentes

### Nuestra Solución: Pipeline de Datos Correlacionados

Utilizamos **datos de vuelo reales** (OpenRocket o registros de vuelos) como fuente única de verdad. Un script Python convierte estos datos al formato que cada sensor produciría, garantizando coherencia física absoluta.

---

## Pipeline Completo

### Paso 1: Obtener Datos de Vuelo

**Opción A - OpenRocket (Recomendado para empezar)**
1. Descargar [OpenRocket](https://openrocket.info/)
2. Diseñar un cohete o usar uno de ejemplo
3. Ejecutar simulación
4. `File → Export → CSV` con las columnas:
   - Time (s)
   - Altitude (m)
   - Vertical velocity (m/s)
   - Vertical acceleration (m/s²)
   - Total acceleration (m/s²)
   - Lateral distance (m)
   - Angle of attack (°)
   - Air pressure (Pa)
   - Mach number

**Opción B - Datos de vuelos reales**
- Buscar datasets de vuelos amateur (NAR, TRA, UKRA)
- Formato CSV con columnas similares
- Asegurarse de que incluye presión o calcularla desde altitud

### Paso 2: Formato CSV de Entrada

```csv
time_s,altitude_m,velocity_ms,accel_ms2,pressure_pa,temperature_c,latitude,longitude
0.000,0.00,0.00,0.00,101325.0,20.0,40.4168,-3.7038
0.010,0.00,0.00,0.00,101325.0,20.0,40.4168,-3.7038
0.100,0.05,1.20,25.00,101319.1,20.0,40.4168,-3.7038
0.200,0.35,5.80,38.00,101283.2,19.9,40.4168,-3.7038
...
```

### Paso 3: Conversión a Datos de Sensor

El script `tools/data-converter/flight_to_proteus.py` realiza la conversión:

```python
# Pseudocódigo de la conversión
for each timestep in flight_data:
    # BMP280: presión → registros raw I2C
    bmp_raw = pressure_to_bmp280_registers(pressure_pa, temperature_c)
    
    # MPU6050: aceleración + velocidad angular → registros raw I2C
    mpu_raw = accel_gyro_to_mpu6050_registers(accel_xyz, gyro_xyz)
    
    # GPS: posición → trama NMEA
    nmea = position_to_nmea_gga(lat, lon, alt_gps, time, sats=8)
    
    # Escribir archivos de estímulo
    write_proteus_stimulus(timestamp, bmp_raw, mpu_raw, nmea)
```

### Paso 4: Inyección en Proteus

Dependiendo del método de simulación elegido, hay varias estrategias:

#### Método A: MCU Auxiliar como Generador de Datos (Recomendado)

```
┌──────────────────────┐        ┌──────────────────────┐
│   Arduino/STM32      │  I2C   │    STM32F401         │
│   "Sensor Faker"     │───────▶│    Flight Computer   │
│                      │  UART  │                      │
│  Lee CSV desde       │───────▶│  Lee datos como si   │
│  arrays en flash     │        │  fueran sensores     │
│                      │        │  reales              │
└──────────────────────┘        └──────────────────────┘
```

- Un segundo MCU en la simulación actúa como **I2C slave** emulando los sensores
- Responde a las mismas direcciones que MPU6050, BMP280, etc.
- Los valores que responde vienen de los datos de vuelo pre-cargados
- **Ventaja**: La comunicación I2C es real, el firmware de vuelo no sabe que son datos simulados

#### Método B: Virtual Instruments + Generators

- Usar los generadores de señal de Proteus
- Configurar rampas que correspondan al perfil de vuelo
- Menos preciso pero más simple para tests iniciales

#### Método C: Valores Directos en Código (Para Debug)

```c
#ifdef SIMULATION_MODE
    // Override sensor reads con datos de tabla
    extern const flight_data_t flight_profile[];
    extern uint32_t sim_tick;
    
    sensor_data.pressure = flight_profile[sim_tick].pressure_pa;
    sensor_data.accel_z  = flight_profile[sim_tick].accel_z;
    // ...
    sim_tick++;
#endif
```

---

## Configuración de Proteus por Módulo

### Instrumentos Virtuales Comunes

| Instrumento          | Uso                                            |
|----------------------|------------------------------------------------|
| Virtual Terminal     | Debug UART, logs, tramas NMEA                  |
| I2C Debugger         | Verificar tráfico I2C (direcciones, datos)     |
| SPI Debugger         | Verificar comunicación con Flash/LoRa          |
| Oscilloscope         | Señales PWM de servos, timing de tareas        |
| Logic Analyzer       | Secuencia de eventos, timing de interrupciones |
| Graph (Analogue)     | Voltajes en tiempo real (batería, ADC)         |
| Graph (Digital)      | Señales de control (pyro, LEDs)                |

### Configuración del Proyecto Proteus

```
Proyecto Proteus (.pdsprj)
├── Design Sheet 1: Power Management
│   └── Reguladores, INA219, LEDs
├── Design Sheet 2: MCU Core
│   └── STM32F401, oscilador, reset, SWD
├── Design Sheet 3: Sensors
│   └── MPU6050, BMP280, pull-ups I2C
├── Design Sheet 4: GPS + Radio
│   └── NEO-6M (UART), RFM95W (SPI)
├── Design Sheet 5: Recovery + Control
│   └── MOSFETs, servos, arming switch
└── Design Sheet 6: Diagnostics
    └── LEDs RGB, buzzer, debug UART
```

---

## Perfiles de Vuelo Predefinidos

### Perfil 1: Vuelo Nominal
```
Duración total: ~120 segundos
Motor: H128 (impulso total: 128 Ns)
Altitud máxima: ~800 m
Velocidad máxima: ~180 m/s
Aceleración máxima: ~8g
Tiempo a apogeo: ~12 s
Main deploy: 300 m

Fases:
  0-0.5s:   PAD_IDLE → ARMED (manual)
  0.5-2.5s: BOOST (motor encendido, ~8g → 2g)
  2.5-12s:  COAST (desaceleración, 0g → -1g)
  12s:      APOGEE (v=0, fire drogue)
  12-80s:   DESCENT_DROGUE (~15 m/s descenso)
  80s:      MAIN_DEPLOY (alt=300m, fire main)
  80-115s:  DESCENT_MAIN (~5 m/s descenso)
  115-120s: LANDED + RECOVERY_MODE
```

### Perfil 2: Vuelo con Fallo de Drogue
```
Igual que Perfil 1 pero:
  12s:      APOGEE → drogue NO despliega
  12-14s:   Backup timer → force deploy
  14s:      Drogue despliega por backup
```

### Perfil 3: Batería Baja en Vuelo
```
Igual que Perfil 1 pero:
  8s:       Voltaje de batería cae por debajo de 9.0V
  8s:       CRITICAL alarm → Emergency procedures
```

---

## Verificaciones de Simulación (Checklist)

### Por Módulo Individual
- [ ] El módulo arranca sin errores
- [ ] La comunicación I2C/SPI/UART funciona correctamente
- [ ] Los datos leídos coinciden con los inyectados
- [ ] Los logs en Virtual Terminal son correctos
- [ ] Sin warnings ni errores de compilación

### Sistema Integrado
- [ ] Todos los módulos arrancan en secuencia correcta
- [ ] Los datos de sensores son coherentes entre sí
- [ ] La FSM transiciona correctamente con datos reales
- [ ] El drogue se despliega en apogeo (±50ms)
- [ ] El main se despliega a la altitud correcta
- [ ] La telemetría transmite datos correctos
- [ ] El logging registra todos los datos
- [ ] El watchdog funciona ante timeout de tarea
- [ ] La batería baja genera alarma correcta
- [ ] El modo RECOVERY activa buzzer y GPS tracking

### Timing
- [ ] Tareas IMU/BARO se ejecutan a 100Hz (±1%)
- [ ] Jitter de tareas críticas < 1ms
- [ ] Latencia apogeo-deploy < 50ms
- [ ] Sin stack overflow durante vuelo completo
- [ ] CPU utilization < 70%

---

## Captura de Resultados para Portfolio

### Screenshots Recomendados
1. Esquemático completo del sistema
2. Virtual Terminal con secuencia de boot
3. Gráfica de altitud vs tiempo durante vuelo simulado
4. I2C Debugger mostrando tráfico de sensores
5. Momento de detección de apogeo (log + timing)
6. Señales PWM de servos en osciloscopio
7. Transiciones de la FSM en secuencia completa

### Video Demo
Grabar simulación completa mostrando:
1. Boot sequence (todos los OK)
2. Arming (switch)
3. Lanzamiento detectado (aceleración)
4. Ascenso (altitud creciente en tiempo real)
5. Apogeo detectado + drogue fire
6. Descenso con main deploy
7. Landing detection
8. Recovery mode con GPS tracking

---

*Esta guía se actualiza conforme se completan módulos y se descubren mejores prácticas de simulación.*
