# Flight Data - Datos de Vuelo

## Visión General

Este directorio contiene los datos de vuelo que alimentan todo el sistema de simulación. La filosofía central del proyecto es que **los sensores simulados en Proteus reproduzcan datos físicamente coherentes derivados de perfiles de vuelo reales**.

```
flight-data/
├── raw/          ← CSVs exportados directamente de OpenRocket
├── processed/    ← Datos procesados con columnas derivadas
└── profiles/     ← Perfiles de misión listos para conversión a stimulus
```

---

## 1. Fuentes de Datos

### 1.1 OpenRocket (Fuente Principal)

[OpenRocket](https://openrocket.info/) es un simulador de vuelo de cohetes open source que genera perfiles realistas basados en las propiedades físicas del cohete (masa, drag, motor, etc.).

**Exportación desde OpenRocket:**
```
1. Diseñar cohete con propiedades similares al hardware real
2. Seleccionar motor (e.g., Estes E12, Cesaroni H143)
3. Simular vuelo
4. File → Export → Flight Data Export (CSV)
5. Seleccionar columnas:
   - Time (s)
   - Altitude (m)
   - Vertical velocity (m/s)
   - Vertical acceleration (m/s²)
   - Total acceleration (m/s²)
   - Lateral distance (m)
   - Angle of attack (°)
   - Roll rate (°/s)
   - Pitch rate (°/s)
   - Yaw rate (°/s)
   - Mass (kg)
   - Air pressure (Pa)
   - Air temperature (K)
   - Mach number
   - Thrust (N)
```

### 1.2 Datos Experimentales (Futuro)

Cuando se realicen vuelos reales con el hardware, los logs de la flash (W25Q128) se descargan y almacenan aquí para:
- Comparar simulación vs realidad
- Calibrar modelos
- Validar algoritmos

---

## 2. Formato de Datos

### 2.1 Raw CSV (OpenRocket Export)

```csv
# Time (s), Altitude (m), Vertical velocity (m/s), Vertical acceleration (m/s²), Total acceleration (m/s²), Lateral distance (m), Angle of attack (°), Roll rate (°/s), Pitch rate (°/s), Yaw rate (°/s), Air pressure (Pa), Air temperature (K), Thrust (N), Mass (kg)
0.000, 0.000, 0.000, 0.000, 9.810, 0.000, 0.000, 0.000, 0.000, 0.000, 101325.0, 288.15, 0.000, 0.850
0.010, 0.000, 0.050, 5.250, 11.120, 0.000, 0.100, 0.000, 0.050, 0.020, 101325.0, 288.15, 12.500, 0.849
0.020, 0.001, 0.103, 5.350, 11.250, 0.000, 0.150, 0.000, 0.080, 0.030, 101324.9, 288.15, 12.800, 0.848
...
```

### 2.2 Processed CSV (Columnas Derivadas)

El script de procesamiento añade columnas calculadas que son necesarias para generar los estímulos de cada sensor:

```csv
# time_s, altitude_m, velocity_ms, accel_vertical_ms2, accel_total_ms2, pressure_pa, temperature_k, accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps, pressure_hpa, altitude_baro_m, latitude, longitude, gps_altitude_m, flight_phase
0.000, 0.000, 0.000, 0.000, 1.000, 101325.0, 288.15, 0.000, 0.000, 1.000, 0.000, 0.000, 0.000, 1013.25, 0.000, 28.573469, -80.648972, 0.0, PAD_IDLE
0.010, 0.000, 0.050, 0.535, 1.134, 101325.0, 288.15, 0.010, 0.020, 1.130, 0.050, 0.080, 0.020, 1013.25, 0.000, 28.573469, -80.648972, 0.0, BOOST
...
```

**Columnas derivadas:**

| Columna | Cálculo | Unidad |
|---------|---------|--------|
| `accel_x_g` | `lateral_accel * sin(angle_attack) / 9.81` | g |
| `accel_y_g` | `lateral_accel * cos(angle_attack) / 9.81` | g |
| `accel_z_g` | `(accel_vertical + 9.81) / 9.81` | g |
| `gyro_x_dps` | `roll_rate` (directo de OpenRocket) | °/s |
| `gyro_y_dps` | `pitch_rate` | °/s |
| `gyro_z_dps` | `yaw_rate` | °/s |
| `pressure_hpa` | `pressure_pa / 100` | hPa |
| `altitude_baro_m` | `44330 * (1 - (P/P0)^0.19029)` | m |
| `latitude` | `lat0 + drift_north(t)` | ° |
| `longitude` | `lon0 + drift_east(t)` | ° |
| `gps_altitude_m` | `altitude_m + noise(±3m)` | m |
| `flight_phase` | Basado en condiciones de transición FSM | enum |

---

## 3. Perfiles de Vuelo Predefinidos

### 3.1 Low-Power (Motor E)

```
Motor:         Estes E12-6
Apogeo:        ~250m
Tiempo vuelo:  ~25s
Max velocity:  ~85 m/s
Max accel:     ~8g
Uso:           Testing básico, validación de FSM
```

### 3.2 Mid-Power (Motor H)

```
Motor:         Cesaroni H143-14A
Apogeo:        ~850m
Tiempo vuelo:  ~55s
Max velocity:  ~180 m/s
Max accel:     ~12g
Uso:           Perfil nominal, pruebas de telemetría
```

### 3.3 High-Power (Motor J)

```
Motor:         Cesaroni J280-14A
Apogeo:        ~1800m
Tiempo vuelo:  ~90s
Max velocity:  ~280 m/s
Max accel:     ~15g
Uso:           Stress test, límites de sensores
```

### 3.4 Abort Scenarios

```
abort_early_burnout:   Motor falla a T+1s (caída desde baja altitud)
abort_cato:            Explosión de motor (aceleración negativa súbita)
abort_chute_fail:      Paracaídas no despliega (caída balística)
wind_drift:            Viento lateral fuerte (20 km/h)
```

---

## 4. Correlación Física de Datos

### 4.1 Reglas de Coherencia

Todos los datos derivados deben cumplir estas relaciones físicas en todo instante `t`:

```
REGLA 1: Altitud-Presión (Fórmula Barométrica)
    P(h) = P₀ × (1 - 2.25577×10⁻⁵ × h)^5.25588
    Donde: P₀ = presión a nivel del suelo, h = altitud en metros
    Tolerancia: ±2 hPa

REGLA 2: Velocidad-Altitud (Derivada)
    v(t) ≈ [h(t) - h(t-dt)] / dt
    Tolerancia: ±2 m/s (promediado sobre 5 muestras)

REGLA 3: Aceleración-Velocidad (Derivada)
    a(t) ≈ [v(t) - v(t-dt)] / dt
    Tolerancia: ±0.5 g

REGLA 4: Apogeo (Condiciones simultáneas)
    En t_apogee:
    - velocity_vertical = 0  (±2 m/s)
    - altitude = máximo del vuelo
    - pressure = mínimo del vuelo
    - accel_z ≈ -1g (caída libre, solo gravedad)

REGLA 5: GPS-Barométrico (Correlación)
    |altitude_gps - altitude_baro| < 15m
    (GPS tiene menor resolución y mayor latencia)

REGLA 6: Conservación de Energía (Aproximada)
    En coast (sin thrust):
    ½mv² + mgh ≈ constante (decrece por drag)
    La energía total solo puede decrecer, nunca aumentar
```

### 4.2 Tabla de Correlación por Fase de Vuelo

```
┌──────────────┬──────────┬──────────┬──────────┬──────────┬──────────┐
│    Fase      │ Altitud  │Velocidad │ Accel Z  │ Presión  │   GPS    │
├──────────────┼──────────┼──────────┼──────────┼──────────┼──────────┤
│ PAD_IDLE     │   0      │   0      │  +1g     │   P₀     │  Fija    │
│ ARMED        │   0      │   0      │  +1g     │   P₀     │  Fija    │
│ BOOST        │   ↑↑     │   ↑↑     │  >>1g    │   ↓↓     │  ↑ slow  │
│ COAST        │   ↑      │   ↑→↓    │  <1g     │   ↓      │  ↑       │
│ APOGEE       │  MAX     │   ≈0     │  ≈-1g    │  MIN     │  ≈MAX    │
│ DROGUE_DEPL  │   ↓      │   ↓      │  spike+  │   ↑      │  ↓       │
│ DESC_DROGUE  │   ↓      │  -15~-25 │  ≈-1g    │   ↑      │  ↓       │
│ MAIN_DEPLOY  │   ↓      │   ↓      │  spike+  │   ↑      │  ↓       │
│ DESC_MAIN    │   ↓      │  -3~-5   │  ≈-1g    │   ↑      │  ↓       │
│ RECOVERY     │   ≈0     │   ≈0     │  +1g     │  ≈P₀     │  Fija    │
└──────────────┴──────────┴──────────┴──────────┴──────────┴──────────┘
```

---

## 5. Scripts de Procesamiento

### 5.1 Procesador Principal

```python
#!/usr/bin/env python3
"""
process_flight_data.py
Convierte CSV raw de OpenRocket a CSV procesado con columnas derivadas.

Uso:
    python process_flight_data.py --input raw/motor_H143.csv \
                                  --output processed/motor_H143_full.csv \
                                  --launch-lat 28.573469 \
                                  --launch-lon -80.648972
"""

import argparse
import numpy as np
import pandas as pd

# Constantes físicas
P0_HPA = 1013.25
G = 9.80665

def altitude_to_pressure(altitude_m, p0=P0_HPA):
    """Fórmula barométrica internacional."""
    return p0 * (1 - 2.25577e-5 * altitude_m) ** 5.25588

def pressure_to_altitude(pressure_hpa, p0=P0_HPA):
    """Inversa de la fórmula barométrica."""
    return 44330.0 * (1 - (pressure_hpa / p0) ** 0.19029)

def generate_gps_coordinates(df, launch_lat, launch_lon, wind_speed_ms=5.0, wind_dir_deg=270):
    """
    Genera coordenadas GPS simuladas basadas en drift por viento.
    Añade ruido gaussiano para simular precisión GPS real (CEP ~2.5m).
    """
    wind_rad = np.radians(wind_dir_deg)
    
    # Drift acumulado por viento
    dt = np.diff(df['time_s'].values, prepend=0)
    drift_east = np.cumsum(wind_speed_ms * np.sin(wind_rad) * dt)
    drift_north = np.cumsum(wind_speed_ms * np.cos(wind_rad) * dt)
    
    # Convertir metros a grados (aproximación local)
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * np.cos(np.radians(launch_lat))
    
    # Coordenadas con ruido GPS
    noise_lat = np.random.normal(0, 2.5 / meters_per_deg_lat, len(df))
    noise_lon = np.random.normal(0, 2.5 / meters_per_deg_lon, len(df))
    
    df['latitude'] = launch_lat + drift_north / meters_per_deg_lat + noise_lat
    df['longitude'] = launch_lon + drift_east / meters_per_deg_lon + noise_lon
    df['gps_altitude_m'] = df['altitude_m'] + np.random.normal(0, 3.0, len(df))
    df['gps_altitude_m'] = df['gps_altitude_m'].clip(lower=0)
    
    return df

def derive_imu_data(df):
    """
    Convierte aceleraciones y rotaciones de OpenRocket a formato IMU (MPU6050).
    Ejes del cohete: Z = longitudinal (thrust), X/Y = laterales
    """
    # Aceleración en g (incluye gravedad)
    # En reposo: accel_z = +1g (sensor mide reacción a gravedad)
    # En boost: accel_z = thrust_accel/g + 1
    df['accel_x_g'] = np.random.normal(0, 0.02, len(df))  # Lateral (mínimo)
    df['accel_y_g'] = np.random.normal(0, 0.02, len(df))  # Lateral (mínimo)
    df['accel_z_g'] = (df['accel_vertical_ms2'] + G) / G  # Longitudinal
    
    # Añadir ruido IMU realista (MPU6050 noise density: 400 µg/√Hz)
    imu_noise = 0.01  # ~0.01g RMS a 100Hz
    df['accel_x_g'] += np.random.normal(0, imu_noise, len(df))
    df['accel_y_g'] += np.random.normal(0, imu_noise, len(df))
    df['accel_z_g'] += np.random.normal(0, imu_noise, len(df))
    
    # Gyroscope (de OpenRocket si disponible, sino generar)
    if 'roll_rate_dps' not in df.columns:
        df['gyro_x_dps'] = np.random.normal(0, 0.5, len(df))
        df['gyro_y_dps'] = np.random.normal(0, 0.5, len(df))
        df['gyro_z_dps'] = np.random.normal(0, 0.5, len(df))
    else:
        df['gyro_x_dps'] = df['roll_rate_dps'] + np.random.normal(0, 0.1, len(df))
        df['gyro_y_dps'] = df['pitch_rate_dps'] + np.random.normal(0, 0.1, len(df))
        df['gyro_z_dps'] = df['yaw_rate_dps'] + np.random.normal(0, 0.1, len(df))
    
    return df

def assign_flight_phase(df):
    """Asigna fase de vuelo basándose en las condiciones de transición de la FSM."""
    phases = []
    current_phase = 'PAD_IDLE'
    armed = False
    drogue_deployed = False
    main_deployed = False
    
    for _, row in df.iterrows():
        t = row['time_s']
        alt = row['altitude_m']
        vel = row['velocity_ms']
        accel = row.get('accel_total_ms2', abs(row['accel_vertical_ms2']))
        
        if current_phase == 'PAD_IDLE' and t >= 2.0:
            current_phase = 'ARMED'
        elif current_phase == 'ARMED' and accel > 3.0 * G:
            current_phase = 'BOOST'
        elif current_phase == 'BOOST' and accel < 0.5 * G and t > 3.0:
            current_phase = 'COAST'
        elif current_phase == 'COAST' and vel <= 2.0 and alt > 50:
            current_phase = 'APOGEE'
        elif current_phase == 'APOGEE':
            current_phase = 'DROGUE_DEPLOY'
            drogue_deployed = True
        elif current_phase == 'DROGUE_DEPLOY':
            current_phase = 'DESCENT_DROGUE'
        elif current_phase == 'DESCENT_DROGUE' and alt < 150:
            current_phase = 'MAIN_DEPLOY'
            main_deployed = True
        elif current_phase == 'MAIN_DEPLOY':
            current_phase = 'DESCENT_MAIN'
        elif current_phase == 'DESCENT_MAIN' and abs(vel) < 2.0 and alt < 5:
            current_phase = 'RECOVERY'
        
        phases.append(current_phase)
    
    df['flight_phase'] = phases
    return df

def process_raw_csv(input_path, output_path, launch_lat, launch_lon):
    """Pipeline principal de procesamiento."""
    print(f"Loading: {input_path}")
    
    # Leer CSV raw (manejar diferentes formatos de OpenRocket)
    df = pd.read_csv(input_path, comment='#', skipinitialspace=True)
    
    # Normalizar nombres de columnas
    column_map = {
        'Time (s)': 'time_s',
        'Altitude (m)': 'altitude_m',
        'Vertical velocity (m/s)': 'velocity_ms',
        'Vertical acceleration (m/s²)': 'accel_vertical_ms2',
        'Total acceleration (m/s²)': 'accel_total_ms2',
        'Air pressure (Pa)': 'pressure_pa',
        'Air temperature (K)': 'temperature_k',
        'Roll rate (°/s)': 'roll_rate_dps',
        'Pitch rate (°/s)': 'pitch_rate_dps',
        'Yaw rate (°/s)': 'yaw_rate_dps',
        'Thrust (N)': 'thrust_n',
        'Mass (kg)': 'mass_kg',
    }
    df = df.rename(columns={k: v for k, v in column_map.items() if k in df.columns})
    
    print(f"  Rows: {len(df)}, Columns: {list(df.columns)}")
    
    # Derivar presión barométrica
    if 'pressure_pa' in df.columns:
        df['pressure_hpa'] = df['pressure_pa'] / 100.0
    else:
        df['pressure_hpa'] = altitude_to_pressure(df['altitude_m'])
    
    df['altitude_baro_m'] = pressure_to_altitude(df['pressure_hpa'])
    
    # Derivar datos IMU
    df = derive_imu_data(df)
    
    # Generar coordenadas GPS
    df = generate_gps_coordinates(df, launch_lat, launch_lon)
    
    # Asignar fases de vuelo
    df = assign_flight_phase(df)
    
    # Seleccionar columnas de salida
    output_columns = [
        'time_s', 'altitude_m', 'velocity_ms', 'accel_vertical_ms2',
        'pressure_hpa', 'temperature_k', 'altitude_baro_m',
        'accel_x_g', 'accel_y_g', 'accel_z_g',
        'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps',
        'latitude', 'longitude', 'gps_altitude_m',
        'flight_phase'
    ]
    
    # Solo incluir columnas que existen
    output_columns = [c for c in output_columns if c in df.columns]
    
    df[output_columns].to_csv(output_path, index=False, float_format='%.6f')
    print(f"  Output: {output_path} ({len(df)} rows, {len(output_columns)} columns)")
    
    # Estadísticas
    print(f"\n  === Flight Statistics ===")
    print(f"  Duration:     {df['time_s'].max():.1f} s")
    print(f"  Max altitude: {df['altitude_m'].max():.1f} m")
    print(f"  Max velocity: {df['velocity_ms'].max():.1f} m/s")
    if 'accel_total_ms2' in df.columns:
        print(f"  Max accel:    {df['accel_total_ms2'].max()/G:.1f} g")
    print(f"  Min pressure: {df['pressure_hpa'].min():.1f} hPa")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process OpenRocket flight data')
    parser.add_argument('--input', required=True, help='Input raw CSV path')
    parser.add_argument('--output', required=True, help='Output processed CSV path')
    parser.add_argument('--launch-lat', type=float, default=28.573469,
                        help='Launch site latitude (default: KSC)')
    parser.add_argument('--launch-lon', type=float, default=-80.648972,
                        help='Launch site longitude (default: KSC)')
    args = parser.parse_args()
    
    process_raw_csv(args.input, args.output, args.launch_lat, args.launch_lon)
```

### 5.2 Validador de Coherencia

```python
#!/usr/bin/env python3
"""
validate_coherence.py
Verifica que un CSV procesado cumple todas las reglas de coherencia física.

Uso:
    python validate_coherence.py --input processed/motor_H143_full.csv
"""

import argparse
import numpy as np
import pandas as pd
import sys

P0_HPA = 1013.25
G = 9.80665

def check_altitude_pressure(df):
    """REGLA 1: Correlación altitud-presión."""
    expected_p = P0_HPA * (1 - 2.25577e-5 * df['altitude_m']) ** 5.25588
    error = np.abs(df['pressure_hpa'] - expected_p)
    max_error = error.max()
    
    passed = max_error < 5.0  # 5 hPa tolerancia
    print(f"  Altitude-Pressure: max_error={max_error:.2f} hPa "
          f"{'PASS' if passed else 'FAIL'}")
    return passed

def check_velocity_altitude(df):
    """REGLA 2: Velocidad es derivada de altitud."""
    dt = np.diff(df['time_s'].values)
    dh = np.diff(df['altitude_m'].values)
    v_calc = dh / dt
    v_actual = df['velocity_ms'].values[1:]
    
    error = np.abs(v_calc - v_actual)
    # Promediar error sobre ventana de 5 muestras
    kernel = np.ones(5) / 5
    if len(error) > 5:
        error_smooth = np.convolve(error, kernel, mode='valid')
        max_error = error_smooth.max()
    else:
        max_error = error.max()
    
    passed = max_error < 10.0  # m/s tolerancia (drag y discretización)
    print(f"  Velocity-Altitude: max_smooth_error={max_error:.2f} m/s "
          f"{'PASS' if passed else 'FAIL'}")
    return passed

def check_apogee(df):
    """REGLA 4: Condiciones de apogeo."""
    max_alt_idx = df['altitude_m'].idxmax()
    apogee = df.loc[max_alt_idx]
    
    vel_ok = abs(apogee['velocity_ms']) < 5.0
    min_p_idx = df['pressure_hpa'].idxmin()
    pressure_ok = abs(max_alt_idx - min_p_idx) <= 10
    
    passed = vel_ok and pressure_ok
    print(f"  Apogee: vel={apogee['velocity_ms']:.2f} m/s, "
          f"alt_idx={max_alt_idx}, p_min_idx={min_p_idx} "
          f"{'PASS' if passed else 'FAIL'}")
    return passed

def check_gps_baro_correlation(df):
    """REGLA 5: GPS vs barométrico."""
    if 'gps_altitude_m' not in df.columns:
        print("  GPS-Baro: SKIP (no GPS data)")
        return True
    
    error = np.abs(df['gps_altitude_m'] - df['altitude_m'])
    max_error = error.max()
    
    passed = max_error < 20.0  # 20m tolerancia
    print(f"  GPS-Baro: max_error={max_error:.1f} m "
          f"{'PASS' if passed else 'FAIL'}")
    return passed

def run_all_checks(input_path):
    df = pd.read_csv(input_path)
    print(f"\nValidating: {input_path} ({len(df)} rows)")
    print("=" * 50)
    
    results = [
        check_altitude_pressure(df),
        check_velocity_altitude(df),
        check_apogee(df),
        check_gps_baro_correlation(df),
    ]
    
    print("=" * 50)
    if all(results):
        print("ALL CHECKS PASSED ✓")
        return 0
    else:
        print("SOME CHECKS FAILED ✗")
        return 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    args = parser.parse_args()
    sys.exit(run_all_checks(args.input))
```

---

## 6. Generación de Datos Sintéticos

Si no tienes acceso a OpenRocket, puedes generar un perfil de vuelo sintético:

```python
#!/usr/bin/env python3
"""
generate_synthetic_flight.py
Genera un perfil de vuelo sintético basado en ecuaciones de movimiento.

Uso:
    python generate_synthetic_flight.py --output raw/synthetic_E12.csv \
                                        --motor E12 \
                                        --mass 0.85
"""

import numpy as np
import pandas as pd

G = 9.80665
RHO_AIR = 1.225  # kg/m³ a nivel del mar
CD = 0.5         # Coeficiente de drag típico
A_REF = 0.001    # Área de referencia (m²), ~35mm diámetro

MOTORS = {
    'E12': {'thrust_avg': 12.0, 'burn_time': 3.5, 'prop_mass': 0.030},
    'H143': {'thrust_avg': 143.0, 'burn_time': 1.2, 'prop_mass': 0.094},
    'J280': {'thrust_avg': 280.0, 'burn_time': 2.0, 'prop_mass': 0.250},
}

def simulate_flight(motor_name, dry_mass, dt=0.01):
    motor = MOTORS[motor_name]
    
    t, h, v = 0.0, 0.0, 0.0
    mass = dry_mass + motor['prop_mass']
    
    data = []
    
    while True:
        # Thrust
        if t < motor['burn_time']:
            thrust = motor['thrust_avg']
            mass_rate = motor['prop_mass'] / motor['burn_time']
        else:
            thrust = 0.0
            mass_rate = 0.0
        
        # Drag  
        drag = 0.5 * RHO_AIR * CD * A_REF * v * abs(v)
        
        # Aceleración neta
        accel = (thrust - drag) / mass - G
        
        # Integración (Euler)
        v += accel * dt
        h += v * dt
        mass -= mass_rate * dt
        t += dt
        
        # Presión barométrica
        pressure_pa = 101325.0 * (1 - 2.25577e-5 * max(0, h)) ** 5.25588
        
        data.append({
            'time_s': round(t, 3),
            'altitude_m': max(0, h),
            'velocity_ms': v,
            'accel_vertical_ms2': accel,
            'accel_total_ms2': abs(accel + G),
            'pressure_pa': pressure_pa,
            'temperature_k': 288.15 - 0.0065 * max(0, h),
            'thrust_n': thrust,
            'mass_kg': mass,
        })
        
        # Terminar cuando aterriza
        if h <= 0 and t > motor['burn_time'] + 1.0:
            break
        
        if t > 300:  # Safety timeout
            break
    
    return pd.DataFrame(data)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--motor', default='E12', choices=MOTORS.keys())
    parser.add_argument('--mass', type=float, default=0.85, help='Dry mass (kg)')
    args = parser.parse_args()
    
    df = simulate_flight(args.motor, args.mass)
    df.to_csv(args.output, index=False, float_format='%.6f')
    print(f"Generated {len(df)} rows → {args.output}")
    print(f"  Apogee: {df['altitude_m'].max():.1f} m")
    print(f"  Max velocity: {df['velocity_ms'].max():.1f} m/s")
    print(f"  Flight time: {df['time_s'].max():.1f} s")
```

---

## 7. Flujo de Trabajo Completo

```
1. OBTENER DATOS RAW
   ├── Opción A: Exportar desde OpenRocket → raw/motor_H143.csv
   └── Opción B: Generar sintético → python generate_synthetic_flight.py

2. PROCESAR DATOS
   └── python process_flight_data.py --input raw/motor_H143.csv \
                                     --output processed/motor_H143_full.csv

3. VALIDAR COHERENCIA
   └── python validate_coherence.py --input processed/motor_H143_full.csv

4. CONVERTIR A STIMULUS (ver tools/data-converter/)
   └── python csv_to_stimulus.py --input processed/motor_H143_full.csv \
                                 --output-dir ../integration/full-system-simulation/stimulus/

5. SIMULAR EN PROTEUS
   └── Cargar stimulus files en Proteus → Ejecutar simulación

6. VALIDAR SIMULACIÓN
   └── python validate_simulation.py --telemetry results/telemetry.log
```

---

## Siguiente Paso

> Obtener un CSV de OpenRocket (o generar uno sintético) y ejecutar el pipeline completo para tener datos listos para la primera simulación.
