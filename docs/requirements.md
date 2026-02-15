# Requisitos del Sistema - Rocket Avionics

## Alcance

Sistema de aviónica completo para cohete sounding amateur con simulación full-stack en Proteus VSM, alimentado con datos de vuelo reales para validación físicamente coherente.

---

## Requisitos Funcionales

### RF-01: Adquisición de Datos Inerciales
- **RF-01.1**: El sistema DEBE leer aceleración en 3 ejes a ≥100 Hz
- **RF-01.2**: El sistema DEBE leer velocidad angular en 3 ejes a ≥100 Hz
- **RF-01.3**: El sistema DEBE calcular orientación (pitch, roll, yaw) mediante fusión de sensores
- **RF-01.4**: El filtro Madgwick DEBE converger en < 2 segundos desde el arranque

### RF-02: Medición de Altitud
- **RF-02.1**: El sistema DEBE medir presión barométrica a ≥50 Hz
- **RF-02.2**: El sistema DEBE calcular altitud AGL con resolución de ±1 metro
- **RF-02.3**: El sistema DEBE calcular velocidad vertical con resolución de ±0.5 m/s
- **RF-02.4**: El sistema DEBE detectar el apogeo con latencia < 50 ms

### RF-03: Posicionamiento GPS
- **RF-03.1**: El sistema DEBE parsear tramas NMEA GGA y RMC
- **RF-03.2**: El sistema DEBE reportar posición con frecuencia ≥1 Hz
- **RF-03.3**: El sistema DEBE reportar calidad de fix (sin fix, 2D, 3D)
- **RF-03.4**: En modo RECOVERY, el GPS DEBE transmitir posición cada 5 segundos

### RF-04: Máquina de Estados
- **RF-04.1**: El sistema DEBE implementar 10 estados de vuelo (PAD_IDLE → RECOVERY_MODE)
- **RF-04.2**: Las transiciones DEBEN ser deterministas y basadas en datos de sensores
- **RF-04.3**: ARMED → BOOST requiere aceleración > 2g sostenida por 100 ms
- **RF-04.4**: COAST → APOGEE requiere cambio de signo en velocidad vertical
- **RF-04.5**: DESCENT_DROGUE → MAIN_DEPLOY requiere altitud < umbral configurable
- **RF-04.6**: LANDED requiere velocidad vertical ≈ 0 durante > 3 segundos

### RF-05: Sistema de Recuperación
- **RF-05.1**: El sistema DEBE controlar al menos 2 canales pirotécnicos independientes
- **RF-05.2**: El drogue chute DEBE desplegarse automáticamente en APOGEE
- **RF-05.3**: El main chute DEBE desplegarse a altitud configurable (default: 300 m AGL)
- **RF-05.4**: El sistema DEBE verificar continuidad de e-matches antes del arming
- **RF-05.5**: El sistema DEBE tener un arming switch físico como seguridad

### RF-06: Telemetría
- **RF-06.1**: El sistema DEBE transmitir paquetes de telemetría a ≥5 Hz durante vuelo
- **RF-06.2**: Cada paquete DEBE incluir: timestamp, estado FSM, altitud, velocidad vertical, orientación, GPS, batería
- **RF-06.3**: Los paquetes DEBEN incluir checksum CRC-16 para integridad
- **RF-06.4**: En RECOVERY_MODE, el sistema DEBE transmitir posición GPS cada 5 s

### RF-07: Data Logging
- **RF-07.1**: El sistema DEBE almacenar datos de todos los sensores a ≥100 Hz
- **RF-07.2**: El sistema DEBE usar timestamps sincronizados (ms desde boot)
- **RF-07.3**: El sistema DEBE marcar eventos críticos (transiciones de estado, despliegues)
- **RF-07.4**: El buffer circular en RAM DEBE ser ≥4 KB para absorber picos de escritura

### RF-08: Gestión de Energía
- **RF-08.1**: El sistema DEBE monitorizar voltaje y corriente de batería
- **RF-08.2**: El sistema DEBE generar alarma cuando V_bat < 10.0 V (3S LiPo)
- **RF-08.3**: El sistema DEBE ejecutar deploy de emergencia si V_bat < 9.0 V en vuelo
- **RF-08.4**: Los reguladores DEBEN soportar picos de 2A (5V) y 500mA (3.3V)

### RF-09: Diagnósticos
- **RF-09.1**: El sistema DEBE realizar health check de todos los sensores al arranque
- **RF-09.2**: El sistema DEBE reportar errores categorizados (INFO, WARN, ERROR, CRITICAL)
- **RF-09.3**: El watchdog DEBE reiniciar el MCU si una tarea no responde en 1 segundo
- **RF-09.4**: El sistema DEBE detectar stack overflow en tareas RTOS

### RF-10: Control de Vuelo (Opcional/Avanzado)
- **RF-10.1**: El controlador PID DEBE estabilizar pitch y yaw durante BOOST y COAST
- **RF-10.2**: Los servos DEBEN operar a 50 Hz con pulsos de 1000-2000 µs
- **RF-10.3**: La deflexión DEBE limitarse a ±15° para seguridad

---

## Requisitos No Funcionales

### RNF-01: Rendimiento
- **RNF-01.1**: Latencia máxima de detección de apogeo: 50 ms
- **RNF-01.2**: Jitter de tareas críticas (IMU, BARO): < 1 ms
- **RNF-01.3**: El sistema DEBE mantener 100 Hz de muestreo sostenido sin pérdida de datos
- **RNF-01.4**: Throughput de telemetría: ≥5 paquetes/segundo sin pérdida

### RNF-02: Fiabilidad
- **RNF-02.1**: El RTOS DEBE ejecutar sin crashes durante ≥10 minutos de simulación continua
- **RNF-02.2**: Detección redundante de apogeo: barómetro AND/OR IMU
- **RNF-02.3**: Timeout de deploy de emergencia por tiempo (backup timer)
- **RNF-02.4**: Recuperación automática ante fallo de sensor individual

### RNF-03: Simulabilidad
- **RNF-03.1**: Todos los componentes DEBEN tener modelos VSM disponibles en Proteus
- **RNF-03.2**: Los datos de simulación DEBEN ser físicamente coherentes (correlación multi-sensor)
- **RNF-03.3**: Cada módulo DEBE ser simulable de forma independiente
- **RNF-03.4**: El sistema integrado DEBE completar un perfil de vuelo de ≥120 segundos

### RNF-04: Mantenibilidad
- **RNF-04.1**: Código modular con interfaces claras entre subsistemas
- **RNF-04.2**: Documentación técnica por cada módulo
- **RNF-04.3**: Control de versiones con commits descriptivos
- **RNF-04.4**: Código comentado con estándar Doxygen

---

## Requisitos de Simulación

### RS-01: Datos de Entrada
- **RS-01.1**: Los datos de vuelo DEBEN provenir de OpenRocket o registros de vuelos reales
- **RS-01.2**: El CSV de entrada DEBE contener: tiempo, altitud, velocidad, aceleración, presión
- **RS-01.3**: Los datos DEBEN convertirse a valores de registro I2C/SPI correspondientes a cada sensor
- **RS-01.4**: Las tramas GPS DEBEN generarse con coordenadas coherentes con el perfil de vuelo

### RS-02: Correlación de Datos
- **RS-02.1**: En el apogeo: velocidad_vertical = 0, presión = mínima, aceleración = -1g
- **RS-02.2**: En BOOST: aceleración > 2g, presión decreciente, altitud creciente
- **RS-02.3**: En DESCENT: altitud decreciente, presión creciente, velocidad_vertical < 0
- **RS-02.4**: GPS altitud DEBE ser coherente con altitud barométrica (±50m de diferencia aceptable)

### RS-03: Validación
- **RS-03.1**: La FSM DEBE transicionar correctamente con datos reales inyectados
- **RS-03.2**: El drogue DEBE desplegarse dentro de 50ms del cruce de apogeo en datos
- **RS-03.3**: El main DEBE desplegarse al cruzar la altitud configurada en descenso
- **RS-03.4**: Los datos loggeados DEBEN coincidir con los datos inyectados (±tolerancia del sensor)

---

## Matriz de Trazabilidad

| Módulo               | Requisitos Cubiertos                    |
|----------------------|-----------------------------------------|
| 01-Power Management  | RF-08, RNF-02                           |
| 02-MCU Core          | RNF-01, RNF-02, RNF-04                  |
| 03-Sensors Interface | RF-01, RF-02, RF-03                      |
| 04-IMU Navigation    | RF-01, RNF-01.2                          |
| 05-Barometer         | RF-02, RS-02.1                           |
| 06-GNSS GPS          | RF-03, RS-01.4                           |
| 07-Telemetry Radio   | RF-06, RNF-01.4                          |
| 08-Data Logging      | RF-07, RNF-01.3                          |
| 09-Recovery System   | RF-05, RNF-02.2, RNF-02.3               |
| 10-Flight Control    | RF-10                                    |
| 11-State Machine     | RF-04, RS-03.1                           |
| 12-Diagnostics       | RF-09, RNF-02.4                          |

---

*Cada requisito se valida con tests específicos documentados en el README de cada módulo.*
