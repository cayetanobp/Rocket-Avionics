# Módulo 06: GNSS GPS - Sistema de Posicionamiento Global

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Séptimo módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core, 03-Sensors Interface  
> **Dependientes**: 07-Telemetry, 08-Data Logging, 11-State Machine (RECOVERY_MODE)

---

## Resumen

Receptor GPS NEO-6M para posicionamiento geográfico del cohete. Aunque **no es crítico durante el vuelo activo** (la altitud barométrica y la IMU dominan las decisiones de vuelo), es **esencial para la fase de recuperación**: transmite la posición de aterrizaje por telemetría para localizar el cohete. También proporciona altitud GPS como referencia cruzada y velocidad sobre suelo.

---

## Especificaciones Técnicas

### Módulo NEO-6M
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Interfaz                 | UART (TX/RX)                    |
| Baud rate default        | 9600 baud (configurable)        |
| Voltaje operación        | 2.7V - 3.6V                    |
| Consumo                  | 45 mA (adquisición), 37 mA (tracking) |
| Time To First Fix (TTFF) | 26s (cold start), 1s (hot start)|
| Precisión posición       | 2.5 m CEP (horizontal)          |
| Precisión altitud        | ~5 m (mucho peor que barómetro) |
| Velocidad máxima         | 500 m/s (limitación ITAR)       |
| Altitud máxima           | 50 km (limitación ITAR)         |
| Update rate              | 1 Hz default (configurable hasta 5 Hz) |
| Protocolo                | NMEA 0183 (GGA, RMC, GSV, GSA) |
| Modelo Proteus           | ⚠️ No nativo                    |

### Tramas NMEA Utilizadas
| Trama   | Datos                                    | Prioridad |
|---------|------------------------------------------|-----------|
| `$GPGGA`| Posición, altitud, fix, satélites, HDOP  | Alta      |
| `$GPRMC`| Posición, velocidad, curso, fecha/hora   | Alta      |
| `$GPGSA`| Modo de fix, DOP, satélites en uso       | Media     |
| `$GPGSV`| Satélites en vista, SNR                  | Baja      |

---

## Decisiones Técnicas

### ¿Cómo simular GPS en Proteus si no hay modelo nativo?

**Método elegido: MCU auxiliar como generador NMEA**

```
┌────────────────────────┐         ┌──────────────────────┐
│  Arduino/STM32 auxiliar │  UART   │   STM32F401          │
│  "GPS Simulator"        │────────▶│   Flight Computer    │
│                          │         │                      │
│  Genera tramas NMEA      │         │  Parsea NMEA como    │
│  sincronizadas con       │         │  si fuera NEO-6M     │
│  el perfil de vuelo      │         │  real                │
└────────────────────────┘         └──────────────────────┘
```

- El MCU auxiliar tiene precargadas las coordenadas interpoladas del perfil de vuelo
- Envía tramas GGA y RMC a 1 Hz por UART a 9600 baud
- El firmware del flight computer **no sabe que no es un GPS real** - parsea exactamente igual
- **Alternativa**: Inyectar tramas directamente desde Virtual Terminal de Proteus (manual)

### ¿Por qué 9600 baud y no 115200?
- 9600 es el baud rate **default de fábrica** del NEO-6M
- A 9600 baud, una trama GGA (~82 chars) toma ~85 ms - OK para 1 Hz update
- Cambiar el baud rate requiere enviar comandos UBX al GPS - complejidad innecesaria
- **Para vuelo real**: Se podría subir a 38400 para 5 Hz update, pero 1 Hz es suficiente

### ¿Por qué solo parsear GGA y RMC?
- **GGA**: Tiene todo lo necesario - posición, altitud, fix, satélites, HDOP
- **RMC**: Añade velocidad sobre suelo y curso - útil para tracking
- Las demás tramas (GSV, GSA, VTG) aportan info secundaria que no afecta al vuelo
- Parsear menos tramas = menos CPU = más margen para tareas críticas

### ¿Por qué el GPS no se usa para detección de apogeo?
- **Latencia GPS**: 1 segundo de update rate → 1000 ms de delay
- **Latencia barómetro**: 10ms de sample rate + 30ms de filtro → 40ms total
- **Requisito**: Detección de apogeo < 50 ms - el GPS es 20× más lento
- **Precisión altitud GPS**: ±5 m vs ±1 m del barómetro
- El GPS se usa como **referencia cruzada** y para **recovery tracking**

---

## Diseño del Circuito

### Esquemático (con GPS real)

```
                    3.3V
                     │
                   100nF
                     │
              ┌──────┴──────┐
              │   NEO-6M    │
    PA10 ─────┤ TX      VCC ├──── 3.3V
    PA9  ─────┤ RX      GND ├──── GND
              │         PPS ├──── PB2 (opcional, 1PPS timing)
              └──────┬──────┘
                     │
                  Antena cerámica
```

### Esquemático (Simulación con MCU auxiliar)

```
              ┌──────────────────┐
              │ Arduino Nano     │
              │ (GPS Simulator)  │
    PA10 ─────┤ TX (D1)    VCC  ├──── 3.3V/5V
              │            GND  ├──── GND
              │                 │
              │ Cargado con     │
              │ flight_gps.h    │
              │ (tramas NMEA    │
              │  pre-generadas) │
              └──────────────────┘
```

---

## Desarrollo Step-by-Step

### Paso 1: Parser NMEA - GGA
```c
// Ejemplo de trama GGA:
// $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47

typedef struct {
    float latitude;      // Decimal degrees (+ = N, - = S)
    float longitude;     // Decimal degrees (+ = E, - = W)
    float altitude_msl;  // Metros sobre nivel del mar
    uint8_t fix_quality; // 0=invalid, 1=GPS, 2=DGPS
    uint8_t satellites;  // Número de satélites
    float hdop;          // Horizontal Dilution of Precision
    uint8_t hours, minutes, seconds;
    bool valid;
} nmea_gga_t;

bool nmea_parse_gga(const char *sentence, nmea_gga_t *gga) {
    if (strncmp(sentence, "$GPGGA", 6) != 0) return false;
    
    char *fields[15];
    uint8_t field_count = nmea_split_fields(sentence, fields, 15);
    if (field_count < 14) return false;
    
    // UTC Time: HHMMSS.ss
    gga->hours   = atoi_n(fields[1], 2);
    gga->minutes = atoi_n(fields[1]+2, 2);
    gga->seconds = atoi_n(fields[1]+4, 2);
    
    // Latitude: DDMM.MMMM,N/S
    float lat_raw = atof(fields[2]);
    int lat_deg = (int)(lat_raw / 100);
    float lat_min = lat_raw - (lat_deg * 100);
    gga->latitude = lat_deg + (lat_min / 60.0f);
    if (fields[3][0] == 'S') gga->latitude = -gga->latitude;
    
    // Longitude: DDDMM.MMMM,E/W
    float lon_raw = atof(fields[4]);
    int lon_deg = (int)(lon_raw / 100);
    float lon_min = lon_raw - (lon_deg * 100);
    gga->longitude = lon_deg + (lon_min / 60.0f);
    if (fields[5][0] == 'W') gga->longitude = -gga->longitude;
    
    // Fix quality
    gga->fix_quality = atoi(fields[6]);
    gga->satellites = atoi(fields[7]);
    gga->hdop = atof(fields[8]);
    gga->altitude_msl = atof(fields[9]);
    
    gga->valid = (gga->fix_quality > 0);
    return true;
}
```

### Paso 2: Parser NMEA - RMC
```c
// $GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

typedef struct {
    float latitude, longitude;
    float speed_knots;    // Velocidad sobre suelo
    float course_deg;     // Curso (dirección de movimiento)
    uint8_t day, month, year;
    bool valid;           // A=active, V=void
} nmea_rmc_t;

bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *rmc);
```

### Paso 3: Validación de Checksum NMEA
```c
// El checksum NMEA es XOR de todos los bytes entre $ y *
bool nmea_verify_checksum(const char *sentence) {
    if (sentence[0] != '$') return false;
    
    uint8_t calc_checksum = 0;
    const char *p = sentence + 1;
    while (*p && *p != '*') {
        calc_checksum ^= *p;
        p++;
    }
    
    if (*p != '*') return false;
    uint8_t recv_checksum = strtol(p + 1, NULL, 16);
    
    return (calc_checksum == recv_checksum);
}
```

### Paso 4: Buffer Circular UART + DMA
```c
#define GPS_DMA_BUF_SIZE 512
static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t gps_read_pos = 0;

// Línea completa buffer para ensamblar tramas
#define GPS_LINE_BUF_SIZE 128
static char gps_line_buf[GPS_LINE_BUF_SIZE];
static uint8_t gps_line_pos = 0;

void gps_process_dma_buffer(void) {
    uint16_t write_pos = GPS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    
    while (gps_read_pos != write_pos) {
        char c = gps_dma_buf[gps_read_pos];
        gps_read_pos = (gps_read_pos + 1) % GPS_DMA_BUF_SIZE;
        
        if (c == '\n') {
            gps_line_buf[gps_line_pos] = '\0';
            gps_process_sentence(gps_line_buf);  // Parsear trama completa
            gps_line_pos = 0;
        } else if (c != '\r' && gps_line_pos < GPS_LINE_BUF_SIZE - 1) {
            gps_line_buf[gps_line_pos++] = c;
        }
    }
}
```

### Paso 5: Tarea RTOS
```c
void vTaskGPS(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);  // 10 Hz check
    
    // Iniciar DMA circular
    HAL_UART_Receive_DMA(&huart1, gps_dma_buf, GPS_DMA_BUF_SIZE);
    printf("[GPS] UART DMA started, waiting for fix...\n");
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        gps_process_dma_buffer();
        
        gps_data_t data = gps_get_latest();
        xQueueOverwrite(gps_data_q, &data);
        
        // Log periódico
        static uint8_t log_counter = 0;
        if (++log_counter >= 10) {  // Cada 1 segundo
            log_counter = 0;
            if (data.fix > 0) {
                printf("[GPS] Fix:%d Sats:%d Lat:%.6f Lon:%.6f Alt:%.1fm\n",
                       data.fix, data.satellites, data.latitude, 
                       data.longitude, data.altitude_gps);
            } else {
                printf("[GPS] No fix, searching...\n");
            }
        }
    }
}
```

### Paso 6: Simulador GPS (MCU auxiliar)
```c
// Código para Arduino que simula el NEO-6M
// Se carga en el MCU auxiliar dentro de Proteus

#include "flight_gps_data.h"  // Array de coordenadas pre-generado

// flight_gps_data.h contiene:
// const gps_point_t flight_path[] = {
//   {0.0, 40.4168, -3.7038, 0.0,   0.0},    // t=0, pad
//   {1.0, 40.4168, -3.7038, 50.0,  180.0},   // t=1, boost
//   {2.0, 40.4168, -3.7038, 200.0, 350.0},   // t=2, boost
//   ...
// };

void loop() {
    static uint16_t idx = 0;
    
    if (idx < FLIGHT_PATH_LENGTH) {
        char nmea[128];
        generate_gpgga(nmea, flight_path[idx]);
        Serial.println(nmea);
        
        generate_gprmc(nmea, flight_path[idx]);
        Serial.println(nmea);
        
        idx++;
    }
    
    delay(1000);  // 1 Hz update rate
}
```

---

## Correlación de Datos en Simulación

### GPS vs Barómetro - Altitud

| Instante    | Alt. Barómetro (m) | Alt. GPS (m) | Diferencia | Nota                     |
|-------------|---------------------|---------------|------------|--------------------------|
| PAD_IDLE    | 0.0                 | 650 (MSL)     | N/A        | Barómetro = AGL, GPS = MSL |
| Apogeo      | 800.0               | 1450          | ~0 AGL     | 800 AGL = 650+800 MSL   |
| Main deploy | 300.0               | 950           | ~0 AGL     | Coherente                |
| Landed      | 0.0                 | 650           | ~0 AGL     | De vuelta al inicio      |

### Generación de Coordenadas de Vuelo
```python
# El cohete sube casi vertical → las coordenadas apenas cambian
# Solo se desplaza ligeramente por viento

def generate_gps_track(base_lat, base_lon, base_alt_msl, 
                       flight_profile, wind_speed_ms=5, wind_dir_deg=270):
    """
    Genera coordenadas GPS a partir del perfil de vuelo.
    El desplazamiento horizontal es mínimo (vuelo casi vertical).
    """
    track = []
    for t, alt_agl, vspeed in flight_profile:
        # Desplazamiento por viento (simplificado)
        drift_m = wind_speed_ms * t * 0.1  # Factor de arrastre
        dlat = drift_m * math.cos(math.radians(wind_dir_deg)) / 111320
        dlon = drift_m * math.sin(math.radians(wind_dir_deg)) / (111320 * math.cos(math.radians(base_lat)))
        
        track.append({
            'time': t,
            'lat': base_lat + dlat,
            'lon': base_lon + dlon,
            'alt_msl': base_alt_msl + alt_agl,
            'speed_kmh': abs(vspeed) * 3.6
        })
    
    return track
```

---

## API del Módulo

```c
// gnss_gps.h

// Inicialización
HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart);

// Lectura de últimos datos válidos
gps_data_t gps_get_data(void);

// Estado
bool gps_has_fix(void);
uint8_t gps_get_satellite_count(void);
float gps_get_hdop(void);

// Para recovery mode
float gps_distance_to(float lat2, float lon2);  // Haversine, metros
float gps_bearing_to(float lat2, float lon2);    // Grados

// Estadísticas
uint32_t gps_get_valid_sentence_count(void);
uint32_t gps_get_checksum_error_count(void);
```

---

## Criterios de Aceptación

- [ ] Parser GGA extrae todos los campos correctamente
- [ ] Parser RMC extrae velocidad y curso correctamente
- [ ] Checksum NMEA se valida correctamente
- [ ] Tramas con checksum inválido se descartan
- [ ] DMA circular no pierde bytes a 9600 baud
- [ ] Fix status se reporta correctamente (0/1/2)
- [ ] Coordenadas del simulador coinciden con el perfil de vuelo
- [ ] Altitud GPS coherente con altitud barométrica (±50 m)
- [ ] En RECOVERY_MODE, posición se transmite cada 5 s
- [ ] Sin memory leaks en parsing de tramas

---

## Referencias

- [NEO-6M Datasheet](https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf)
- [NMEA 0183 Specification](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
- [UBX Protocol Reference](https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf)

---

*Módulo 06 - El localizador. No importa dónde caiga, siempre sabremos dónde está.*
