# Módulo 08: Data Logging - Registro de Datos de Vuelo

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Octavo módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core, 03-Sensors Interface  
> **Dependientes**: Post-vuelo análisis, validación de simulación

---

## Resumen

Sistema de almacenamiento en memoria Flash SPI (W25Q128, 16 MB) para registro de alta frecuencia (100 Hz) de todos los datos de sensores y eventos del vuelo. A diferencia de la telemetría (que puede perder paquetes), el data logger es la **fuente de verdad**: registra absolutamente todo lo que ocurrió durante el vuelo para análisis posterior.

---

## Especificaciones Técnicas

### Memoria W25Q128 (Flash SPI)
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Interfaz                 | SPI (Mode 0, CPOL=0, CPHA=0)   |
| Chip Select              | PA4 (GPIO, active low)          |
| Capacidad                | 16 MB (128 Mbit)               |
| Organización             | 256 pages × 256 bytes = 65536 pages |
| Tamaño de page           | 256 bytes                       |
| Tamaño de sector         | 4 KB (16 pages)                 |
| Tamaño de bloque         | 64 KB (256 pages)               |
| Velocidad SPI            | Hasta 104 MHz                   |
| Page Program time        | 0.7 ms típico, 3 ms máximo     |
| Sector Erase time        | 45 ms típico, 400 ms máximo    |
| Voltaje operación        | 2.7V - 3.6V                    |
| Consumo (write)          | 15 mA                          |
| Consumo (standby)        | 1 µA                           |
| Endurance                | 100,000 erase cycles            |
| Modelo Proteus           | ✅ Disponible (SPI)             |

### Capacidad de Logging
| Parámetro                | Cálculo                         |
|--------------------------|---------------------------------|
| Tamaño de registro       | 32 bytes por muestra            |
| Frecuencia de logging    | 100 Hz                          |
| Bytes por segundo         | 3,200 B/s                      |
| Bytes por minuto          | 192,000 B/min                  |
| Capacidad total           | 16,777,216 bytes                |
| **Duración de logging**   | **~87 minutos** (más que suficiente) |
| Vuelo típico             | ~120 s → usa ~384 KB (2.3%)    |

---

## Decisiones Técnicas

### ¿Por qué Flash SPI y no tarjeta microSD?
| Criterio              | W25Q128 (Flash SPI)| MicroSD + FatFs    |
|-----------------------|--------------------|--------------------|
| **Modelo Proteus**    | ✅ Sí              | ⚠️ Parcial         |
| Complejidad firmware  | Baja (SPI directo) | Alta (FatFs stack)  |
| Tiempo de escritura   | 0.7 ms (page)      | 5-250 ms (variable!)|
| RAM necesaria         | ~256 bytes buffer   | ~4 KB (FatFs)       |
| Determinismo          | Alto                | Bajo (wear leveling)|
| Vibración             | Inmune              | Socket puede fallar |

**Decisión**: W25Q128 - determinismo de escritura crucial para logging a 100 Hz. Una microSD puede bloquearse 250 ms durante wear leveling, perdiendo 25 muestras. La Flash SPI escribe en 0.7 ms garantizados.

### ¿Por qué formato binario y no CSV?
- **Binario**: 32 bytes por registro, eficiente, rápido de escribir
- **CSV**: ~150 bytes por registro (texto), 5× más espacio, overhead de sprintf
- La conversión binario → CSV se hace **post-vuelo** con script Python
- 32 bytes caben exactamente en una operación FIFO → zero-copy posible

### ¿Por qué 100 Hz y no 10 Hz?
- A 10 Hz, eventos rápidos (apogee, deploy) pueden caer entre muestras
- A 100 Hz, resolución temporal de 10 ms = captura precisa de transiciones
- El W25Q128 puede sostener 100 Hz sin problema: 32 bytes × 100 = 3.2 KB/s << capacidad SPI

### Estrategia de Escritura: Page Buffer
```
RAM Buffer (256 bytes = 8 registros)
     │
     │  Se llena cada 80 ms (8 × 10ms)
     │
     ▼
Flash Write (page program, 256 bytes, 0.7ms)
     │
     ▼
Siguiente page address
```
- Acumulamos 8 registros en RAM buffer (256 bytes = 1 page)
- Cuando el buffer está lleno, escribimos 1 page completa
- Page write = 0.7 ms → no bloquea la tarea significativamente

---

## Diseño del Circuito

### Esquemático

```
                    3.3V
                     │
                   100nF
                     │
              ┌──────┴──────┐
    PA5 ──────┤ CLK     VCC ├──── 3.3V
    PA7 ──────┤ DI      /WP ├──── 3.3V (write protect disabled)
    PA6 ──────┤ DO    /HOLD ├──── 3.3V (hold disabled)
    PA4 ──────┤ /CS     GND ├──── GND
              └─────────────┘
                 W25Q128
```

### Componentes

| Ref  | Componente     | Valor        | Notas                          |
|------|----------------|--------------|--------------------------------|
| U6   | W25Q128FV      | SOIC-8       | SPI Flash 16MB                 |
| C14  | Bypass cap     | 100nF        | Cerámico, junto a VCC          |

> **Nota**: /WP y /HOLD se conectan a VCC para deshabilitarlos (escritura siempre permitida, sin hold).

---

## Desarrollo Step-by-Step

### Paso 1: Driver SPI del W25Q128
```c
#define FLASH_CS_PORT  GPIOA
#define FLASH_CS_PIN   GPIO_PIN_4

// Comandos W25Q128
#define W25Q_CMD_WRITE_ENABLE   0x06
#define W25Q_CMD_WRITE_DISABLE  0x04
#define W25Q_CMD_READ_STATUS1   0x05
#define W25Q_CMD_READ_DATA      0x03
#define W25Q_CMD_PAGE_PROGRAM   0x02
#define W25Q_CMD_SECTOR_ERASE   0x20
#define W25Q_CMD_CHIP_ERASE     0xC7
#define W25Q_CMD_READ_ID        0x9F
#define W25Q_CMD_POWER_DOWN     0xB9
#define W25Q_CMD_RELEASE_PD     0xAB

// Leer Manufacturer/Device ID
uint32_t flash_read_id(void) {
    uint8_t tx[4] = {W25Q_CMD_READ_ID, 0, 0, 0};
    uint8_t rx[4];
    spi_select(FLASH_CS_PORT, FLASH_CS_PIN);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 100);
    spi_deselect(FLASH_CS_PORT, FLASH_CS_PIN);
    // Winbond W25Q128: Manufacturer=0xEF, Device=0x4018
    return (rx[1] << 16) | (rx[2] << 8) | rx[3];
}
```

### Paso 2: Operaciones Básicas
```c
// Esperar a que la Flash termine una operación
void flash_wait_busy(void) {
    uint8_t status;
    do {
        uint8_t tx[2] = {W25Q_CMD_READ_STATUS1, 0};
        uint8_t rx[2];
        spi_select(FLASH_CS_PORT, FLASH_CS_PIN);
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
        spi_deselect(FLASH_CS_PORT, FLASH_CS_PIN);
        status = rx[1];
    } while (status & 0x01);  // BUSY bit
}

// Habilitar escritura (antes de cada write/erase)
void flash_write_enable(void) {
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
    spi_select(FLASH_CS_PORT, FLASH_CS_PIN);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    spi_deselect(FLASH_CS_PORT, FLASH_CS_PIN);
}

// Escribir una page (max 256 bytes)
void flash_page_program(uint32_t address, const uint8_t *data, uint16_t length) {
    flash_write_enable();
    
    uint8_t cmd[4] = {
        W25Q_CMD_PAGE_PROGRAM,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    
    spi_select(FLASH_CS_PORT, FLASH_CS_PIN);
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, length, 100);
    spi_deselect(FLASH_CS_PORT, FLASH_CS_PIN);
    
    flash_wait_busy();  // ~0.7ms
}

// Leer datos
void flash_read(uint32_t address, uint8_t *data, uint16_t length) {
    uint8_t cmd[4] = {
        W25Q_CMD_READ_DATA,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    
    spi_select(FLASH_CS_PORT, FLASH_CS_PIN);
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Receive(&hspi1, data, length, 100);
    spi_deselect(FLASH_CS_PORT, FLASH_CS_PIN);
}

// Borrar sector (4KB) - necesario antes de escribir si ya fue escrito
void flash_sector_erase(uint32_t address) {
    flash_write_enable();
    
    uint8_t cmd[4] = {
        W25Q_CMD_SECTOR_ERASE,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    
    spi_select(FLASH_CS_PORT, FLASH_CS_PIN);
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    spi_deselect(FLASH_CS_PORT, FLASH_CS_PIN);
    
    flash_wait_busy();  // ~45ms!
}
```

### Paso 3: Formato de Registro
```c
// Registro de datos de vuelo (32 bytes exactos = 8 por page)
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;     // 4 bytes - ms desde boot
    uint8_t  flight_state;     // 1 byte  - estado FSM
    uint8_t  flags;            // 1 byte  - bits de evento
    int16_t  accel_x;          // 2 bytes - ×100 m/s²
    int16_t  accel_y;          // 2 bytes - ×100 m/s²
    int16_t  accel_z;          // 2 bytes - ×100 m/s²
    int16_t  gyro_x;           // 2 bytes - ×10 °/s
    int16_t  gyro_y;           // 2 bytes - ×10 °/s
    int16_t  gyro_z;           // 2 bytes - ×10 °/s
    int16_t  pitch;            // 2 bytes - ×10 °
    int16_t  roll;             // 2 bytes - ×10 °
    uint32_t pressure;         // 4 bytes - Pa ×100
    int16_t  altitude;         // 2 bytes - ×10 m (max ±3276m)
    int16_t  vspeed;           // 2 bytes - ×100 m/s
    uint16_t battery_mv;       // 2 bytes - mV
} log_record_t;

_Static_assert(sizeof(log_record_t) == 32, "Record must be 32 bytes");

// Flags de evento (bit field)
#define LOG_FLAG_LAUNCH     (1 << 0)
#define LOG_FLAG_BURNOUT    (1 << 1)
#define LOG_FLAG_APOGEE     (1 << 2)
#define LOG_FLAG_DROGUE     (1 << 3)
#define LOG_FLAG_MAIN       (1 << 4)
#define LOG_FLAG_LANDED     (1 << 5)
#define LOG_FLAG_ERROR      (1 << 6)
#define LOG_FLAG_GPS_FIX    (1 << 7)
```

### Paso 4: Page Buffer y Escritura
```c
#define RECORDS_PER_PAGE    (256 / sizeof(log_record_t))  // 8
#define FLASH_LOG_START     0x000000  // Inicio del área de logging
#define FLASH_LOG_END       0xFFFFFF  // 16 MB

static log_record_t page_buffer[RECORDS_PER_PAGE];
static uint8_t buffer_idx = 0;
static uint32_t write_address = FLASH_LOG_START;
static uint32_t records_written = 0;

void logger_add_record(const log_record_t *record) {
    page_buffer[buffer_idx++] = *record;
    
    if (buffer_idx >= RECORDS_PER_PAGE) {
        // Page completa, escribir a Flash
        if (write_address < FLASH_LOG_END) {
            // Erase sector si estamos al inicio de uno (cada 4KB = 16 pages)
            if ((write_address & 0xFFF) == 0) {
                flash_sector_erase(write_address);
            }
            
            flash_page_program(write_address, (uint8_t*)page_buffer, 256);
            write_address += 256;
            records_written += RECORDS_PER_PAGE;
        }
        buffer_idx = 0;
    }
}

// Flush: escribir datos parciales en el buffer (al aterrizar)
void logger_flush(void) {
    if (buffer_idx > 0 && write_address < FLASH_LOG_END) {
        if ((write_address & 0xFFF) == 0) {
            flash_sector_erase(write_address);
        }
        flash_page_program(write_address, (uint8_t*)page_buffer, buffer_idx * sizeof(log_record_t));
        write_address += 256;
        records_written += buffer_idx;
        buffer_idx = 0;
    }
}
```

### Paso 5: Tarea RTOS
```c
void vTaskLogging(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 100 Hz
    
    // Verificar Flash
    uint32_t id = flash_read_id();
    if ((id >> 16) == 0xEF) {
        printf("[LOG] W25Q128 detected, ID=0x%06X\n", (unsigned int)id);
    } else {
        printf("[LOG] ERROR: Flash not found!\n");
        vTaskSuspend(NULL);
    }
    
    // Borrar área de logging (en producción: smart indexing)
    printf("[LOG] Erasing flash for new flight...\n");
    // Para simulación: solo borrar primeros sectores necesarios
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        // Construir registro desde datos actuales
        log_record_t rec;
        imu_data_t imu;
        baro_data_t baro;
        power_data_t power;
        
        xQueuePeek(imu_data_q, &imu, 0);
        xQueuePeek(baro_data_q, &baro, 0);
        xQueuePeek(power_data_q, &power, 0);
        
        rec.timestamp_ms = HAL_GetTick();
        rec.flight_state = fsm_get_state();
        rec.flags = 0;  // Se setean por eventos
        rec.accel_x = (int16_t)(imu.accel_x * 100);
        rec.accel_y = (int16_t)(imu.accel_y * 100);
        rec.accel_z = (int16_t)(imu.accel_z * 100);
        rec.gyro_x = (int16_t)(imu.gyro_x * 10);
        rec.gyro_y = (int16_t)(imu.gyro_y * 10);
        rec.gyro_z = (int16_t)(imu.gyro_z * 10);
        rec.pitch = (int16_t)(imu.pitch * 10);
        rec.roll = (int16_t)(imu.roll * 10);
        rec.pressure = (uint32_t)(baro.pressure * 100);
        rec.altitude = (int16_t)(baro.altitude * 10);
        rec.vspeed = (int16_t)(baro.vspeed * 100);
        rec.battery_mv = (uint16_t)(power.bus_voltage * 1000);
        
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            logger_add_record(&rec);
            xSemaphoreGive(spi_mutex);
        }
    }
}
```

### Paso 6: Extracción Post-Vuelo
```c
// Función para descargar datos por UART después del vuelo
void logger_dump_uart(UART_HandleTypeDef *huart) {
    printf("[LOG] Dumping %lu records (%lu bytes)...\n", 
           records_written, records_written * sizeof(log_record_t));
    
    for (uint32_t addr = FLASH_LOG_START; 
         addr < write_address; 
         addr += 256) {
        uint8_t page[256];
        flash_read(addr, page, 256);
        HAL_UART_Transmit(huart, page, 256, 1000);
    }
    
    printf("[LOG] Dump complete\n");
}
```

---

## Herramienta de Conversión Post-Vuelo

```python
# tools/data-converter/flash_to_csv.py

import struct
import csv

RECORD_FORMAT = '<IBBhhhhhhhhhIhhH'  # 32 bytes, little-endian
RECORD_SIZE = 32

def decode_record(data):
    fields = struct.unpack(RECORD_FORMAT, data)
    return {
        'timestamp_ms': fields[0],
        'flight_state': fields[1],
        'flags': fields[2],
        'accel_x': fields[3] / 100.0,
        'accel_y': fields[4] / 100.0,
        'accel_z': fields[5] / 100.0,
        'gyro_x': fields[6] / 10.0,
        'gyro_y': fields[7] / 10.0,
        'gyro_z': fields[8] / 10.0,
        'pitch': fields[9] / 10.0,
        'roll': fields[10] / 10.0,
        'pressure_pa': fields[11] / 100.0,
        'altitude_m': fields[12] / 10.0,
        'vspeed_ms': fields[13] / 100.0,
        'battery_v': fields[14] / 1000.0,
    }

def flash_dump_to_csv(binary_file, csv_file):
    with open(binary_file, 'rb') as f, open(csv_file, 'w', newline='') as csvf:
        writer = None
        while True:
            data = f.read(RECORD_SIZE)
            if len(data) < RECORD_SIZE:
                break
            record = decode_record(data)
            if writer is None:
                writer = csv.DictWriter(csvf, fieldnames=record.keys())
                writer.writeheader()
            writer.writerow(record)
```

---

## API del Módulo

```c
// data_logging.h

// Inicialización
HAL_StatusTypeDef logger_init(SPI_HandleTypeDef *hspi);

// Logging
void logger_add_record(const log_record_t *record);
void logger_mark_event(uint8_t flag);  // Setear flag en próximo registro
void logger_flush(void);               // Escribir buffer parcial

// Estado
uint32_t logger_get_records_written(void);
uint32_t logger_get_bytes_used(void);
uint32_t logger_get_bytes_free(void);
bool     logger_is_full(void);

// Extracción
void logger_dump_uart(UART_HandleTypeDef *huart);

// Mantenimiento
void logger_erase_all(void);           // Chip erase (~25s!)
```

---

## Criterios de Aceptación

- [ ] W25Q128 detectado (Manufacturer ID = 0xEF)
- [ ] Page write de 256 bytes completa en < 3 ms
- [ ] Sector erase completa en < 400 ms
- [ ] 100 Hz de logging sostenido sin pérdida de registros
- [ ] Datos escritos se leen correctamente (write-read-verify)
- [ ] Page buffer se flushea al aterrizar (sin pérdida de últimas muestras)
- [ ] Mutex SPI previene conflictos con LoRa
- [ ] Dump UART produce datos correctos
- [ ] Conversión binario → CSV produce valores coherentes
- [ ] Capacidad para ≥120 s de vuelo sin overflow

---

## Referencias

- [W25Q128FV Datasheet](https://www.winbond.com/resource-files/w25q128fv%20rev.m%2005132016%20kms.pdf)
- [SPI Flash Application Note](https://www.winbond.com/resource-files/AN-043_SPI_flash_programming_guide.pdf)

---

*Módulo 08 - La caja negra. Todo lo que pase durante el vuelo queda registrado aquí.*
