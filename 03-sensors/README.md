# Módulo 03: Sensors Interface - Abstracción de Sensores

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Cuarto módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core  
> **Dependientes**: 04-IMU, 05-Barometer, 06-GPS

---

## Resumen

Capa de abstracción y gestión centralizada de todos los buses de comunicación con sensores. Implementa detección automática de dispositivos I2C, manejo de errores, timeouts, y acceso thread-safe a buses compartidos mediante mutexes RTOS. No lee datos de sensores directamente - proporciona la infraestructura sobre la cual los módulos individuales operan.

---

## Especificaciones Técnicas

### Bus I2C1 (Sensores)
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Velocidad          | 400 kHz (Fast Mode)            |
| Pull-ups           | 4.7 kΩ en SDA y SCL            |
| Voltaje            | 3.3V                           |
| Direcciones usadas | 0x40 (INA219), 0x68 (MPU6050), 0x76 (BMP280) |
| Timeout            | 100 ms por transacción          |
| Protección         | Mutex RTOS (i2c_mutex)          |

### Bus SPI1 (Flash + Radio)
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Velocidad          | 8 MHz (CPOL=0, CPHA=0, Mode 0) |
| Chip Selects       | PA4 (Flash W25Q128), PB0 (LoRa RFM95W) |
| Voltaje            | 3.3V                           |
| Protección         | Mutex RTOS (spi_mutex)          |

### UART1 (GPS)
| Parámetro          | Valor                           |
|--------------------|---------------------------------|
| Baud rate          | 9600                            |
| Configuración      | 8N1 (8 data, no parity, 1 stop)|
| Buffer RX          | 256 bytes (DMA circular)        |
| Parsing            | Interrupt-driven + queue        |

---

## Decisiones Técnicas

### ¿Por qué no usar multiplexor I2C (TCA9548A)?
- Solo tenemos 3 dispositivos I2C con direcciones **únicas** (0x40, 0x68, 0x76)
- No hay conflicto de direcciones → multiplexor innecesario
- Menos componentes = menor complejidad = menos puntos de fallo
- **Se añadiría el TCA9548A solo si** se necesita un segundo MPU6050 (redundancia)

### ¿Por qué mutexes y no deshabilitar interrupciones?
- Deshabilitar interrupciones bloquea **todo** el sistema - inaceptable para recovery
- Mutex solo bloquea la tarea que intenta acceder al bus - otras tareas siguen ejecutándose
- Priority inheritance en FreeRTOS mutex evita inversión de prioridad
- Si la tarea Recovery necesita leer un sensor urgentemente, el mutex con priority inheritance asegura que la tarea que tiene el bus sube temporalmente de prioridad

### ¿Por qué DMA para UART GPS?
- El GPS envía tramas NMEA continuamente (~80 chars @ 9600 baud = ~83ms por trama)
- Polling bloquearía la tarea durante recepción
- Interrupt por byte tiene overhead alto a 9600 baud
- **DMA circular**: Recibe en background, tarea parsea cuando hay datos - zero CPU overhead

### ¿Por qué I2C a 400kHz y no 100kHz?
- 100kHz: Lectura de 14 bytes del MPU6050 ≈ 2.5ms
- 400kHz: Lectura de 14 bytes del MPU6050 ≈ 0.7ms
- Con 3 dispositivos a 100Hz, el bus estaría ocupado ~12ms/ciclo a 100kHz vs ~3ms a 400kHz
- **MPU6050, BMP280, INA219** todos soportan Fast Mode (400kHz)

---

## Diseño del Circuito

### Diagrama de Conexiones I2C

```
3.3V ──┬─────────────┬──────────────┐
       │             │              │
     4.7kΩ         4.7kΩ           │
       │             │              │
       │    SDA ─────┼──── MPU6050 (0x68)
       │             │         │
       │    SCL ─────┼──── BMP280 (0x76)
       │             │         │
       │             │    INA219 (0x40)
       │             │         │
    PB7 (SDA)    PB6 (SCL)   GND
    (STM32)      (STM32)
```

### Diagrama de Conexiones SPI

```
                    ┌── PA4 (CS_FLASH) ── W25Q128
                    │
PA5 (SCK) ─────────┼───────────────────────────────
PA6 (MISO) ────────┼───────────────────────────────
PA7 (MOSI) ────────┼───────────────────────────────
                    │
                    └── PB0 (CS_LORA) ── RFM95W
```

---

## Desarrollo Step-by-Step

### Paso 1: Bus I2C - Hardware
1. Verificar pull-ups de 4.7kΩ en SDA y SCL
2. Calcular impedancia del bus: 3 dispositivos × ~10pF = ~30pF → OK con 4.7kΩ @ 400kHz
3. Añadir bypass caps (100nF) cerca de cada dispositivo I2C

### Paso 2: I2C Scanner
1. Implementar función de escaneo I2C:
   ```c
   uint8_t i2c_scan(I2C_HandleTypeDef *hi2c, uint8_t *found_devices) {
       uint8_t count = 0;
       for (uint8_t addr = 0x08; addr < 0x78; addr++) {
           if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 100) == HAL_OK) {
               found_devices[count++] = addr;
               printf("[SENSORS] Found device at 0x%02X\n", addr);
           }
       }
       return count;
   }
   ```
2. Ejecutar al boot y reportar por UART
3. **Test**: Verificar que detecta 0x40, 0x68, 0x76

### Paso 3: Thread-Safe I2C Access
1. Implementar wrapper thread-safe:
   ```c
   HAL_StatusTypeDef i2c_read_safe(uint8_t addr, uint8_t reg, 
                                     uint8_t *data, uint16_t len) {
       HAL_StatusTypeDef status;
       if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
           status = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, 
                                      I2C_MEMADD_SIZE_8BIT, data, len, 100);
           xSemaphoreGive(i2c_mutex);
       } else {
           status = HAL_TIMEOUT;
           printf("[ERROR] I2C mutex timeout\n");
       }
       return status;
   }
   ```
2. Implementar similar para write, SPI read, SPI write
3. **Test**: Dos tareas accediendo al I2C simultáneamente sin corrupción

### Paso 4: Error Handling
1. Implementar retry con backoff:
   ```c
   HAL_StatusTypeDef i2c_read_retry(uint8_t addr, uint8_t reg, 
                                      uint8_t *data, uint16_t len, uint8_t retries) {
       for (uint8_t i = 0; i < retries; i++) {
           if (i2c_read_safe(addr, reg, data, len) == HAL_OK) return HAL_OK;
           vTaskDelay(pdMS_TO_TICKS(5 * (i + 1)));  // Backoff
           printf("[WARN] I2C retry %d for device 0x%02X\n", i+1, addr);
       }
       printf("[ERROR] I2C device 0x%02X not responding\n", addr);
       return HAL_ERROR;
   }
   ```
2. Bus recovery: Si I2C se cuelga, generar 9 clock pulses para desbloquear
3. **Test**: Desconectar sensor en simulación → error reportado, las demás lecturas siguen OK

### Paso 5: Bus SPI - Setup
1. Configurar SPI1 en modo Master, CPOL=0, CPHA=0
2. Implementar chip select management:
   ```c
   void spi_select(GPIO_TypeDef *cs_port, uint16_t cs_pin) {
       HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);  // Active low
   }
   void spi_deselect(GPIO_TypeDef *cs_port, uint16_t cs_pin) {
       HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
   }
   ```
3. **Test**: Enviar byte SPI y verificar en SPI Debugger de Proteus

### Paso 6: UART GPS con DMA
1. Configurar UART1 con DMA circular receive:
   ```c
   uint8_t gps_dma_buffer[256];
   HAL_UART_Receive_DMA(&huart1, gps_dma_buffer, sizeof(gps_dma_buffer));
   ```
2. Implementar half/complete transfer callbacks:
   ```c
   void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
       // Parsear primera mitad del buffer
   }
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
       // Parsear segunda mitad del buffer
   }
   ```
3. **Test**: Enviar trama NMEA desde Virtual Terminal → verificar parsing

---

## API del Módulo

```c
// sensors_interface.h

// Inicialización de todos los buses
HAL_StatusTypeDef sensors_init(void);

// I2C thread-safe
HAL_StatusTypeDef sensors_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef sensors_i2c_write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

// SPI thread-safe
HAL_StatusTypeDef sensors_spi_transfer(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                        uint8_t *tx, uint8_t *rx, uint16_t len);

// Diagnóstico
uint8_t sensors_scan_i2c(uint8_t *found_devices);
bool sensors_is_device_ready(uint8_t i2c_addr);

// Estadísticas
typedef struct {
    uint32_t i2c_transactions;
    uint32_t i2c_errors;
    uint32_t spi_transactions;
    uint32_t spi_errors;
    uint32_t uart_bytes_received;
} sensor_stats_t;

sensor_stats_t sensors_get_stats(void);
```

---

## Criterios de Aceptación

- [ ] I2C scan detecta los 3 dispositivos al boot
- [ ] Lecturas I2C thread-safe: sin corrupción con 3 tareas simultáneas
- [ ] Escrituras SPI thread-safe: Flash y LoRa sin conflicto
- [ ] GPS DMA recibe tramas NMEA sin pérdida de caracteres
- [ ] Timeout I2C de 100ms funciona correctamente
- [ ] Retry con backoff recupera de errores transitorios
- [ ] Bus recovery funciona ante I2C hang
- [ ] Estadísticas de errores se reportan por UART debug
- [ ] Zero data corruption en 10 minutos de operación continua

---

*Módulo 03 - La infraestructura invisible que hace posible todo lo demás.*
