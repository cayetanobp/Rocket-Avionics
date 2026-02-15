# Módulo 07: Telemetry Radio - Radio Telemetría

> **Estado**: 🔴 No iniciado  
> **Prioridad**: Fase 2 - Noveno módulo a desarrollar  
> **Dependencias**: 01-Power, 02-MCU Core, 03-Sensors Interface  
> **Dependientes**: Estación terrestre (fuera del alcance de Proteus)

---

## Resumen

Enlace de radio basado en LoRa (RFM95W / SX1276) para transmisión de telemetría en tiempo real a una estación terrestre. Envía paquetes con estado de vuelo, altitud, orientación, GPS y voltaje de batería. En modo RECOVERY, transmite posición GPS periódicamente para localización del cohete.

---

## Especificaciones Técnicas

### Módulo RFM95W (SX1276)
| Parámetro                | Valor                           |
|--------------------------|---------------------------------|
| Interfaz                 | SPI (Mode 0, CPOL=0, CPHA=0)   |
| Chip Select              | PB0 (GPIO, active low)         |
| Voltaje operación        | 1.8V - 3.7V                    |
| Frecuencia               | 433 MHz (Europa/regulación)    |
| Modulación               | LoRa (CSS - Chirp Spread Spectrum)|
| Potencia TX máxima       | +20 dBm (100 mW)               |
| Sensibilidad RX          | -148 dBm (SF12, BW 125kHz)     |
| Spreading Factor         | SF7 - SF12 (configurable)      |
| Bandwidth                | 125 kHz (default)              |
| Coding Rate              | 4/5 - 4/8                      |
| Data Rate                | 0.3 - 37.5 kbps                |
| Alcance teórico          | 2-15 km (línea de vista)       |
| Consumo TX               | 120 mA @ +20 dBm               |
| Consumo RX               | 10.3 mA                        |
| Consumo sleep            | 0.2 µA                         |
| Modelo Proteus           | ⚠️ No nativo (SPI simulable)   |

### Configuración LoRa Elegida
| Parámetro         | Valor  | Razón                                       |
|-------------------|--------|----------------------------------------------|
| Spreading Factor  | SF7    | Máximo data rate, menor time-on-air         |
| Bandwidth         | 125kHz | Balance entre sensibilidad y velocidad      |
| Coding Rate       | 4/5    | Mínimo overhead de FEC                      |
| TX Power          | +17dBm | Suficiente para 2-5 km, menor consumo       |
| Preamble          | 8      | Default, suficiente para sincronización      |

**Data rate resultante**: ~5.5 kbps → paquete de 32 bytes en ~46 ms

---

## Decisiones Técnicas

### ¿Por qué LoRa y no NRF24L01+ o HC-12?

| Criterio         | RFM95W (LoRa) | NRF24L01+    | HC-12        |
|------------------|----------------|--------------|--------------|
| Alcance           | 2-15 km       | 100-1000 m   | 1-2 km       |
| Frecuencia        | 433/868 MHz   | 2.4 GHz      | 433 MHz      |
| Interfaz          | SPI            | SPI          | UART         |
| Sensibilidad      | -148 dBm      | -94 dBm      | -117 dBm     |
| Potencia TX       | +20 dBm       | 0 dBm        | +20 dBm      |
| Modulación        | CSS (LoRa)    | GFSK         | GFSK         |
| Robustez a ruido  | Excelente      | Media        | Media        |

**Decisión**: LoRa - alcance superior, excelente robustez, ideal para telemetría de cohetes donde la distancia y fiabilidad son críticas.

### ¿Cómo simular radio en Proteus?

Proteus no simula RF. La verificación se hace a nivel SPI:

1. **SPI Debugger**: Verificar que los comandos al SX1276 son correctos
2. **Virtual Terminal**: Mostrar el contenido de los paquetes como si los recibiéramos
3. **GPIO monitoring**: DIO0 (TX Done interrupt) se simula manualmente

```
En simulación:
MCU → SPI → [Verificar registros escritos en SX1276]
         → [Verificar paquete binario correcto]
         → [Virtual Terminal muestra paquete decodificado]

En hardware real:
MCU → SPI → RFM95W → Antena → ~~~ → RFM95W → Estación Terrestre
```

### Protocolo de Paquetes

```c
// Estructura del paquete de telemetría (32 bytes, fixed size)
typedef struct __attribute__((packed)) {
    uint8_t  header;           // 0xAA (sync byte)
    uint8_t  packet_type;      // 0x01=telemetry, 0x02=gps, 0x03=status
    uint16_t sequence;         // Número de secuencia (rollover at 65535)
    uint32_t timestamp_ms;     // ms desde boot
    uint8_t  flight_state;     // Estado FSM (0-9)
    int16_t  altitude;         // Altitud AGL × 10 (0.1m resolución, max ±3276m)
    int16_t  vspeed;           // Velocidad vertical × 100 (0.01 m/s)
    int16_t  accel_z;          // Aceleración Z × 100 (0.01 m/s²)
    int16_t  pitch;            // Pitch × 10 (0.1°)
    int16_t  roll;             // Roll × 10 (0.1°)
    int16_t  yaw;              // Yaw × 10 (0.1°)
    int32_t  latitude;         // Lat × 1e6 (microdegrees)
    int32_t  longitude;        // Lon × 1e6 (microdegrees)
    uint8_t  battery_pct;      // Porcentaje de batería (0-100)
    uint8_t  gps_sats;         // Número de satélites GPS
    uint16_t crc16;            // CRC-16/CCITT
} telem_packet_t;

_Static_assert(sizeof(telem_packet_t) == 32, "Packet must be 32 bytes");
```

---

## Desarrollo Step-by-Step

### Paso 1: Driver SPI del SX1276
```c
#define LORA_CS_PORT  GPIOB
#define LORA_CS_PIN   GPIO_PIN_0
#define LORA_DIO0_PORT GPIOB
#define LORA_DIO0_PIN  GPIO_PIN_1

// Registros importantes del SX1276
#define REG_FIFO          0x00
#define REG_OP_MODE       0x01
#define REG_FR_MSB        0x06
#define REG_FR_MID        0x07
#define REG_FR_LSB        0x08
#define REG_PA_CONFIG     0x09
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE  0x0E
#define REG_IRQ_FLAGS     0x12
#define REG_PAYLOAD_LEN   0x22
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_VERSION       0x42

uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7F, 0x00};
    uint8_t rx[2];
    spi_select(LORA_CS_PORT, LORA_CS_PIN);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    spi_deselect(LORA_CS_PORT, LORA_CS_PIN);
    return rx[1];
}

void lora_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg | 0x80, value};
    spi_select(LORA_CS_PORT, LORA_CS_PIN);
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    spi_deselect(LORA_CS_PORT, LORA_CS_PIN);
}
```

### Paso 2: Inicialización LoRa
```c
bool lora_init(void) {
    // Verificar versión del chip
    uint8_t version = lora_read_reg(REG_VERSION);
    if (version != 0x12) {
        printf("[TELEM] ERROR: LoRa chip not found (ver=0x%02X)\n", version);
        return false;
    }
    
    // Sleep mode para configurar
    lora_write_reg(REG_OP_MODE, 0x80 | 0x00);  // LoRa mode + Sleep
    
    // Frecuencia: 433 MHz
    // Frf = (Freq × 2^19) / 32MHz
    uint32_t frf = (uint32_t)((433.0 * (1 << 19)) / 32.0);
    lora_write_reg(REG_FR_MSB, (frf >> 16) & 0xFF);
    lora_write_reg(REG_FR_MID, (frf >> 8) & 0xFF);
    lora_write_reg(REG_FR_LSB, frf & 0xFF);
    
    // Potencia TX: +17 dBm (PA_BOOST)
    lora_write_reg(REG_PA_CONFIG, 0x80 | 0x0F);  // PA_BOOST, MaxPower=7, OutputPower=15
    
    // Modem Config: BW=125kHz, CR=4/5, Implicit header=No
    lora_write_reg(REG_MODEM_CONFIG1, 0x72);  // BW=125, CR=4/5, explicit header
    
    // SF=7, CRC on
    lora_write_reg(REG_MODEM_CONFIG2, 0x74);  // SF7, CRC enable
    
    // Standby mode
    lora_write_reg(REG_OP_MODE, 0x80 | 0x01);  // LoRa + Standby
    
    printf("[TELEM] LoRa initialized: 433MHz, SF7, BW125, +17dBm\n");
    return true;
}
```

### Paso 3: Transmisión de Paquetes
```c
void lora_send_packet(const uint8_t *data, uint8_t length) {
    // Set FIFO pointer to TX base
    lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_TX_BASE));
    
    // Write data to FIFO
    for (uint8_t i = 0; i < length; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
    
    // Set payload length
    lora_write_reg(REG_PAYLOAD_LEN, length);
    
    // Start TX
    lora_write_reg(REG_OP_MODE, 0x80 | 0x03);  // LoRa + TX
    
    // Esperar TX Complete (DIO0 = TxDone)
    // En simulación: timeout de 100ms
    uint32_t start = HAL_GetTick();
    while (!(lora_read_reg(REG_IRQ_FLAGS) & 0x08)) {
        if (HAL_GetTick() - start > 100) {
            printf("[TELEM] TX timeout\n");
            break;
        }
    }
    
    // Clear IRQ flags
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    
    // Back to standby
    lora_write_reg(REG_OP_MODE, 0x80 | 0x01);
}
```

### Paso 4: CRC-16 CCITT
```c
uint16_t crc16_ccitt(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

### Paso 5: Empaquetado de Telemetría
```c
void telem_build_packet(telem_packet_t *pkt, 
                         const flight_data_t *flight) {
    pkt->header = 0xAA;
    pkt->packet_type = 0x01;
    pkt->sequence = telem_seq_num++;
    pkt->timestamp_ms = HAL_GetTick();
    pkt->flight_state = flight->flight_state;
    pkt->altitude = (int16_t)(flight->baro.altitude * 10);
    pkt->vspeed = (int16_t)(flight->baro.vspeed * 100);
    pkt->accel_z = (int16_t)(flight->imu.accel_z * 100);
    pkt->pitch = (int16_t)(flight->imu.pitch * 10);
    pkt->roll = (int16_t)(flight->imu.roll * 10);
    pkt->yaw = (int16_t)(flight->imu.yaw * 10);
    pkt->latitude = (int32_t)(flight->gps.latitude * 1e6);
    pkt->longitude = (int32_t)(flight->gps.longitude * 1e6);
    pkt->battery_pct = power_voltage_to_pct(flight->power.bus_voltage);
    pkt->gps_sats = flight->gps.satellites;
    
    // CRC sobre todo menos los 2 últimos bytes
    pkt->crc16 = crc16_ccitt((uint8_t*)pkt, sizeof(telem_packet_t) - 2);
}
```

### Paso 6: Tarea RTOS
```c
void vTaskTelemetry(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(100);  // 10 Hz durante vuelo
    
    lora_init();
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        
        flight_data_t flight;
        // Recopilar datos más recientes de todas las colas
        xQueuePeek(imu_data_q, &flight.imu, 0);
        xQueuePeek(baro_data_q, &flight.baro, 0);
        xQueuePeek(gps_data_q, &flight.gps, 0);
        xQueuePeek(power_data_q, &flight.power, 0);
        flight.flight_state = fsm_get_state();
        
        telem_packet_t pkt;
        telem_build_packet(&pkt, &flight);
        
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            lora_send_packet((uint8_t*)&pkt, sizeof(pkt));
            xSemaphoreGive(spi_mutex);
        }
        
        // En RECOVERY_MODE: reducir rate a cada 5s, pero incluir GPS
        if (flight.flight_state == STATE_RECOVERY_MODE) {
            period = pdMS_TO_TICKS(5000);
        }
    }
}
```

---

## API del Módulo

```c
// telemetry_radio.h

// Inicialización
bool telem_init(SPI_HandleTypeDef *hspi);

// Transmisión
void telem_send_flight_data(const flight_data_t *data);
void telem_send_gps_position(float lat, float lon, float alt);
void telem_send_status(const char *message);

// Configuración
void telem_set_tx_power(uint8_t dbm);       // 2-20 dBm
void telem_set_frequency(float mhz);         // 433.0-435.0
void telem_set_rate(uint16_t packets_per_sec);

// Estadísticas
uint32_t telem_get_packets_sent(void);
uint32_t telem_get_tx_errors(void);
int8_t   telem_get_last_rssi(void);           // Para ACK (si se implementa)
```

---

## Criterios de Aceptación

- [ ] SX1276 detectado via SPI (version register = 0x12)
- [ ] Registros de configuración escritos correctamente (verificar con SPI debugger)
- [ ] Paquete de 32 bytes se transmite en < 50 ms (SF7, BW125)
- [ ] CRC-16 se calcula correctamente (verificar con test vector)
- [ ] Paquetes contienen datos coherentes con sensores
- [ ] Sequence number incrementa correctamente
- [ ] Rate de 10 Hz sostenido durante vuelo
- [ ] Rate cambia a 5s en RECOVERY_MODE
- [ ] Mutex SPI previene conflictos con Flash
- [ ] No hay pérdida de datos en colas de entrada

---

## Referencias

- [SX1276 Datasheet](https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V7.pdf)
- [RFM95W Datasheet](https://cdn.sparkfun.com/assets/learn_tutorials/8/0/4/RFM95_96_97_98W.pdf)
- [LoRa Modem Design Guide](https://www.semtech.com/uploads/documents/LoraDesignGuide_STD.pdf)

---

*Módulo 07 - La voz del cohete. Transmite todo lo que sabe a quienes esperan en tierra.*
