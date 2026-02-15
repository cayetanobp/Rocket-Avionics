# Módulo 01: Power Management - Gestión de Energía

> **Estado**: 🟡 En desarrollo
> **Prioridad**: Fase 2 - Primer módulo a desarrollar  
> **Dependencias**: Ninguna (módulo base)  
> **Dependientes**: Todos los demás módulos

---

## Resumen

Este módulo proporciona alimentación regulada y monitorizada a todos los subsistemas del cohete. Convierte el voltaje de la batería LiPo 3S (9.0-12.6V) a los dos rails necesarios: 5V para actuadores y 3.3V para el MCU y sensores digitales. Incluye monitorización en tiempo real de voltaje y corriente mediante INA219.

---

## Especificaciones Técnicas

### Batería
| Parámetro          | Valor                    |
|--------------------|--------------------------|
| Tipo               | LiPo 3S                 |
| Voltaje nominal    | 11.1V                   |
| Rango de operación | 9.0V - 12.6V            |
| Capacidad mínima   | 1000 mAh                |
| Descarga máxima    | 10C (para picos)        |

### Rail 5V (Buck Converter)
| Parámetro          | Valor                    |
|--------------------|--------------------------|
| Regulador          | LM2596 o MC34063         |
| Voltaje de salida  | 5.0V ± 2%               |
| Corriente máxima   | 2A continuo              |
| Eficiencia         | >85%                     |
| Ripple máximo      | 50 mV pico-pico          |
| Carga principal    | Servos (4×, 500mA pico c/u) |

TO-DO:

### Rail 3.3V (LDO)
| Parámetro          | Valor                    |
|--------------------|--------------------------|
| Regulador          | AMS1117-3.3 o LM1117    |
| Voltaje de entrada | 5V (desde buck converter)|
| Voltaje de salida  | 3.3V ± 1%               |
| Corriente máxima   | 800 mA                  |
| Dropout            | 1.2V típico              |
| Carga total        | ~306 mA (ver desglose)  |

### Monitor de Batería (INA219)
| Parámetro          | Valor                    |
|--------------------|--------------------------|
| Interfaz           | I2C (dirección 0x40)     |
| Rango de voltaje   | 0-26V                   |
| Resolución voltaje | 4 mV                    |
| Shunt resistor     | 0.1Ω                    |
| Rango de corriente | ±3.2A                   |
| Resolución corriente| 0.8 mA                 |

---

## Decisiones Técnicas

### ¿Por qué LiPo 3S en lugar de 2S?
- **3S (11.1V)** ofrece margen suficiente para el buck converter de 5V incluso con batería a 70% (≈10.5V)
- Con 2S (7.4V), el buck tendría apenas ~2V de headroom a batería baja, afectando eficiencia
- La potencia instantánea necesaria (~800mA pico) se maneja mejor con mayor voltaje

### ¿Por qué buck converter + LDO en cascada?
- **Buck 12V→5V**: Alta eficiencia (>85%) para la mayor caída de voltaje
- **LDO 5V→3.3V**: Ultrabajo ruido en el rail del MCU y sensores I2C
- Alternativa descartada: Buck directo 12V→3.3V - mayor ripple, riesgo de ruido en ADC y sensores

### ¿Por qué INA219 y no un divisor resistivo?
- INA219 mide **voltaje Y corriente** simultáneamente via I2C
- Permite calcular potencia y energía consumida
- Mucho más preciso que un ADC con divisor resistivo
- Notificación programable de umbrales (alert pin)

### ¿Por qué no usar el ADC del STM32 directamente?
- El ADC del STM32 mide max 3.3V - necesitaría divisor resistivo con sus imprecisiones
- INA219 libera canales ADC para otros usos
- Medición de corriente mucho más compleja sin sensor dedicado

---

## Diseño del Circuito

### Diagrama de Bloques

```
LiPo 3S (9-12.6V)
    │
    ├── Fusible 5A ── Diodo Schottky (protección inversión)
    │                         │
    │                    ┌────▼────┐
    │                    │  INA219 │── I2C → MCU
    │                    │  Shunt  │
    │                    └────┬────┘
    │                         │
    │                    ┌────▼─────────┐
    │                    │ Buck LM2596  │
    │                    │  12V → 5V    │
    │                    │  L=33µH      │
    │                    │  C_out=220µF │
    │                    └────┬─────────┘
    │                         │
    │                    ┌────▼──────────┐
    │              5V ───┤ LDO AMS1117  │
    │                    │  5V → 3.3V   │
    │                    │  C_in=10µF   │
    │                    │  C_out=22µF  │
    │                    └────┬─────────┘
    │                         │
    │                       3.3V
    │
    ├── LED Verde ── "Power OK"
    ├── LED Rojo  ── "Battery Low" (controlado por MCU via GPIO)
    └── LED Ámbar ── "System Active" (heartbeat)
```

### Componentes del Esquemático

| Ref  | Componente            | Valor/Tipo        | Proteus Model  |
|------|-----------------------|-------------------|----------------|
| U1   | INA219                | SOIC-8            | ✅ Disponible  |
| U2   | LM2596-5.0            | TO-263            | ✅ Disponible  |
| U3   | AMS1117-3.3            | SOT-223           | ✅ Disponible  |
| D1   | Diodo Schottky         | SS34 (3A)         | ✅ Genérico    |
| D2   | Diodo recirculación    | SS34              | ✅ Genérico    |
| F1   | Fusible                | 5A                | ✅ Disponible  |
| L1   | Inductor               | 33µH, 3A          | ✅ Disponible  |
| R_sh | Shunt INA219           | 0.1Ω, 1W          | Resistencia    |
| C1   | Cap entrada buck       | 100µF/25V electr. | ✅             |
| C2   | Cap salida buck        | 220µF/10V electr. | ✅             |
| C3   | Cap entrada LDO        | 10µF cerámico     | ✅             |
| C4   | Cap salida LDO         | 22µF cerámico     | ✅             |
| R1-R3| Resistencias LED       | 330Ω              | ✅             |

---

## Desarrollo Step-by-Step

### Paso 1: Esquemático Base de Alimentación
1. Abrir Proteus, crear nuevo proyecto: `01-power-management/simulation/power_mgmt.pdsprj`
2. Colocar fuente DC (simula batería) - configurable 9V-12.6V
3. Añadir potenciómetro en serie para simular descarga
4. Colocar fusible y diodo Schottky de protección
5. **Test**: Medir voltaje en la salida con voltímetro virtual

### Paso 2: Buck Converter 5V
1. Colocar LM2596-5.0 con circuito de aplicación del datasheet
2. Añadir inductor 33µH y diodo de recirculación
3. Añadir condensadores de entrada (100µF) y salida (220µF)
4. **Test**: Verificar 5V estable con carga de 100Ω-500Ω

### Paso 3: LDO 3.3V
1. Colocar AMS1117-3.3 alimentado desde rail 5V
2. Añadir condensadores de entrada (10µF) y salida (22µF)
3. **Test**: Verificar 3.3V estable con carga de 100Ω

### Paso 4: Monitor de Batería INA219
1. Colocar INA219 con resistencia shunt de 0.1Ω en línea de alimentación
2. Conectar SDA/SCL a pines I2C (con pull-ups 4.7kΩ)
3. Configurar dirección I2C: A0=GND, A1=GND → 0x40
4. **Test**: Enviar lectura I2C y verificar voltaje/corriente

### Paso 5: LEDs de Estado
1. Añadir LED verde (power OK) conectado directo al rail 3.3V
2. Añadir LED rojo (battery low) controlado por GPIO del MCU
3. Añadir LED ámbar (heartbeat) controlado por GPIO del MCU
4. **Test**: Verificar encendido con resistencias de 330Ω

### Paso 6: Firmware del Módulo
1. Escribir driver INA219 (I2C read registers)
2. Implementar función `power_get_voltage()` y `power_get_current()`
3. Implementar detección de batería baja (threshold configurable)
4. Implementar logging de estado de energía

### Paso 7: Tests de Simulación
1. Simular descarga de batería (reducir potenciómetro gradualmente)
2. Verificar alarma de batería baja al cruzar 10.0V
3. Verificar output de logs en Virtual Terminal
4. Verificar estabilidad de rails bajo carga variable

---

## Interfaz con Otros Módulos

### Salidas de Potencia
```c
// Definiciones de interfaz de Power Management
#define POWER_RAIL_5V       5.0f    // Rail para servos
#define POWER_RAIL_3V3      3.3f    // Rail para MCU + sensores
```

### API del Módulo
```c
// power_management.h

typedef struct {
    float bus_voltage;      // Voltaje de batería (V)
    float shunt_voltage;    // Voltaje en shunt (mV)
    float current;          // Corriente (mA)
    float power;            // Potencia (mW)
} power_data_t;

typedef enum {
    POWER_OK = 0,
    POWER_LOW_WARNING,      // V < 10.0V
    POWER_CRITICAL,         // V < 9.0V
    POWER_SENSOR_ERROR      // INA219 no responde
} power_status_t;

// Inicialización
HAL_StatusTypeDef power_init(I2C_HandleTypeDef *hi2c);

// Lectura de datos
power_data_t power_read(void);

// Estado de la batería
power_status_t power_get_status(void);

// Callback de alarma (llamado por ISR o tarea)
void power_set_low_voltage_callback(void (*callback)(power_status_t));
```

### Conexiones I2C
```
INA219 (0x40) ── SDA/SCL ── MCU I2C1
```

---

## Criterios de Aceptación

- [ ] Rail 5V estable bajo carga de 0-2A (±2%)
- [ ] Rail 3.3V estable bajo carga de 0-500mA (±1%)
- [ ] INA219 reporta voltaje con error < 50mV
- [ ] INA219 reporta corriente con error < 10mA
- [ ] Alarma de batería baja se activa a V < 10.0V
- [ ] Alarma crítica se activa a V < 9.0V
- [ ] LEDs de estado funcionan correctamente
- [ ] Sin oscilaciones ni inestabilidad en reguladores
- [ ] Protección contra polaridad inversa funcional

---

## Simulación en Proteus

### Instrumentos a Utilizar
- **Voltímetros DC**: En cada rail (batería, 5V, 3.3V)
- **Amperímetro DC**: En línea principal
- **Graph (Analogue)**: Voltaje de batería durante descarga simulada
- **Virtual Terminal**: Logs de estado de energía
- **I2C Debugger**: Verificar comunicación con INA219
- **Potenciómetro**: Simular descarga de batería

### Escenarios de Test

| # | Escenario                    | Resultado Esperado                          |
|---|------------------------------|---------------------------------------------|
| 1 | Batería a 12.6V (full)       | Todos los rails estables, POWER_OK          |
| 2 | Batería a 10.5V (media)      | Rails estables, POWER_OK                    |
| 3 | Batería a 10.0V              | POWER_LOW_WARNING, LED rojo ON              |
| 4 | Batería a 9.0V               | POWER_CRITICAL, acción de emergencia        |
| 5 | Batería a 8.5V               | LDO puede fallar, verificar comportamiento  |
| 6 | Carga 5V: pico de 2A         | Rail 5V se mantiene ±2%                     |
| 7 | Desconexión de INA219        | POWER_SENSOR_ERROR en logs                  |

---

## Referencias

- [INA219 Datasheet](https://www.ti.com/lit/ds/symlink/ina219.pdf)
- [LM2596 Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)
- [AMS1117 Datasheet](http://www.advanced-monolithic.com/pdf/ds1117.pdf)
- [LiPo Battery Safety Guide](https://www.rcgroups.com/forums/showthread.php?1146291-A-Guide-to-LiPo-Batteries)

---

*Módulo 01 - Base de todo el sistema. Sin energía estable, nada funciona.*
