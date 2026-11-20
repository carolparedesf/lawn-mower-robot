#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

// Dirección I2C por defecto INA228
#define INA228_ADDR         0x40

// Registros INA228
#define INA228_REG_CONFIG   0x00
#define INA228_REG_SHUNTCAL 0x02
#define INA228_REG_VBUS     0x05
#define INA228_REG_CURRENT  0x07
#define INA228_REG_POWER    0x08
#define INA228_REG_ENERGY   0x09

// Parámetros batería 36V LiPo/Li-ion hooverboard
#define BAT_VOLTAGE_MAX     36.0f   // V — 10S fully charged
#define BAT_VOLTAGE_MIN     30.0f   // V — 10S cutoff
#define BAT_LOW_THRESHOLD   30.0f   // % — umbral batería baja

// Filtro
#define BAT_FILT_N          4       // ventana media móvil

// Robustez del I2C (el lazo de control no puede esperar al INA228)
#define BAT_I2C_TIMEOUT_MS  5       // timeout de Wire por transaccion
#define BAT_FAIL_LIMIT      3       // lecturas fallidas seguidas => "sin dato"

class BatteryManager {
public:
    BatteryManager(float shunt_ohms,
                   float max_amps,
                   uint8_t address = INA228_ADDR);

    bool  begin();                  // inicializar I2C + calibrar (no falla si el INA no esta)
    // Llamar en cada ciclo. Acotado: lee como mucho cada READ_INTERVAL_MS, con
    // timeout I2C corto (BAT_I2C_TIMEOUT_MS). Si el INA228 no responde se marca
    // "sin dato" (isReady()=false, valores NAN) y se reintenta cada RETRY_INTERVAL_MS.
    void  update();

    float getVoltage()    const { return voltage_filt_; }   // V
    float getCurrent()    const { return current_filt_; }   // A
    float getPower()      const { return power_; }          // W
    float getPercent()    const { return percent_; }        // 0-100
    bool  isLow()         const { return is_low_; }
    bool  isReady()       const { return ready_; }          // true = hay dato valido

private:
    float   shunt_ohms_;
    float   max_amps_;
    uint8_t address_;
    bool    ready_;

    float voltage_filt_;
    float current_filt_;
    float power_;
    float percent_;
    bool  is_low_;

    float   buf_v_[BAT_FILT_N];
    float   buf_i_[BAT_FILT_N];
    uint8_t buf_idx_;

    unsigned long last_read_ms_;
    static const unsigned long READ_INTERVAL_MS  = 500;   // leer cada 500ms
    static const unsigned long RETRY_INTERVAL_MS = 2000;  // sin dato: reintentar cada 2 s
    uint8_t fail_count_;
    bool    wire_started_;

    bool    configure();            // CONFIG + SHUNT_CAL (tambien al reaparecer)
    void    setNoData();

    bool    writeRegister(uint8_t reg, uint16_t value);
    bool    readRegister24(uint8_t reg, int32_t& out);   // false si no hubo respuesta
    int16_t readRegister16(uint8_t reg);
    float   movingAvg(float* buf, float val);
    float   voltageToPercent(float voltage);
};

#endif
