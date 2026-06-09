#include "BatteryManager.h"

BatteryManager::BatteryManager(float shunt_ohms, float max_amps, uint8_t address)
    : shunt_ohms_(shunt_ohms), max_amps_(max_amps), address_(address),
      ready_(false), voltage_filt_(0), current_filt_(0),
      power_(0), percent_(0), is_low_(false),
      buf_idx_(0), last_read_ms_(0)
{
    memset(buf_v_, 0, sizeof(buf_v_));
    memset(buf_i_, 0, sizeof(buf_i_));
}

bool BatteryManager::begin()
{
    Wire.begin();

    // Verificar que el INA228 responde
    Wire.beginTransmission(address_);
    if (Wire.endTransmission() != 0) {
        return false;  // no encontrado
    }

    // CONFIG — modo continuo, promedio 16 muestras, 1052us conversion
    writeRegister(INA228_REG_CONFIG, 0x0000);

    // SHUNT_CAL — calibración según shunt y corriente máxima
    // SHUNT_CAL = 13107.2 × 10^6 × shunt_ohms × max_amps / (2^19)
    float cal = 13107.2e6f * shunt_ohms_ * max_amps_ / 524288.0f;
    writeRegister(INA228_REG_SHUNTCAL, (uint16_t)cal);

    ready_ = true;
    return true;
}

void BatteryManager::update()
{
    if (!ready_) return;

    unsigned long now = millis();
    if (now - last_read_ms_ < READ_INTERVAL_MS) return;
    last_read_ms_ = now;

    // Leer voltaje — registro 20 bits, LSB = 3.125mV / 16
    int32_t raw_v = readRegister24(INA228_REG_VBUS);
    raw_v >>= 4;  // 20 bits efectivos
    float voltage = raw_v * 0.0001953125f;  // 195.3125 µV/LSB

    // Leer corriente — registro 20 bits con signo
    int32_t raw_i = readRegister24(INA228_REG_CURRENT);
    raw_i >>= 4;
    float current_lsb = max_amps_ / 524288.0f;
    float current = raw_i * current_lsb;

    // Filtrar
    buf_idx_++;
    voltage_filt_ = movingAvg(buf_v_, voltage);
    current_filt_ = movingAvg(buf_i_, current);

    // Potencia
    power_ = voltage_filt_ * current_filt_;

    // Porcentaje
    percent_ = voltageToPercent(voltage_filt_);

    // Estado batería baja
    is_low_ = (percent_ < BAT_LOW_THRESHOLD);
}

float BatteryManager::voltageToPercent(float voltage)
{
    if (voltage >= BAT_VOLTAGE_MAX) return 100.0f;
    if (voltage <= BAT_VOLTAGE_MIN) return 0.0f;
    return (voltage - BAT_VOLTAGE_MIN) /
           (BAT_VOLTAGE_MAX - BAT_VOLTAGE_MIN) * 100.0f;
}

bool BatteryManager::writeRegister(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    return Wire.endTransmission() == 0;
}

int32_t BatteryManager::readRegister24(uint8_t reg)
{
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address_, (uint8_t)3);

    int32_t value = 0;
    if (Wire.available() == 3) {
        value  = (int32_t)Wire.read() << 16;
        value |= (int32_t)Wire.read() << 8;
        value |= Wire.read();
        // Extender signo 24 bits
        if (value & 0x800000) value |= 0xFF000000;
    }
    return value;
}

int16_t BatteryManager::readRegister16(uint8_t reg)
{
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address_, (uint8_t)2);

    int16_t value = 0;
    if (Wire.available() == 2) {
        value  = (int16_t)Wire.read() << 8;
        value |= Wire.read();
    }
    return value;
}

float BatteryManager::movingAvg(float* buf, float val)
{
    buf[buf_idx_ % BAT_FILT_N] = val;
    float sum = 0;
    for (int i = 0; i < BAT_FILT_N; i++) sum += buf[i];
    return sum / BAT_FILT_N;
}
