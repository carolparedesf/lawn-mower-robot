#include "BatteryManager.h"

BatteryManager::BatteryManager(float shunt_ohms, float max_amps, uint8_t address)
    : shunt_ohms_(shunt_ohms), max_amps_(max_amps), address_(address),
      ready_(false), voltage_filt_(NAN), current_filt_(NAN),
      power_(NAN), percent_(NAN), is_low_(false),
      buf_idx_(0), last_read_ms_(0), fail_count_(0), wire_started_(false)
{
    memset(buf_v_, 0, sizeof(buf_v_));
    memset(buf_i_, 0, sizeof(buf_i_));
}

bool BatteryManager::configure()
{
    // Verificar que el INA228 responde
    Wire.beginTransmission(address_);
    if (Wire.endTransmission() != 0) {
        return false;  // no encontrado
    }

    // CONFIG — modo continuo, promedio 16 muestras, 1052us conversion
    if (!writeRegister(INA228_REG_CONFIG, 0x0000)) return false;

    // SHUNT_CAL — calibración según shunt y corriente máxima
    // SHUNT_CAL = 13107.2 × 10^6 × shunt_ohms × max_amps / (2^19)
    float cal = 13107.2e6f * shunt_ohms_ * max_amps_ / 524288.0f;
    if (!writeRegister(INA228_REG_SHUNTCAL, (uint16_t)cal)) return false;

    // Arrancar el filtro desde cero (igual que en el arranque)
    memset(buf_v_, 0, sizeof(buf_v_));
    memset(buf_i_, 0, sizeof(buf_i_));
    fail_count_ = 0;
    ready_ = true;
    return true;
}

bool BatteryManager::begin()
{
    if (!wire_started_) {          // Wire.begin() una sola vez (no en cada reintento)
        Wire.begin();
        Wire.setClock(400000);
        Wire.setTimeOut(BAT_I2C_TIMEOUT_MS);
        wire_started_ = true;
    }
    last_read_ms_ = millis();
    if (!configure()) { setNoData(); return false; }
    return true;
}

// Marca la bateria como "sin dato": nadie debe usar estos valores
void BatteryManager::setNoData()
{
    ready_        = false;
    voltage_filt_ = NAN;
    current_filt_ = NAN;
    power_        = NAN;
    percent_      = NAN;
    is_low_       = false;
}

void BatteryManager::update()
{
    unsigned long now = millis();

    // Sin dato: un intento (acotado por el timeout de Wire) cada RETRY_INTERVAL_MS
    if (!ready_) {
        if (!wire_started_ || now - last_read_ms_ < RETRY_INTERVAL_MS) return;
        last_read_ms_ = now;
        configure();
        return;
    }

    if (now - last_read_ms_ < READ_INTERVAL_MS) return;
    last_read_ms_ = now;

    // Leer voltaje — registro 20 bits, LSB = 3.125mV / 16
    // Si algo no responde se descarta la muestra: no se hace una 2a lectura
    // contra un dispositivo que ya fallo.
    int32_t raw_v, raw_i;
    if (!readRegister24(INA228_REG_VBUS, raw_v) ||
        !readRegister24(INA228_REG_CURRENT, raw_i)) {
        if (++fail_count_ >= BAT_FAIL_LIMIT) setNoData();
        return;
    }
    fail_count_ = 0;

    raw_v >>= 4;  // 20 bits efectivos
    float voltage = raw_v * 0.0001953125f;  // 195.3125 µV/LSB

    // Corriente — registro 20 bits con signo
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

bool BatteryManager::readRegister24(uint8_t reg, int32_t& out)
{
    Wire.beginTransmission(address_);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = Wire.requestFrom(address_, (uint8_t)3);

    if (received != 3 || Wire.available() != 3) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }

    int32_t value = 0;

    value  = (int32_t)Wire.read() << 16;
    value |= (int32_t)Wire.read() << 8;
    value |= (int32_t)Wire.read();

    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    out = value;
    return true;
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
