//new bumper manager
#ifndef BUMPER_MANAGER_H
#define BUMPER_MANAGER_H
#include <Arduino.h>
 
#define BUMPER_MAX_SENSORS 5
#define BUMPER_DEBOUNCE_MS 25   // tiempo mínimo estable para confirmar cambio
 
struct BumperReading {
    bool pressed;   // estado confirmado (debounced)
    bool changed;   // true durante UN ciclo de update() tras cambio
};
 
class BumperManager {
public:
// pins: array de pines en orden [B1, B2, B3, B4, B5]
    BumperManager(const uint8_t pins[BUMPER_MAX_SENSORS],
                  uint8_t debounce_ms = BUMPER_DEBOUNCE_MS);
 
    void begin();
 
    // Llamar en cada loop() — no bloquea nunca
    void update();
 
    BumperReading get(uint8_t index) const; //0-4
    bool anyPressed() const;                // si cualquiera esta presionado
    bool anyChanged() const;                // cambio de lectura
    static const char* name(uint8_t index); // nombres 
 
private:
    uint8_t       pins_[BUMPER_MAX_SENSORS];
    uint8_t       debounce_ms_;
 
    bool          confirmed_[BUMPER_MAX_SENSORS];   // estado debounced actual
    bool          candidate_[BUMPER_MAX_SENSORS];   // estado raw pendiente de confirmar
    unsigned long change_time_[BUMPER_MAX_SENSORS]; // millis() del último cambio raw
 
    BumperReading readings_[BUMPER_MAX_SENSORS];    // resultado expuesto al exterior
};
 
#endif // BUMPER_MANAGER_H

