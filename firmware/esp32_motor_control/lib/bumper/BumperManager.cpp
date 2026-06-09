#include "BumperManager.h"

static const char* BUMPER_NAMES[BUMPER_MAX_SENSORS] = {
    "B1_front_left",
    "B2_front_right",
    "B3_left",
    "B4_right",
    "B5_back"
};

BumperManager::BumperManager(const uint8_t pins[BUMPER_MAX_SENSORS],
                              uint8_t debounce_ms)
    : debounce_ms_(debounce_ms)
{
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        pins_[i]        = pins[i];
        confirmed_[i]   = false;
        candidate_[i]   = false;
        change_time_[i] = 0;
        readings_[i]    = { false, false };
    }
}

void BumperManager::begin()
{
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        pinMode(pins_[i], INPUT_PULLUP);
        // Leer estado inicial para no disparar "changed" al arrancar
        confirmed_[i] = (digitalRead(pins_[i]) == LOW);
        candidate_[i] = confirmed_[i];
    }
}

// ── Sin delay — debounce por tiempo transcurrido ──────────────
//
// Lógica:
//   1. Leer pin raw
//   2. Si difiere del candidato actual → registrar tiempo y guardar nuevo candidato
//   3. Si el candidato lleva >= debounce_ms_ estable → confirmar y marcar changed
//   4. Si no cambió → limpiar flag changed
//
void BumperManager::update()
{
    unsigned long now = millis();

    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        bool raw = (digitalRead(pins_[i]) == LOW);

        if (raw != candidate_[i]) {
            // Nuevo cambio detectado — reiniciar temporizador
            candidate_[i]   = raw;
            change_time_[i] = now;
        }

        readings_[i].changed = false;

        if (candidate_[i] != confirmed_[i]) {
            // Esperar a que el candidato sea estable el tiempo de debounce
            if ((now - change_time_[i]) >= debounce_ms_) {
                confirmed_[i]        = candidate_[i];
                readings_[i].changed = true;
            }
        }

        readings_[i].pressed = confirmed_[i];
    }
}

BumperReading BumperManager::get(uint8_t index) const
{
    if (index >= BUMPER_MAX_SENSORS) return { false, false };
    return readings_[index];
}

bool BumperManager::anyPressed() const
{
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        if (readings_[i].pressed) return true;
    }
    return false;
}

bool BumperManager::anyChanged() const
{
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        if (readings_[i].changed) return true;
    }
    return false;
}

const char* BumperManager::name(uint8_t index)
{
    if (index >= BUMPER_MAX_SENSORS) return "unknown";
    return BUMPER_NAMES[index];
}
