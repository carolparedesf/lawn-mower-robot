#ifndef BUZZER_MANAGER_H
#define BUZZER_MANAGER_H

#include <Arduino.h>

// Canal LEDC para el buzzer — usar uno libre (4)
#define BUZZER_LEDC_CH   4
#define BUZZER_FREQ      2000   // Hz tono base
#define BUZZER_RES       8

// Patrones disponibles
enum class BuzzerPattern {
    NONE,
    CONNECTED,       // micro-ROS conectado — 2 beeps cortos
    DISCONNECTED,    // micro-ROS perdido   — 3 beeps largos
    BATTERY_LOW,     // batería baja        — beep continuo lento
    START,           // inicio navegación   — tono ascendente
    LIFTED,          // levantado del suelo — beep rápido continuo
    CUTTING          // inicio corte        — 1 beep largo
};

struct BuzzerStep {
    uint16_t freq;      // 0 = silencio
    uint16_t on_ms;     // duración tono
    uint16_t off_ms;    // silencio después
    uint8_t  repeats;   // veces a repetir
};

class BuzzerManager {
public:
    BuzzerManager(uint8_t pin);

    void begin();
    void play(BuzzerPattern pattern);
    void update();          // llamar en loop — no bloqueante
    bool isPlaying() const  { return playing_; }

private:
    uint8_t  pin_;
    bool     playing_;
    bool     active_buzzer_;   // true = buzzer activo (sin PWM)

    BuzzerPattern    current_pattern_;
    const BuzzerStep* steps_;
    uint8_t          total_steps_;
    uint8_t          current_step_;
    uint8_t          current_repeat_;
    unsigned long    step_start_;
    bool             in_tone_;   // true = sonando, false = silencio

    void loadPattern(BuzzerPattern pattern);
    void startTone(uint16_t freq);
    void stopTone();
};

#endif
