#include "BuzzerManager.h"

// ── Definición de patrones ────────────────────────────────────
// {freq, on_ms, off_ms, repeats}
static const BuzzerStep PAT_CONNECTED[]    = { {2000, 100, 80, 2} };
static const BuzzerStep PAT_DISCONNECTED[] = { {1000, 400, 200, 3} };
static const BuzzerStep PAT_BATTERY_LOW[]  = { {800,  500, 500, 10} };
static const BuzzerStep PAT_START[]        = {
    {1000, 100, 50, 1},
    {1500, 100, 50, 1},
    {2000, 200, 0,  1}
};
static const BuzzerStep PAT_LIFTED[]       = { {3000, 100, 100, 20} };
static const BuzzerStep PAT_CUTTING[]      = { {1800, 600, 0,   1} };

// ── Constructor ───────────────────────────────────────────────
BuzzerManager::BuzzerManager(uint8_t pin)
    : pin_(pin), playing_(false), active_buzzer_(true),
      current_pattern_(BuzzerPattern::NONE),
      steps_(nullptr), total_steps_(0),
      current_step_(0), current_repeat_(0),
      step_start_(0), in_tone_(false)
{}

void BuzzerManager::begin()
{
    // Buzzer activo — solo digitalWrite, no necesita PWM
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
}

void BuzzerManager::play(BuzzerPattern pattern)
{
    // Si ya está tocando el mismo patrón no interrumpir
    if (playing_ && current_pattern_ == pattern) return;

    current_pattern_ = pattern;
    loadPattern(pattern);

    if (total_steps_ == 0) return;

    current_step_   = 0;
    current_repeat_ = 0;
    step_start_     = millis();
    in_tone_        = true;
    playing_        = true;

    startTone(steps_[0].freq);
}

void BuzzerManager::update()
{
    if (!playing_) return;

    const BuzzerStep& step = steps_[current_step_];
    unsigned long elapsed  = millis() - step_start_;

    if (in_tone_) {
        if (elapsed >= step.on_ms) {
            stopTone();
            in_tone_   = false;
            step_start_ = millis();
        }
    } else {
        if (elapsed >= step.off_ms) {
            current_repeat_++;

            if (current_repeat_ >= step.repeats) {
                // Avanzar al siguiente paso
                current_step_++;
                current_repeat_ = 0;

                if (current_step_ >= total_steps_) {
                    // Patrón terminado
                    playing_ = false;
                    current_pattern_ = BuzzerPattern::NONE;
                    return;
                }
            }

            // Siguiente repetición o paso
            step_start_ = millis();
            in_tone_    = true;
            startTone(steps_[current_step_].freq);
        }
    }
}

void BuzzerManager::loadPattern(BuzzerPattern pattern)
{
    switch (pattern) {
        case BuzzerPattern::CONNECTED:
            steps_ = PAT_CONNECTED;
            total_steps_ = sizeof(PAT_CONNECTED) / sizeof(BuzzerStep);
            break;
        case BuzzerPattern::DISCONNECTED:
            steps_ = PAT_DISCONNECTED;
            total_steps_ = sizeof(PAT_DISCONNECTED) / sizeof(BuzzerStep);
            break;
        case BuzzerPattern::BATTERY_LOW:
            steps_ = PAT_BATTERY_LOW;
            total_steps_ = sizeof(PAT_BATTERY_LOW) / sizeof(BuzzerStep);
            break;
        case BuzzerPattern::START:
            steps_ = PAT_START;
            total_steps_ = sizeof(PAT_START) / sizeof(BuzzerStep);
            break;
        case BuzzerPattern::LIFTED:
            steps_ = PAT_LIFTED;
            total_steps_ = sizeof(PAT_LIFTED) / sizeof(BuzzerStep);
            break;
        case BuzzerPattern::CUTTING:
            steps_ = PAT_CUTTING;
            total_steps_ = sizeof(PAT_CUTTING) / sizeof(BuzzerStep);
            break;
        default:
            steps_ = nullptr;
            total_steps_ = 0;
            break;
    }
}

void BuzzerManager::startTone(uint16_t freq)
{
    // Buzzer activo — no necesita frecuencia, solo ON/OFF
    digitalWrite(pin_, HIGH);
}

void BuzzerManager::stopTone()
{
    digitalWrite(pin_, LOW);
}
