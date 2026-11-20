#include "SonarManager.h"

// Argumento de cada ISR: a que sonar pertenece el pin
struct SonarIsrArg { SonarManager* self; int idx; };
static SonarIsrArg g_sonar_isr_arg[SONAR_MAX_SENSORS];

static void IRAM_ATTR sonarIsr(void* arg)
{
    SonarIsrArg* a = (SonarIsrArg*)arg;
    a->self->onEchoEdge(a->idx);
}


SonarManager::SonarManager(uint8_t trig_pin,
                            uint8_t echo_l,
                            uint8_t echo_c,
                            uint8_t echo_r)
    : trig_pin_(trig_pin), current_(0),
      state_(State::IDLE), t_trig_us_(0), next_trig_us_(0),
      t_rise_us_(0), t_fall_us_(0)
{
    echo_pins_[0] = echo_l;
    echo_pins_[1] = echo_c;
    echo_pins_[2] = echo_r;

    for (int i = 0; i < SONAR_MAX_SENSORS; i++) {
        readings_[i] = { -1.0f, false };
    }
}

void SonarManager::begin()
{
    pinMode(trig_pin_, OUTPUT);
    digitalWrite(trig_pin_, LOW);
    for (int i = 0; i < SONAR_MAX_SENSORS; i++) {
        pinMode(echo_pins_[i], INPUT);
        g_sonar_isr_arg[i] = { this, i };
        attachInterruptArg(digitalPinToInterrupt(echo_pins_[i]), sonarIsr,
                           &g_sonar_isr_arg[i], CHANGE);
    }
}

void IRAM_ATTR SonarManager::onEchoEdge(int idx)
{
    if (state_ != State::WAIT_ECHO || idx != current_) return;
    uint32_t t = micros() | 1u;   // nunca 0 (0 = "sin flanco")
    if (digitalRead(echo_pins_[idx])) {
        if (t_rise_us_ == 0) t_rise_us_ = t;
    } else {
        if (t_rise_us_ != 0 && t_fall_us_ == 0) t_fall_us_ = t;
    }
}

void SonarManager::finish(bool valid, float distance_m)
{
    readings_[current_] = valid ? SonarReading{ distance_m, true }
                                : SonarReading{ -1.0f, false };
    next_trig_us_ = t_trig_us_ + SONAR_MIN_GAP_US;
    current_ = (current_ + 1) % SONAR_MAX_SENSORS;
    state_ = State::IDLE;
}

void SonarManager::update()
{
    uint32_t now = micros();

    if (state_ == State::IDLE) {
        if ((int32_t)(now - next_trig_us_) < 0) return;   // aun no toca
        t_rise_us_ = 0;
        t_fall_us_ = 0;
        state_ = State::WAIT_ECHO;
        // Disparar trigger (12 us en total; es lo unico que "espera")
        digitalWrite(trig_pin_, LOW);
        delayMicroseconds(2);
        digitalWrite(trig_pin_, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig_pin_, LOW);
        t_trig_us_ = micros();
        return;
    }

    // WAIT_ECHO: solo mirar los timestamps que dejo la ISR
    uint32_t rise = t_rise_us_;
    uint32_t fall = t_fall_us_;
    if (rise != 0 && fall != 0) {
        finish(true, ((fall - rise) * SONAR_SOUND_SPEED) / 2.0f);
    } else if (rise == 0 && (now - t_trig_us_) > SONAR_TIMEOUT_US) {
        finish(false, 0.0f);          // nunca subio el eco
    } else if (rise != 0 && (now - rise) > SONAR_TIMEOUT_US) {
        finish(false, 0.0f);          // el eco no bajo
    }
}

bool SonarManager::obstacleDetected(float threshold_m) const
{
    for (int i = 0; i < SONAR_MAX_SENSORS; i++) {
        if (readings_[i].valid && readings_[i].distance <= threshold_m) {
            return true;
        }
    }
    return false;
}
