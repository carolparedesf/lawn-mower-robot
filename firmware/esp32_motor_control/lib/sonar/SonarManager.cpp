#include "SonarManager.h"

SonarManager::SonarManager(uint8_t trig_pin,
                            uint8_t echo_l,
                            uint8_t echo_c,
                            uint8_t echo_r)
    : trig_pin_(trig_pin), current_(0), last_time_(0)
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
    }
}

void SonarManager::update()
{
    if (millis() - last_time_ < SONAR_PERIOD_MS) return;

    float d = medir(echo_pins_[current_]);
    readings_[current_].distance = d;
    readings_[current_].valid    = (d >= 0.0f);

    //delay(SONAR_DELAY_MS);
    current_ = (current_ + 1) % SONAR_MAX_SENSORS;
    last_time_ = millis();
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

float SonarManager::medir(uint8_t echo_pin)
{
    digitalWrite(trig_pin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);

    unsigned long t0 = micros();
    while (digitalRead(echo_pin) == LOW) {
        if (micros() - t0 > SONAR_TIMEOUT_US) return -1.0f;
    }

    unsigned long rise = micros();
    while (digitalRead(echo_pin) == HIGH) {
        if (micros() - rise > SONAR_TIMEOUT_US) return -1.0f;
    }

    unsigned long dt = micros() - rise;
    return (dt * SONAR_SOUND_SPEED) / 2.0f;
}
