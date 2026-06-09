#ifndef SONAR_MANAGER_H
#define SONAR_MANAGER_H

#include <Arduino.h>

#define SONAR_MAX_SENSORS 3
#define SONAR_TIMEOUT_US  6000    // ~1m
#define SONAR_SOUND_SPEED 0.000346f  // m/us a 25°C
#define SONAR_DELAY_MS    20      // pausa entre disparos
#define SONAR_PERIOD_MS   50      // período por sonar

struct SonarReading {
    float distance;   // metros, -1.0 = timeout/sin obstáculo
    bool  valid;      // false si timeout
};

class SonarManager {
public:
    SonarManager(uint8_t trig_pin,
                 uint8_t echo_l,
                 uint8_t echo_c,
                 uint8_t echo_r);

    void begin();

    // Llamar en cada loop() — maneja rotación interna
    void update();

    SonarReading getLeft()   const { return readings_[0]; }
    SonarReading getCenter() const { return readings_[1]; }
    SonarReading getRight()  const { return readings_[2]; }

    // true si alguno detecta obstáculo dentro de threshold_m
    bool obstacleDetected(float threshold_m) const;

private:
    uint8_t trig_pin_;
    uint8_t echo_pins_[SONAR_MAX_SENSORS];
    SonarReading readings_[SONAR_MAX_SENSORS];

    int current_;
    unsigned long last_time_;

    float medir(uint8_t echo_pin);
};

#endif