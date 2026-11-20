#ifndef SONAR_MANAGER_H
#define SONAR_MANAGER_H

#include <Arduino.h>

#define SONAR_MAX_SENSORS 3
#define SONAR_TIMEOUT_US  8700      // ~1.5m
#define SONAR_SOUND_SPEED 0.000343f  // m/us a 25°C
#define SONAR_MIN_GAP_US  20000     // separacion minima entre disparos (un sonar cada 20 ms)

struct SonarReading {
    float distance;   // metros, -1.5 = timeout/sin obstáculo
    bool  valid;      // false si timeout
};

class SonarManager {
public:
    SonarManager(uint8_t trig_pin,
                 uint8_t echo_l,
                 uint8_t echo_c,
                 uint8_t echo_r);

    void begin();
    // NO bloqueante: dispara, y los flancos del eco los toma una ISR. Cada llamada
    // solo mira timestamps (microsegundos); un sonar desconectado no frena nada.
    void update(); // Llamar en cada loop()

    SonarReading getLeft()   const { return readings_[0]; }
    SonarReading getCenter() const { return readings_[1]; }
    SonarReading getRight()  const { return readings_[2]; }

    // true si alguno detecta obstáculo dentro de threshold_m
    bool obstacleDetected(float threshold_m) const;

    // Uso interno (ISR de los pines de eco)
    void IRAM_ATTR onEchoEdge(int idx);

private:
    uint8_t trig_pin_;
    uint8_t echo_pins_[SONAR_MAX_SENSORS];
    SonarReading readings_[SONAR_MAX_SENSORS];

    volatile int current_;      // índice del sonar midiendo (0=left, 1=center, 2=right)

    enum class State : uint8_t { IDLE, WAIT_ECHO };
    State    state_;
    uint32_t t_trig_us_;        // instante del disparo
    uint32_t next_trig_us_;     // no disparar antes de este instante
    volatile uint32_t t_rise_us_;   // 0 = aun no llego el flanco
    volatile uint32_t t_fall_us_;

    void finish(bool valid, float distance_m);
};

#endif