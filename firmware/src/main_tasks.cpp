// Firmware del robot organizado en tareas de FreeRTOS (entorno mow_tasks).
//
//   control  (nucleo 1, prio 5): encoders, PID, PWM y odometria a 50 Hz
//   microros (nucleo 0, prio 3): UNICA tarea que llama a rcl/rclc
//   sensors  (nucleo 1, prio 2): sonares, bumpers, INA228, OLED y buzzer
//
// El entorno mow (main.cpp) no se toca: es el respaldo.
#include <Arduino.h>

#include "Motor.h"
#include "SharedState.h"
#include "TaskLogic.h"

// ── Motores (mismos pines/canales que main.cpp) ─────────────────────────────
const float PID_I_TERM_MAX_PWM = 80.0f;
Motor motor_der(18, 19, 0, 1, 70, 230, 1.4f, 15.0f, 0.5f, 0.02f, PID_I_TERM_MAX_PWM);
Motor motor_izq(26, 27, 2, 3, 70, 230, 2.4f, 15.0f, 0.5f, 0.02f, PID_I_TERM_MAX_PWM);

void setup()
{
    Serial.begin(921600);
    // Esqueleto: motores en PWM 0 hasta que existan las tareas
    motor_der.begin();
    motor_izq.begin();
}

void loop()
{
    // Todo el trabajo va a estar en tareas propias; loopTask no se usa
    vTaskDelete(NULL);
}
