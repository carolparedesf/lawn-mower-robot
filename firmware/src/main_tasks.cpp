// Firmware del robot organizado en tareas de FreeRTOS (entorno mow_tasks).
//
//   control  (nucleo 1, prio 5): encoders, PID, PWM y odometria a 50 Hz
//   microros (nucleo 0, prio 3): UNICA tarea que llama a rcl/rclc
//   sensors  (nucleo 1, prio 2): sonares, bumpers, INA228, OLED y buzzer
//
// Datos compartidos: lib/shared_state (un escritor por dato, copias en bloque).
// El entorno mow (main.cpp) no se toca: es el respaldo.
#include <Arduino.h>

#include "Motor.h"
#include "DiffDrive.h"
#include "encoder.h"
#include "SharedState.h"
#include "TaskLogic.h"

// ── Parametros ──────────────────────────────────────────────────────────────
#define CONTROL_HZ         50
#define CONTROL_PERIOD_MS  (1000 / CONTROL_HZ)
#define CMD_TIMEOUT_MS     500     // orden mas vieja: PWM 0 e integral 0

// ── Tareas: nucleo, prioridad, stack (bytes) ────────────────────────────────
#define CONTROL_CORE   1
#define CONTROL_PRIO   5
#define CONTROL_STACK  4096

// ── Motores (mismos pines, canales y ganancias que main.cpp) ────────────────
const float PID_I_TERM_MAX_PWM = 80.0f;
Motor motor_der(18, 19, 0, 1, 70, 230, 1.4f, 15.0f, 0.5f, 0.02f, PID_I_TERM_MAX_PWM);
Motor motor_izq(26, 27, 2, 3, 70, 230, 2.4f, 15.0f, 0.5f, 0.02f, PID_I_TERM_MAX_PWM);

// ── Encoders ────────────────────────────────────────────────────────────────
Encoder enc_der(25, 13, 815, false);
Encoder enc_izq(32, 33, 814, true);

// ── DiffDrive (solo lo usa la tarea de control) ─────────────────────────────
DiffDriveConfig dd_cfg = {
    .wheel_radius = 0.0925f,
    .base_width   = 0.355f,
    .dt           = 0.02f
};
DiffDrive robot(motor_der, motor_izq, enc_der, enc_izq, dd_cfg);

// ════════════════════════════════════════════════════════════════════════════
// Tarea de control: 50 Hz exactos con vTaskDelayUntil
// ════════════════════════════════════════════════════════════════════════════
static void controlTask(void*)
{
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    tl::PeriodStats stats;
    uint32_t prev_us = 0;
    uint32_t seq     = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        // Periodo real entre activaciones
        const uint32_t now_us = micros();
        if (prev_us != 0) stats.add(now_us - prev_us);
        prev_us = now_us;

        // Orden y protecciones (copias, sin esperar a nadie)
        const CmdVelShared cmd = shared::getCmdVel();
        const bool fresh = tl::cmdFresh(millis(), cmd.stamp_ms, CMD_TIMEOUT_MS);
        const bool enabled = tl::driveEnabled(shared::agentConnected(), fresh,
                                              shared::estop_external.load(),
                                              shared::estop_lifted.load(),
                                              shared::battery_low.load());

        // Deshabilitado u orden 0: referencia 0 => DiffDrive deja PWM 0 e
        // integral 0 (el PWM minimo solo se aplica con orden distinta de 0).
        if (enabled && !tl::isZeroCmd(cmd.v, cmd.w)) robot.setCmdVel(cmd.v, cmd.w);
        else                                          robot.setCmdVel(0.0f, 0.0f);
        robot.update(enabled);

        // Odometria para la tarea micro-ROS
        shared::setControl({robot.getOdometry(), ++seq});

        if (stats.count() >= CONTROL_HZ) shared::setPeriod(stats.take());
    }
}

// ════════════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(921600);

    robot.begin();   // motores en PWM 0 y limite de salida del PID

    xTaskCreatePinnedToCore(controlTask, "control", CONTROL_STACK, nullptr,
                            CONTROL_PRIO, nullptr, CONTROL_CORE);
}

void loop()
{
    // Todo el trabajo esta en tareas propias; loopTask no se usa
    vTaskDelete(NULL);
}
