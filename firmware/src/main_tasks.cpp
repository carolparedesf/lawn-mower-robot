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
#include "SonarManager.h"
#include "BumperManager.h"
#include "BuzzerManager.h"
#include "BatteryManager.h"
#include "DisplayManager.h"
#include "SharedState.h"
#include "TaskLogic.h"

// ── Parametros ──────────────────────────────────────────────────────────────
#define CONTROL_HZ         50
#define CONTROL_PERIOD_MS  (1000 / CONTROL_HZ)
#define CMD_TIMEOUT_MS     500     // orden mas vieja: PWM 0 e integral 0

#define SENSORS_HZ         20
#define SENSORS_PERIOD_MS  (1000 / SENSORS_HZ)
#define EVENT_DEDUP_MS     2000    // mismo evento: no repetir el sonido antes
#define EVENT_QUEUE_LEN    8

// Bateria: el PID se deshabilita solo si el INA228 MIDE menos de BAT_RUN_MIN_V
// (vuelve con BAT_RUN_MIN_V + BAT_RUN_HYST_V). "Sin dato" no frena.
const float BAT_RUN_MIN_V  = 15.0f;
const float BAT_RUN_HYST_V = 1.0f;

// ── Tareas: nucleo, prioridad, stack (bytes) ────────────────────────────────
#define CONTROL_CORE   1
#define CONTROL_PRIO   5
#define CONTROL_STACK  4096

#define SENSORS_CORE   1
#define SENSORS_PRIO   2
#define SENSORS_STACK  8192

#define BUZZER_PIN 2

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

// ── Sensores, buzzer y display (solo los usa la tarea de sensores) ──────────
SonarManager sonars(15, 35, 34, 36);
const uint8_t BUMPER_PINS[BUMPER_MAX_SENSORS] = {17, 16, 23, 5, 4};
BumperManager bumpers(BUMPER_PINS, 5);
BatteryManager battery(0.002f, 20.0f);
DisplayManager display;

// Eventos de /robot/events: microros -> sensors (tl::Event, 1 byte)
static QueueHandle_t g_event_queue = nullptr;

// ── Buzzer (pin 2, activo: solo ON/OFF) ─────────────────────────────────────
// Se maneja con tl::BeepSequencer y no con BuzzerManager para no modificar
// esa libreria (el binario de mow queda identico). Mismos ritmos que
// BuzzerManager para conectado/desconectado.          {on_ms, off_ms, rep}
static tl::BeepSequencer beeper;
static const tl::Beep SND_CONNECTED[]    = { {100, 80, 2} };    // 2 cortos
static const tl::Beep SND_DISCONNECTED[] = { {400, 200, 3} };   // 3 largos

// Eventos: cortos (<= 1.6 s), distintos entre si y de los dos de arriba.
// Tiempos en multiplos de 50 ms porque el buzzer se actualiza a 20 Hz.
static const tl::Beep SND_LIFTED[]      = { {100, 100, 5} };    // 5 rapidos
static const tl::Beep SND_LOW_BATTERY[] = { {500, 300, 2} };    // 2 largos
static const tl::Beep SND_CUTTING[]     = { {800, 0, 1} };      // 1 muy largo
static const tl::Beep SND_NAVIGATION[]  = {                     // corto-corto-largo
    {100, 100, 2}, {400, 0, 1} };
static const tl::Beep SND_PERIMETER[]   = {                     // largo-corto-largo
    {400, 150, 1}, {100, 150, 1}, {400, 0, 1} };

#define SND(a) (a), (uint8_t)(sizeof(a) / sizeof(tl::Beep))

static void playEvent(tl::Event ev, uint32_t now)
{
    switch (ev) {
        case tl::Event::LIFTED:             beeper.play(SND(SND_LIFTED), now);      break;
        case tl::Event::LOW_BATTERY:        beeper.play(SND(SND_LOW_BATTERY), now); break;
        case tl::Event::CUTTING_STARTED:    beeper.play(SND(SND_CUTTING), now);     break;
        case tl::Event::NAVIGATION_STARTED: beeper.play(SND(SND_NAVIGATION), now);  break;
        case tl::Event::PERIMETER_STARTED:  beeper.play(SND(SND_PERIMETER), now);   break;
        default: break;
    }
}

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
// Tarea de sensores: 20 Hz. Unica duena de sonares, bumpers, INA228 (Wire),
// OLED (Wire) y buzzer (pin 2). Ninguna lectura bloquea mas de ~5 ms; la excepcion
// aceptada es el OLED (~30 ms, una vez por segundo).
// ════════════════════════════════════════════════════════════════════════════
static void sensorsTask(void*)
{
    const TickType_t period = pdMS_TO_TICKS(SENSORS_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    tl::LiftLatch   lift_latch;
    tl::BatteryGate bat_gate(BAT_RUN_MIN_V, BAT_RUN_HYST_V);
    tl::EventDedup  dedup(EVENT_DEDUP_MS);
    tl::RisingEdge  low_bat_edge;

    uint32_t seen_clear     = shared::estop_clear_seq.load();
    uint8_t  prev_agent     = shared::agent_status.load();
    uint32_t last_wait_beep = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        const uint32_t now = millis();

        // ── Sonares: la 1a llamada cierra la medicion en curso y la 2a
        // dispara el siguiente sonar (si no, a 20 Hz se mediria uno cada 100 ms)
        sonars.update();
        sonars.update();

        bumpers.update();
        battery.update();   // lee cada 500 ms, timeout I2C de 5 ms

        // ── Bateria baja MEDIDA (sin dato => no frena) ─────────────────────
        bat_gate.update(battery.isReady(), battery.getVoltage());
        shared::battery_low.store(bat_gate.low());

        // ── Levantado: enganchado hasta /estop false ───────────────────────
        bool b[BUMPER_MAX_SENSORS];
        for (int i = 0; i < BUMPER_MAX_SENSORS; i++) b[i] = bumpers.get(i).pressed;

        const uint32_t clear_seq = shared::estop_clear_seq.load();
        const bool clear_requested = (clear_seq != seen_clear);
        seen_clear = clear_seq;
        // Si los bumpers siguen apretados al liberar, se vuelve a enganchar
        const bool lifted_edge = lift_latch.step(clear_requested,
                                                 tl::liftedFromBumpers(b));
        shared::estop_lifted.store(lift_latch.active());

        // ── Eventos: locales y de /robot/events, sin repetir en < 2 s ──────
        // ── Conexion micro-ROS: sonidos como en main.cpp ───────────────────
        // (antes que los eventos: si coinciden, el evento es el que suena)
        const uint8_t agent = shared::agent_status.load();
        if (agent != prev_agent) {
            if (agent == (uint8_t)AgentStatus::CONNECTED)
                beeper.play(SND(SND_CONNECTED), now);
            else if (prev_agent == (uint8_t)AgentStatus::CONNECTED)
                beeper.play(SND(SND_DISCONNECTED), now);
            prev_agent = agent;
        } else if (agent == (uint8_t)AgentStatus::WAITING && !beeper.playing()
                   && now - last_wait_beep >= 1000) {
            last_wait_beep = now;               // esperando agente al arrancar
            beeper.play(SND(SND_DISCONNECTED), now);
        }

        // ── Eventos: locales y de /robot/events, sin repetir en < 2 s ──────
        if (lifted_edge && dedup.shouldPlay(tl::Event::LIFTED, now))
            playEvent(tl::Event::LIFTED, now);

        const bool low_now = battery.isReady() && (battery.isLow() || bat_gate.low());
        if (low_bat_edge.step(low_now) && dedup.shouldPlay(tl::Event::LOW_BATTERY, now))
            playEvent(tl::Event::LOW_BATTERY, now);

        uint8_t ev_raw;
        while (g_event_queue && xQueueReceive(g_event_queue, &ev_raw, 0) == pdTRUE) {
            const tl::Event ev = (tl::Event)ev_raw;
            if (dedup.shouldPlay(ev, now)) playEvent(ev, now);
        }

        // ── Buzzer y display ───────────────────────────────────────────────
        static bool pin_on = false;
        const bool on = beeper.step(now);
        if (on != pin_on) { digitalWrite(BUZZER_PIN, on ? HIGH : LOW); pin_on = on; }

        BuzzerPattern display_state =
            lift_latch.active()                          ? BuzzerPattern::LIFTED
            : (agent == (uint8_t)AgentStatus::CONNECTED) ? BuzzerPattern::CONNECTED
                                                         : BuzzerPattern::DISCONNECTED;
        display.update(battery, display_state);   // redibuja a 1 Hz (~30 ms)

        // ── Copia para la tarea micro-ROS ──────────────────────────────────
        SensorSnap s;
        const SonarReading sr[3] = {sonars.getLeft(), sonars.getCenter(), sonars.getRight()};
        for (int i = 0; i < 3; i++) {
            s.range[i]       = sr[i].distance;
            s.range_valid[i] = sr[i].valid;
        }
        for (int i = 0; i < BUMPER_MAX_SENSORS; i++) s.bumper[i] = b[i];
        s.bat_valid = battery.isReady();
        s.bat_v     = battery.getVoltage();
        s.bat_i     = battery.getCurrent();
        s.bat_pct   = battery.getPercent();
        shared::setSensors(s);
    }
}

// ════════════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(921600);

    robot.begin();   // motores en PWM 0 y limite de salida del PID
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    sonars.begin();  // ISR de eco en el nucleo 1 (setup corre en el nucleo 1)
    bumpers.begin();
    battery.begin(); // Wire + INA228; si no responde queda "sin dato"
    display.begin(); // splash no bloqueante

    g_event_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(uint8_t));

    xTaskCreatePinnedToCore(controlTask, "control", CONTROL_STACK, nullptr,
                            CONTROL_PRIO, nullptr, CONTROL_CORE);
    xTaskCreatePinnedToCore(sensorsTask, "sensors", SENSORS_STACK, nullptr,
                            SENSORS_PRIO, nullptr, SENSORS_CORE);
}

void loop()
{
    // Todo el trabajo esta en tareas propias; loopTask no se usa
    vTaskDelete(NULL);
}
