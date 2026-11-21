// Firmware del robot organizado en tareas de FreeRTOS (entorno mow_tasks).
//
//   control  (nucleo 1, prio 5): encoders, PID, PWM y odometria a 50 Hz
//   microros (nucleo 0, prio 3): UNICA tarea que llama a rcl/rclc
//   sensors  (nucleo 1, prio 2): sonares, bumpers, INA228, OLED y buzzer
//
// Datos compartidos: lib/shared_state (un escritor por dato, copias en bloque).
// El entorno mow (main.cpp) no se toca: es el respaldo.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>

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

// Odometria: publicacion (Hz). Si es menor que CONTROL_HZ se publica la
// ultima muestra cada N = CONTROL_HZ / ODOM_PUB_HZ ciclos de control.
#define ODOM_PUB_HZ        50

#define DOMAIN_ID          42      // ROS_DOMAIN_ID del nodo micro-ROS
#define PING_INTERVAL_MS   200
#define PING_TIMEOUT_MS    100
#define PING_MAX_FAILS     3       // pings fallidos seguidos para desconectar
#define EVENT_MAX_LEN      32      // capacity del string de /robot/events

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

#define MICROROS_CORE  0
#define MICROROS_PRIO  3
#define MICROROS_STACK 20480

static_assert(ODOM_PUB_HZ >= 1 && ODOM_PUB_HZ <= CONTROL_HZ,
              "ODOM_PUB_HZ debe estar entre 1 y CONTROL_HZ");

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
// Tarea micro-ROS: nucleo 0. UNICA que llama a rcl/rclc/rmw (no son
// thread-safe). Todo lo que recibe lo deja en shared:: o en la cola de eventos.
// ════════════════════════════════════════════════════════════════════════════
// Entidades: 10 publishers (tope 14 en colcon_tasks.meta), 3 suscripciones
// (tope 6), 5 handles en el executor (3 subs + 2 timers).
static rcl_subscription_t sub_cmdvel, sub_estop, sub_events;
static rcl_publisher_t    pub_odom, pub_battery;
static rcl_publisher_t    pub_sonar[3];
static rcl_publisher_t    pub_bumper[BUMPER_MAX_SENSORS];
static rcl_timer_t        timer_sensors;   // 5 Hz: sonares y bumpers
static rcl_timer_t        timer_battery;   // 1 Hz
static rclc_executor_t    executor;
static rclc_support_t     support;
static rcl_allocator_t    allocator;
static rcl_node_t         node;
static bool               g_support_ok = false;
static bool               g_node_ok    = false;

static geometry_msgs__msg__Twist      msg_cmdvel;
static std_msgs__msg__Bool            msg_estop_in;
static std_msgs__msg__String          msg_event;
static char                           event_buf[EVENT_MAX_LEN];   // memoria fija
static nav_msgs__msg__Odometry        msg_odom;
static sensor_msgs__msg__BatteryState msg_battery;
static sensor_msgs__msg__Range        msg_sonar[3];
static std_msgs__msg__Bool            msg_bumper[BUMPER_MAX_SENSORS];

// Ignorar el resultado (como RCSOFTCHECK de main.cpp; (void) no silencia
// warn_unused_result en gcc)
#define SOFT(fn) do { rcl_ret_t rc_ = (fn); (void)rc_; } while (0)

// Asignar string rosidl (reusa el buffer si alcanza)
static void set_string(rosidl_runtime_c__String& field, const char* str)
{
    size_t len = strlen(str);
    if (field.capacity <= len) {
        if (field.data) allocator.deallocate(field.data, allocator.state);
        field.data     = (char*)allocator.allocate(len + 1, allocator.state);
        field.capacity = len + 1;
    }
    memcpy(field.data, str, len + 1);
    field.size = len;
}

static inline builtin_interfaces__msg__Time get_ros_time()
{
    int64_t ns = rmw_uros_epoch_nanos();
    builtin_interfaces__msg__Time t;
    t.sec     = (int32_t)(ns / 1000000000LL);
    t.nanosec = (uint32_t)(ns % 1000000000LL);
    return t;
}

// ── Callbacks de suscripciones ──────────────────────────────────────────────
static void cmd_vel_cb(const void* msg_in)
{
    // Con e-stop activo no se guarda la orden: al liberar hace falta una nueva
    if (shared::anyEstop()) { shared::clearCmdVel(); return; }
    const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msg_in;
    shared::setCmdVel((float)m->linear.x, (float)m->angular.z, millis());
}

static void estop_cb(const void* msg_in)
{
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg_in;
    shared::clearCmdVel();
    if (m->data) {
        shared::estop_external.store(true);
    } else {
        // /estop false limpia las dos: estop_external aca, y estop_lifted la
        // limpia sensors al ver el nuevo estop_clear_seq (y la re-engancha si
        // los bumpers de levantado siguen apretados).
        shared::estop_external.store(false);
        shared::estop_clear_seq.fetch_add(1);
    }
}

static void events_cb(const void* msg_in)
{
    const std_msgs__msg__String* m = (const std_msgs__msg__String*)msg_in;
    // Un texto mas largo que capacity no llega a este callback (micro-ROS no
    // puede deserializarlo y lo descarta); igual se valida antes de leer.
    if (m->data.data == nullptr || m->data.size >= EVENT_MAX_LEN) return;
    size_t len = m->data.size;
    while (len > 0 && m->data.data[len - 1] == '\0') len--;
    const tl::Event ev = tl::parseEvent(m->data.data, len);
    if (ev == tl::Event::NONE || g_event_queue == nullptr) return;
    const uint8_t raw = (uint8_t)ev;
    xQueueSend(g_event_queue, &raw, 0);   // cola llena: se pierde, no se espera
}

// ── Timers de publicacion ───────────────────────────────────────────────────
static void timer_sensors_cb(rcl_timer_t* timer, int64_t)
{
    if (timer == NULL) return;
    const SensorSnap s = shared::getSensors();
    const builtin_interfaces__msg__Time now = get_ros_time();
    for (int i = 0; i < 3; i++) {
        msg_sonar[i].header.stamp = now;
        msg_sonar[i].range = s.range_valid[i] ? s.range[i] : msg_sonar[i].max_range;
        SOFT(rcl_publish(&pub_sonar[i], &msg_sonar[i], NULL));
    }
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        msg_bumper[i].data = s.bumper[i];
        SOFT(rcl_publish(&pub_bumper[i], &msg_bumper[i], NULL));
    }
}

static void timer_battery_cb(rcl_timer_t* timer, int64_t)
{
    if (timer == NULL) return;
    const SensorSnap s = shared::getSensors();
    msg_battery.header.stamp = get_ros_time();
    // Sin dato del INA228: NAN y present=false (como main.cpp)
    msg_battery.voltage    = s.bat_valid ? s.bat_v : NAN;
    msg_battery.current    = s.bat_valid ? s.bat_i : NAN;
    msg_battery.percentage = s.bat_valid ? s.bat_pct / 100.0f : NAN;
    msg_battery.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    msg_battery.present = s.bat_valid;
    SOFT(rcl_publish(&pub_battery, &msg_battery, NULL));
}

static void publish_odom(const Odometry& o)
{
    msg_odom.header.stamp          = get_ros_time();
    msg_odom.pose.pose.position.x  = o.x;
    msg_odom.pose.pose.position.y  = o.y;
    msg_odom.twist.twist.linear.x  = o.vx;
    msg_odom.twist.twist.angular.z = o.wz;
    msg_odom.pose.pose.orientation.x = 0.0f;
    msg_odom.pose.pose.orientation.y = 0.0f;
    msg_odom.pose.pose.orientation.z = sinf(o.theta / 2.0f);
    msg_odom.pose.pose.orientation.w = cosf(o.theta / 2.0f);
    msg_odom.pose.covariance[0]   = 0.01;   // x
    msg_odom.pose.covariance[7]   = 0.01;   // y
    msg_odom.pose.covariance[35]  = 0.05;   // yaw
    msg_odom.twist.covariance[0]  = 0.01;   // vx
    msg_odom.twist.covariance[35] = 0.05;   // wz
    SOFT(rcl_publish(&pub_odom, &msg_odom, NULL));
}

// ── Crear / destruir entidades ──────────────────────────────────────────────
static void destroy_entities();

static bool create_entities()
{
    // Todo en cero: destroy_entities() puede llamarse tras un fallo parcial
    sub_cmdvel = rcl_get_zero_initialized_subscription();
    sub_estop  = rcl_get_zero_initialized_subscription();
    sub_events = rcl_get_zero_initialized_subscription();
    pub_odom    = rcl_get_zero_initialized_publisher();
    pub_battery = rcl_get_zero_initialized_publisher();
    for (int i = 0; i < 3; i++) pub_sonar[i] = rcl_get_zero_initialized_publisher();
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++)
        pub_bumper[i] = rcl_get_zero_initialized_publisher();
    timer_sensors = rcl_get_zero_initialized_timer();
    timer_battery = rcl_get_zero_initialized_timer();
    executor      = rclc_executor_get_zero_initialized_executor();

    allocator = rcl_get_default_allocator();

    // Init options con ROS_DOMAIN_ID = DOMAIN_ID. rclc_support_fini() las libera.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
    if (rcl_init_options_set_domain_id(&init_options, DOMAIN_ID) != RCL_RET_OK ||
        rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)
            != RCL_RET_OK) {
        SOFT(rcl_init_options_fini(&init_options));
        return false;
    }
    g_support_ok = true;

#define TRY(x) do { if ((x) != RCL_RET_OK) { destroy_entities(); return false; } } while (0)

    TRY(rclc_node_init_default(&node, "mow_base_node", "", &support));
    g_node_ok = true;
    TRY(rmw_uros_sync_session(1000));

    // ── Suscripciones (QoS por defecto, como main.cpp) ──────────────────────
    TRY(rclc_subscription_init_default(&sub_cmdvel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    TRY(rclc_subscription_init_default(&sub_estop, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "estop"));
    TRY(rclc_subscription_init_default(&sub_events, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "robot/events"));

    // ── Publishers - RELIABLE (default) ─────────────────────────────────────
    TRY(rclc_publisher_init_default(&pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
    TRY(rclc_publisher_init_default(&pub_battery, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery/state"));
    const char* sonar_topics[3] = {"sonar/left", "sonar/center", "sonar/right"};
    for (int i = 0; i < 3; i++)
        TRY(rclc_publisher_init_default(&pub_sonar[i], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), sonar_topics[i]));
    const char* bumper_topics[BUMPER_MAX_SENSORS] = {
        "bumper/B1_front_left", "bumper/B2_front_right",
        "bumper/B3_left", "bumper/B4_right", "bumper/B5_back"
    };
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        TRY(rclc_publisher_init_default(&pub_bumper[i], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), bumper_topics[i]));
        msg_bumper[i].data = false;
    }

    // ── Mensajes ────────────────────────────────────────────────────────────
    const char* sonar_frames[3] = {"SonarL_Link", "SonarC_Link", "SonarR_Link"};
    for (int i = 0; i < 3; i++) {
        set_string(msg_sonar[i].header.frame_id, sonar_frames[i]);
        msg_sonar[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        msg_sonar[i].field_of_view  = 0.2618f;
        msg_sonar[i].min_range      = 0.02f;
        msg_sonar[i].max_range      = 1.5f;
        msg_sonar[i].range          = 1.5f;
    }
    set_string(msg_odom.header.frame_id, "odom");
    set_string(msg_odom.child_frame_id, "diffDrive_Link");
    set_string(msg_battery.header.frame_id, "battery");

    // String de /robot/events con memoria fija (no usa el allocator)
    msg_event.data.data     = event_buf;
    msg_event.data.capacity = EVENT_MAX_LEN;
    msg_event.data.size     = 0;

    // ── Timers ──────────────────────────────────────────────────────────────
    TRY(rclc_timer_init_default(&timer_sensors, &support, RCL_MS_TO_NS(200),
                                timer_sensors_cb));
    TRY(rclc_timer_init_default(&timer_battery, &support, RCL_MS_TO_NS(1000),
                                timer_battery_cb));

    // ── Executor: 3 subs + 2 timers ─────────────────────────────────────────
    TRY(rclc_executor_init(&executor, &support.context, 5, &allocator));
    TRY(rclc_executor_add_subscription(&executor, &sub_cmdvel, &msg_cmdvel,
                                       &cmd_vel_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_subscription(&executor, &sub_estop, &msg_estop_in,
                                       &estop_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_subscription(&executor, &sub_events, &msg_event,
                                       &events_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_timer(&executor, &timer_sensors));
    TRY(rclc_executor_add_timer(&executor, &timer_battery));
#undef TRY
    return true;
}

static void destroy_entities()
{
    if (g_support_ok) {
        rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
        SOFT(rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0));
    }
    if (g_node_ok) {
        // fini sobre entidades en cero (fallo parcial) no hace nada
        SOFT(rcl_publisher_fini(&pub_odom, &node));
        SOFT(rcl_publisher_fini(&pub_battery, &node));
        for (int i = 0; i < 3; i++) SOFT(rcl_publisher_fini(&pub_sonar[i], &node));
        for (int i = 0; i < BUMPER_MAX_SENSORS; i++)
            SOFT(rcl_publisher_fini(&pub_bumper[i], &node));
        SOFT(rcl_subscription_fini(&sub_cmdvel, &node));
        SOFT(rcl_subscription_fini(&sub_estop, &node));
        SOFT(rcl_subscription_fini(&sub_events, &node));
    }
    SOFT(rcl_timer_fini(&timer_sensors));
    SOFT(rcl_timer_fini(&timer_battery));
    SOFT(rclc_executor_fini(&executor));
    if (g_node_ok)    SOFT(rcl_node_fini(&node));
    if (g_support_ok) SOFT(rclc_support_fini(&support));
    g_node_ok    = false;
    g_support_ok = false;
}

static void microrosTask(void*)
{
    set_microros_serial_transports(Serial);

    tl::PingMonitor   ping_mon(PING_MAX_FAILS);
    tl::OdomDecimator odom_dec(tl::odomDecimation(CONTROL_HZ, ODOM_PUB_HZ));
    bool     connected = false;
    uint32_t last_ping = 0;

    for (;;) {
        const uint32_t now = millis();

        // ── Sin agente: intentar conectar cada PING_INTERVAL_MS ────────────
        if (!connected) {
            if (now - last_ping >= PING_INTERVAL_MS) {
                last_ping = now;
                if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK &&
                    create_entities()) {
                    connected = true;
                    ping_mon.reset();
                    odom_dec.reset();
                    shared::clearCmdVel();
                    shared::agent_status.store((uint8_t)AgentStatus::CONNECTED);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ── Conectado: 3 pings fallidos seguidos para desconectar ──────────
        if (now - last_ping >= PING_INTERVAL_MS) {
            last_ping = now;
            const bool ok = (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK);
            if (ping_mon.onPing(ok)) {
                // Primero frenar (control ve "sin agente" y "sin orden"), y
                // limpiar SOLO estop_external: estop_lifted se mantiene.
                shared::agent_status.store((uint8_t)AgentStatus::DISCONNECTED);
                shared::clearCmdVel();
                shared::estop_external.store(false);
                destroy_entities();
                connected = false;
                continue;
            }
        }

        SOFT(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

        // ── Odometria: una vez por muestra nueva (o cada N con ODOM_PUB_HZ) ─
        const ControlSnap c = shared::getControl();
        if (odom_dec.shouldPublish(c.seq)) publish_odom(c.odom);

        vTaskDelay(1);   // ceder el nucleo 0 (idle / watchdog)
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
    xTaskCreatePinnedToCore(microrosTask, "microros", MICROROS_STACK, nullptr,
                            MICROROS_PRIO, nullptr, MICROROS_CORE);
}

void loop()
{
    // Todo el trabajo esta en tareas propias; loopTask no se usa
    vTaskDelete(NULL);
}
