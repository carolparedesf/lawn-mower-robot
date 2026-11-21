#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/battery_state.h>
#include <rosidl_runtime_c/string_functions.h>

#include "Motor.h"
#include "DiffDrive.h"
#include "encoder.h"
#include "SonarManager.h"
#include "BumperManager.h"
#include "BuzzerManager.h"
#include "BatteryManager.h"
#include "DisplayManager.h"

#define BUZZER_PIN 2

// ── Variables FSM ────────────────────────────────────────────
#define PING_INTERVAL_MS  200
unsigned long last_ping_ms = 0;
bool          ping_ok      = false;

// ── FSM estados ──────────────────────────────────────────────
enum class AgentState {
    WAITING_AGENT,      // esperando conexión
    AGENT_CONNECTED,    // conectado y operando
    AGENT_DISCONNECTED  // perdió conexión — intentar reconectar
};
AgentState agent_state = AgentState::WAITING_AGENT;

// ── Watchdog cmd_vel ─────────────────────────────────────────
#define CMD_WATCHDOG_MS 500
unsigned long last_cmd_ms = 0;

// ── Control loop ─────────────────────────────────────────────
const int     PERIOD_MS = 20; // 50 Hz
unsigned long last_ctrl = 0;
float         ctrl_hz   = 0.0f;
unsigned long ctrl_count = 0;
unsigned long ctrl_hz_ts = 0;

float v = 0.0f, w = 0.0f;

// ── E-Stop ───────────────────────────────────────────────────
bool estop = false;

// ── Motores ──────────────────────────────────────────────────
Motor motor_der(18, 19, 0, 1, 70, 230, 1.4f, 14.4f, 0.0f, 0.02f, 10.0f);
Motor motor_izq(26, 27, 2, 3, 70, 230, 2.4f, 18.0f, 0.0f, 0.02f, 10.0f);

// ── Encoders ─────────────────────────────────────────────────
Encoder enc_der(25, 13, 815, false);
Encoder enc_izq(32, 33, 814, true);

// ── DiffDrive ────────────────────────────────────────────────
DiffDriveConfig dd_cfg = {
    .wheel_radius = 0.0925f,
    .base_width   = 0.355f,
    .dt           = 0.02f
};
DiffDrive robot(motor_der, motor_izq, enc_der, enc_izq, dd_cfg);

// ── Sonares ──────────────────────────────────────────────────
SonarManager sonars(15, 36, 35, 34);

// ── Bumpers ──────────────────────────────────────────────────
const uint8_t BUMPER_PINS[BUMPER_MAX_SENSORS] = {17, 16, 5, 4, 23};
BumperManager bumpers(BUMPER_PINS, 5);

// ── Buzzer ──────────────────────────────────────────────────
BuzzerManager buzzer(BUZZER_PIN);

// ── Batería ──────────────────────────────────────────────────
BatteryManager battery(0.002f, 20.0f);

// ── Display ──────────────────────────────────────────────────
DisplayManager display;

// Estado del sistema para el display - inicia en desconectado
BuzzerPattern display_state = BuzzerPattern::DISCONNECTED;

// ── micro-ROS - Entidades ───────────────────────────────────
// Subscribers
rcl_subscription_t sub_cmdvel;
rcl_subscription_t sub_estop;
geometry_msgs__msg__Twist msg_cmdvel;
std_msgs__msg__Bool       msg_estop_in;
// Publisher
rcl_publisher_t pub_debug_cmd;
rcl_publisher_t pub_odom;
rcl_publisher_t pub_battery;
rcl_publisher_t pub_sonar_l, pub_sonar_c, pub_sonar_r;
rcl_publisher_t pub_bumper[BUMPER_MAX_SENSORS];
// Mensajes
geometry_msgs__msg__Twist msg_debug_cmd;
nav_msgs__msg__Odometry   msg_odom;
sensor_msgs__msg__BatteryState msg_battery;
sensor_msgs__msg__Range   msg_sonar[3];
std_msgs__msg__Bool       msg_bumper[BUMPER_MAX_SENSORS];
// Timers separados por frecuencia
rcl_timer_t timer_odom;     // 50Hz
rcl_timer_t timer_battery;  // 2Hz
rcl_timer_t timer_sensors;  // 5Hz

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// ── Prototipos ───────────────────────────────────────────────
void error_loop();

// ── Macros micro-ROS ─────────────────────────────────────────
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ── Helper: asignar string a campo rosidl sin utilidades externas ──
static void set_string(rosidl_runtime_c__String& field, const char* str)
{
    size_t len = strlen(str);
    // Si ya tiene buffer suficiente, reusar; si no, pedir al allocator
    if (field.capacity <= len) {
        if (field.data) allocator.deallocate(field.data, allocator.state);
        field.data     = (char*)allocator.allocate(len + 1, allocator.state);
        field.capacity = len + 1;
    }
    memcpy(field.data, str, len + 1);
    field.size = len;
}
// ── Helper: timestamp desde sesión micro-ROS sincronizada ─────
static inline builtin_interfaces__msg__Time get_ros_time()
{
    int64_t ns = rmw_uros_epoch_nanos();
    builtin_interfaces__msg__Time t;
    t.sec     = (int32_t)(ns / 1000000000LL);
    t.nanosec = (uint32_t)(ns % 1000000000LL);
    return t;
}

// ── Callback cmd_vel ─────────────────────────────────────────
void cmd_vel_cb(const void* msg_in)
{
    if (estop) return; //ignorar cmd si estop activo
    const geometry_msgs__msg__Twist* msg =
        (const geometry_msgs__msg__Twist*)msg_in;
        
    v = (float)msg->linear.x;
    w = (float)msg->angular.z;
    
    last_cmd_ms = millis(); // resetear watchdog
    
    // Debug — republicar
    msg_debug_cmd.linear.x  = v;
    msg_debug_cmd.angular.z = w;
    
    RCSOFTCHECK(rcl_publish(&pub_debug_cmd, &msg_debug_cmd, NULL));
    
    // Aplicar al robot
    if (fabsf(v) < 0.01f && fabsf(w) < 0.01f) {
        robot.stop();
    } else {
        robot.setCmdVel(v, w);
    }
}
//── Callback e stop ─────────────────────────────────────────
void estop_cb(const void* msg_in)
{
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msg_in;
    estop = msg->data;
    if (estop) {
        robot.stop();
        v = 0; w = 0;
        buzzer.play(BuzzerPattern::DISCONNECTED);
        display_state = BuzzerPattern::DISCONNECTED;
    }
}
// ── Timers publishers ────────────────────────────────────────
// ── Timer odom ───────────────────────────────────────────────
void timer_odom_cb(rcl_timer_t* timer, int64_t)
{
    if (timer == NULL) return;

    builtin_interfaces__msg__Time now = get_ros_time();
    Odometry o = robot.getOdometry();

    msg_odom.header.stamp              = now;
    msg_odom.pose.pose.position.x      = o.x;
    msg_odom.pose.pose.position.y      = o.y;
    msg_odom.twist.twist.linear.x      = o.vx;
    msg_odom.twist.twist.angular.z     = o.wz;
    // Quaternion desde yaw (roll=0, pitch=0)
    msg_odom.pose.pose.orientation.x   = 0.0f;
    msg_odom.pose.pose.orientation.y   = 0.0f;
    msg_odom.pose.pose.orientation.z   = sinf(o.theta / 2.0f);
    msg_odom.pose.pose.orientation.w   = cosf(o.theta / 2.0f);
    // Covarianzas mínimas (diagonal)
    // pose: x, y, yaw  |  twist: vx, wz
    msg_odom.pose.covariance[0]        = 0.01; // x
    msg_odom.pose.covariance[7]        = 0.01; // y
    msg_odom.pose.covariance[35]       = 0.05; // yaw
    msg_odom.twist.covariance[0]       = 0.01; // vx
    msg_odom.twist.covariance[35]      = 0.05; // wz

    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

// ── Timer bateria ─────────────────────────────────────────────
void timer_battery_cb(rcl_timer_t* timer, int64_t)
{
    if (timer == NULL) return;
    
    builtin_interfaces__msg__Time now = get_ros_time();
    
    msg_battery.header.stamp    = now;
    msg_battery.voltage         = battery.getVoltage();
    msg_battery.current         = battery.getCurrent();
    msg_battery.percentage      = battery.getPercent() / 100.0f;
    msg_battery.power_supply_status         = 
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    msg_battery.present         = true;
    RCSOFTCHECK(rcl_publish(&pub_battery, &msg_battery, NULL));
}

// ── Timer sensors ─────────────────────────────────────────────
void timer_sensors_cb(rcl_timer_t* timer, int64_t)
{
    if (timer == NULL) return;

    builtin_interfaces__msg__Time now = get_ros_time();
    // ── Sonares ───────────────────────────────────────────────
    SonarReading sl = sonars.getLeft();
    SonarReading sc = sonars.getCenter();
    SonarReading sr = sonars.getRight();

    msg_sonar[0].header.stamp = now;
    msg_sonar[1].header.stamp = now;
    msg_sonar[2].header.stamp = now;
    msg_sonar[0].range = sl.valid ? sl.distance : msg_sonar[0].max_range;
    msg_sonar[1].range = sc.valid ? sc.distance : msg_sonar[1].max_range;
    msg_sonar[2].range = sr.valid ? sr.distance : msg_sonar[2].max_range;

    RCSOFTCHECK(rcl_publish(&pub_sonar_l, &msg_sonar[0], NULL));
    RCSOFTCHECK(rcl_publish(&pub_sonar_c, &msg_sonar[1], NULL));
    RCSOFTCHECK(rcl_publish(&pub_sonar_r, &msg_sonar[2], NULL));

    // ── Bumpers ───────────────────────────────────────────────
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        msg_bumper[i].data = bumpers.get(i).pressed;
        RCSOFTCHECK(rcl_publish(&pub_bumper[i], &msg_bumper[i], NULL));
    }

    // ── Detección de levantamiento por pares opuestos ────────
    bool b1 = bumpers.get(0).pressed;  // front_left
    bool b2 = bumpers.get(1).pressed;  // front_right
    bool b3 = bumpers.get(2).pressed;  // left
    bool b4 = bumpers.get(3).pressed;  // right
    bool b5 = bumpers.get(4).pressed;  // back

    bool lifted = (b3 && b4)           // agarrado lateralmente
               || (b1 && b5)           // agarrado diagonal izq
               || (b2 && b5);          // agarrado diagonal der

    if (lifted && !estop) {
        estop = true;
        robot.stop();
        v = 0; w = 0;
        buzzer.play(BuzzerPattern::LIFTED);
        display_state = BuzzerPattern::LIFTED;
    }
}

// ── Crear entidades micro-ROS ─────────────────────────────────
bool create_entities()
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        return false;
    if (rclc_node_init_default(&node, "mow_base_node", "", &support) != RCL_RET_OK)
        return false;
    if (rmw_uros_sync_session(1000) != RCL_RET_OK)
        return false;
    
    //  ── Subscribers ──────────────────────────────────────────
    if (rclc_subscription_init_default(&sub_cmdvel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;
    
    if (rclc_subscription_init_default(&sub_estop, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "estop") != RCL_RET_OK) return false;
    //  ── Publishers - RELIABLE (default)───────────────────────
    if (rclc_publisher_init_default(&pub_debug_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/cmd_received") != RCL_RET_OK) return false;
 
    if (rclc_publisher_init_default(&pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered") != RCL_RET_OK) return false;
    
    if (rclc_publisher_init_default(&pub_battery, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery/state") != RCL_RET_OK) return false;
    
    if (rclc_publisher_init_default(&pub_sonar_l, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "sonar/left") != RCL_RET_OK) return false;
    if (rclc_publisher_init_default(&pub_sonar_c, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "sonar/center") != RCL_RET_OK) return false;
    if (rclc_publisher_init_default(&pub_sonar_r, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "sonar/right") != RCL_RET_OK) return false;

    const char* bumper_topics[BUMPER_MAX_SENSORS] = {
        "bumper/B1_front_left", "bumper/B2_front_right",
        "bumper/B3_left", "bumper/B4_right", "bumper/B5_back"
    };
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++) {
        if (rclc_publisher_init_default(&pub_bumper[i], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            bumper_topics[i]) != RCL_RET_OK) return false;
        msg_bumper[i].data = false;
    }
    
    // Inicializar mensajes Range
    const char* sonar_frames[3] = {"sonar_left", "sonar_center", "sonar_right"};
    for (int i = 0; i < 3; i++) {
        set_string(msg_sonar[i].header.frame_id, sonar_frames[i]);
        msg_sonar[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        msg_sonar[i].field_of_view  = 0.2618f;
        msg_sonar[i].min_range      = 0.02f;
        msg_sonar[i].max_range      = 1.0f;
        msg_sonar[i].range          = 1.0f;
    }
    set_string(msg_odom.header.frame_id, "odom");
    set_string(msg_odom.child_frame_id,  "base_link");
    set_string(msg_battery.header.frame_id, "battery");

    // ── Timers ───────────────────────────────────────────────
    // odom: 20ms = 50 Hz
    if (rclc_timer_init_default(&timer_odom, &support,
        RCL_MS_TO_NS(20), timer_odom_cb) != RCL_RET_OK) return false;
    // bateria: 500ms = 2 Hz
    if (rclc_timer_init_default(&timer_battery, &support,
        RCL_MS_TO_NS(500), timer_battery_cb) != RCL_RET_OK) return false;
    // sonares, bumpers: 200ms = 5 Hz
    if (rclc_timer_init_default(&timer_sensors, &support,
        RCL_MS_TO_NS(200), timer_sensors_cb) != RCL_RET_OK) return false;

    // ── Executor — 2 subs + 3 timers  = 5 handles ─────────────
    if (rclc_executor_init(&executor, &support.context, 5, &allocator)
        != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(&executor, &sub_cmdvel,
        &msg_cmdvel, &cmd_vel_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &sub_estop,
        &msg_estop_in, &estop_cb, ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &timer_odom)   != RCL_RET_OK) return false;
    if (rclc_executor_add_timer(&executor, &timer_battery) != RCL_RET_OK) return false;
    if (rclc_executor_add_timer(&executor, &timer_sensors) != RCL_RET_OK) return false;
    return true;
}

// ── Destruir entidades micro-ROS ─────────────────────────────
void destroy_entities()
{
    rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    rcl_publisher_fini(&pub_debug_cmd, &node);
    rcl_publisher_fini(&pub_odom,      &node);
    rcl_publisher_fini(&pub_battery,   &node);
    rcl_publisher_fini(&pub_sonar_l,   &node);
    rcl_publisher_fini(&pub_sonar_c,   &node);
    rcl_publisher_fini(&pub_sonar_r,   &node);
    for (int i = 0; i < BUMPER_MAX_SENSORS; i++)
        rcl_publisher_fini(&pub_bumper[i], &node);

    rcl_subscription_fini(&sub_cmdvel, &node);
    rcl_subscription_fini(&sub_estop,  &node);
    rcl_timer_fini(&timer_odom);
    rcl_timer_fini(&timer_battery);
    rcl_timer_fini(&timer_sensors);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ── Error loop ───────────────────────────────────────────────
void error_loop()
{
    while (true) {
        buzzer.update();
        delay(100);
    }
}

// ─────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(921600);
    buzzer.begin();
    robot.begin();
    sonars.begin();
    bumpers.begin();
    battery.begin();
    display.begin();
    
    set_microros_serial_transports(Serial);

    agent_state = AgentState::WAITING_AGENT;
    last_ctrl   = millis();
    ctrl_hz_ts  = millis();
}

// ─────────────────────────────────────────────────────────────
void loop()
{
    unsigned long now = millis();
    // ── FSM micro-ROS ────────────────────────────────────────
    switch (agent_state) {
        // Ping solo cada 200ms — no bloquear el control loop
        case AgentState::WAITING_AGENT:
            // Parpadear buzzer lento esperando conexión
		    if (now - last_ping_ms >= PING_INTERVAL_MS) {
		last_ping_ms = now;
		if (now % 1000 < 200) buzzer.play(BuzzerPattern::DISCONNECTED);
		if (rmw_uros_ping_agent(50, 1) == RCL_RET_OK) {
		    if (create_entities()) {
		        agent_state = AgentState::AGENT_CONNECTED;
		        last_cmd_ms = now;
		        ping_ok     = true;
		        display_state = BuzzerPattern::CONNECTED;
                        buzzer.play(BuzzerPattern::CONNECTED);
		    }
		}
	    }
            break;

        case AgentState::AGENT_CONNECTED:
	    if (now - last_ping_ms >= PING_INTERVAL_MS) {
		last_ping_ms = now;
		ping_ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	    }
	    if (!ping_ok) {
		destroy_entities();
		robot.stop();
		v = 0; w = 0; estop = false;
		agent_state = AgentState::AGENT_DISCONNECTED;
		display_state = BuzzerPattern::DISCONNECTED;
                buzzer.play(BuzzerPattern::DISCONNECTED);
		break;
	    }

            // Watchdog cmd_vel
            if (!estop && (now - last_cmd_ms) > CMD_WATCHDOG_MS) {
                robot.stop();
                v = 0; w = 0;
            }

            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
            break;

        case AgentState::AGENT_DISCONNECTED:
            // Intentar reconectar
            if (now - last_ping_ms >= PING_INTERVAL_MS) {
		last_ping_ms = now;
		if (rmw_uros_ping_agent(50, 1) == RCL_RET_OK) {
		    if (create_entities()) {
		        agent_state = AgentState::AGENT_CONNECTED;
		        last_cmd_ms = now;
		        ping_ok     = true;
		        display_state = BuzzerPattern::CONNECTED;
                        buzzer.play(BuzzerPattern::CONNECTED);
		    }
		}
	    }
            break;
    }

    // ── Control loop 50 Hz ───────────────────────────────────
    if (now - last_ctrl >= PERIOD_MS) {
        last_ctrl = now;
        // Medir frecuencia real del control loop
        ctrl_count++;
        if (now - ctrl_hz_ts >= 1000) {
            ctrl_hz    = (float)ctrl_count;
            ctrl_count = 0;
            ctrl_hz_ts = now;
        }
        if (!estop) robot.update();
        sonars.update();
        bumpers.update();
        buzzer.update();
        battery.update();              
        display.update(battery, display_state);
    }
}
