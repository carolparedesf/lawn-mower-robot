// Minimal micro-ROS test: publishes std_msgs/Int32 counter on /esp32_min/counter
// every 100 ms over serial @ 921600. No sensors, no motors, no robot code.
// Used to rule out ESP32 hardware / serial link problems.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define LED_PIN 2
#define PUBLISH_PERIOD_MS 100
#define PING_INTERVAL_MS 200

enum class State { WAITING_AGENT, CONNECTED, DISCONNECTED };
static State state = State::WAITING_AGENT;

static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

static unsigned long last_ping_ms = 0;

static void timer_cb(rcl_timer_t *t, int64_t) {
  if (t == NULL) return;
  rcl_publish(&publisher, &msg, NULL);
  msg.data++;
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_min", "", &support) != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(
          &publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "esp32_min/counter") != RCL_RET_OK) return false;
  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
                              timer_cb) != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  msg.data = 0;
  delay(2000);
}

void loop() {
  switch (state) {
    case State::WAITING_AGENT:
      if (millis() - last_ping_ms >= PING_INTERVAL_MS) {
        last_ping_ms = millis();
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          if (create_entities()) {
            state = State::CONNECTED;
          } else {
            destroy_entities();
          }
        }
      }
      digitalWrite(LED_PIN, (millis() / 500) % 2);  // slow blink: waiting
      break;

    case State::CONNECTED:
      if (millis() - last_ping_ms >= 1000) {
        last_ping_ms = millis();
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
          state = State::DISCONNECTED;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      digitalWrite(LED_PIN, HIGH);  // solid: connected
      break;

    case State::DISCONNECTED:
      destroy_entities();
      state = State::WAITING_AGENT;
      break;
  }
}
