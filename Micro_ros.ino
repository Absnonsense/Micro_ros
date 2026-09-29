#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#define POT 34
#define PWM 25
#define ENA 27
#define IN1 12
#define IN2 14
#define FREQ 5000
#define BITS 12

rclc_support_t sup;
rcl_allocator_t alloc;
rcl_node_t nd;
rcl_publisher_t pub;
std_msgs__msg__Int32 val;

bool ok = false;
unsigned long t1 = 0;
unsigned long t2 = 0;

void setPwm(uint32_t d) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM, d);
#else
  ledcWrite(0, d);
#endif
}

bool startRos() {
  alloc = rcl_get_default_allocator();
  if (rclc_support_init(&sup, 0, NULL, &alloc) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&nd, "esp32_motor_node", "", &sup) != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pub, &nd, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pot_raw") != RCL_RET_OK) return false;
  return true;
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogReadResolution(12);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWM, FREQ, BITS);
#else
  ledcSetup(0, FREQ, BITS);
  ledcAttachPin(PWM, 0);
#endif
  set_microros_transports();
}

void loop() {
  int p = analogRead(POT);
  setPwm(p);
  unsigned long now = millis();
  if (!ok && now - t1 > 2000) {
    t1 = now;
    if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
      ok = startRos();
    }
  }
  if (ok && now - t2 >= 20) {
    t2 = now;
    val.data = p;
    rcl_publish(&pub, &val, NULL);
  }
}