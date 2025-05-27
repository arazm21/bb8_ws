#include <DRV8871.h>

#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8_multi_array.h>
#include "DRV8871.h"

char* ssid = "Alexandre_iphone";
char* password = "hehexdlmao";
char* agent_ip = "172.20.10.7";
int agent_port = 8888;

rcl_subscription_t subscriber;
std_msgs__msg__Int8MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;



#define LED_PIN 13
#define MOTOR_IN0  14
#define MOTOR_IN1  12
#define PWM_CHANNEL0  0
#define PWM_CHANNEL1  1

DRV8871 motor(MOTOR_IN0, MOTOR_IN1,  PWM_CHANNEL0, PWM_CHANNEL1);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
  Serial.println(rcl_get_error_string().str); \
  rcl_reset_error(); \
  error_loop(); \
}}

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int8MultiArray *msg = (const std_msgs__msg__Int8MultiArray *)msgin;

  if (msg->data.size < 2) {
    Serial.println("Invalid message size");
    return;
  }

  int8_t direction = msg->data.data[0];
  int8_t speed = msg->data.data[1];

  Serial.print("Direction: ");
  Serial.print(direction);
  Serial.print(" Speed: ");
  Serial.println(speed);
  if (direction ==-1) direction = 0;
  if (direction == 1) {
    digitalWrite(LED_PIN, HIGH);
  } else if (direction == 0) {
    digitalWrite(LED_PIN, LOW);
  }
  motor.setMotor(direction, speed);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  motor.init();
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP());

  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_wifi_node", "", &support));

  // Allocate memory for message data
  msg.data.capacity = 2;
  msg.data.size = 2;
  msg.data.data = (int8_t*)malloc(msg.data.capacity * sizeof(int8_t));
  if (msg.data.data == NULL) {
    Serial.println("Memory allocation failed");
    error_loop();
  }

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
    "motor_direction"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected! Reconnecting...");
    WiFi.reconnect();
    delay(2000);
  }

  rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (rc != RCL_RET_OK) {
    Serial.println("Executor spin failed:");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
  }

  delay(10);
}
