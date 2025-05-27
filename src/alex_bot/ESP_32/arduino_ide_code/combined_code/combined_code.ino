#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// --- Wi-Fi Credentials & Agent Info ---
char* ssid       = "Alexandre_iphone";
char* password   = "hehexdlmao";
char* agent_ip   = "172.20.10.7";
int   agent_port = 8888;

// --- ROS 2 Objects ---
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;
rcl_publisher_t    publisher;
rcl_subscription_t subscriber;
rcl_timer_t        timer;
rclc_executor_t    executor;

// --- Messages ---
std_msgs__msg__Int32 pub_msg;
std_msgs__msg__Int32 sub_msg;

// --- LED Pin ---
#define LED_PIN 13

// --- Error Handling Macros ---
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; /* ignore errors */ }

void error_loop() {
  // Blink LED rapidly on any hard error
  pinMode(LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Subscription callback: toggle LED according to received data
void subscription_callback(const void* msgin) {
  const std_msgs__msg__Int32* m = (const std_msgs__msg__Int32*)msgin;
  digitalWrite(LED_PIN, (m->data != 0) ? HIGH : LOW);
}

// Timer callback: publish & increment counter
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != nullptr) {
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, nullptr));
    pub_msg.data++;
  }
}

void setup() {
  // Initialize serial & LED
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(1000);

  // 1) Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected, IP=");
  Serial.println(WiFi.localIP());

  // 2) Configure micro-ROS transport over Wi-Fi
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  // 3) Initialize micro-ROS support & node
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_pubsub_node", "", &support));

  // 4) Create publisher on "chatter"
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "chatter"));

  // Initialize the outgoing message
  pub_msg.data = 0;

  // 5) Create subscription on "chatter"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "chatter"));

  // 6) Create a 500ms timer for publishing
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(500),
    timer_callback));

  // 7) Create executor, add both subscription and timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &sub_msg,
    &subscription_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // Handle incoming messages & timers
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);

  // Optional: auto-reconnect on Wi-Fi drop
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost; reconnecting...");
    WiFi.reconnect();
    delay(500);
  }
}
