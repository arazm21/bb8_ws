
// #include <micro_ros_arduino.h>
// #include <WiFi.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32_multi_array.h>
// #include <ESP32Servo.h>
// #include <wit_c_sdk.h>

// // ---------------- Wi-Fi ----------------
// char* ssid = "Alexandre_iphone";
// char* password = "hehexdlmao";
// char* agent_ip = "172.20.10.8";
// int agent_port = 8888;

// // ---------------- ROS ----------------
// rcl_publisher_t publisher;
// rcl_timer_t timer;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// std_msgs__msg__Float32MultiArray sensor_msg;

// // ---------------- WT901 ----------------
// #define ACC_UPDATE   0x01
// #define ANGLE_UPDATE 0x04
// #define RX1 16
// #define TX1 17

// static volatile char s_cDataUpdate = 0;
// float fAcc[3], fAngle[3];
// const uint32_t c_uiBaud[8] = {0,4800,9600,19200,38400,57600,115200,230400};

// // ---------------- Servo ----------------
// Servo myServo;
// const int servoPin = 13;  // Use any PWM-capable GPIO

// // ---------------- Error Handling ----------------
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// // ---------------- Helpers ----------------
// void servoWriteAngle(float angle) {
//   angle = constrain(angle, 60, 120);
//   myServo.write(angle);
// }

// // ---------------- Data freshness ----------------
// volatile bool newSensorData = false;
// unsigned long lastPrintMs = 0;
// const unsigned long PRINT_INTERVAL_MS = 200;

// // ---------------- Timer Callback ----------------
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);

//   if (!newSensorData) {
//     return; // nothing fresh yet
//   }

//   // Servo stabilization: use roll
//   float roll = fAngle[0];
//   float servo_angle = 90 - roll;   // invert roll so servo corrects tilt
//   servo_angle = 180 - servo_angle;   // invert roll so servo corrects tilt

//   servoWriteAngle(servo_angle);

//   // Publish ROS message
//   sensor_msg.data.data[0] = fAcc[0];
//   sensor_msg.data.data[1] = fAcc[1];
//   sensor_msg.data.data[2] = fAcc[2];
//   sensor_msg.data.data[3] = fAngle[0];
//   sensor_msg.data.data[4] = fAngle[1];
//   sensor_msg.data.data[5] = fAngle[2];

//   RCSOFTCHECK(rcl_publish(&publisher, &sensor_msg, NULL));

//   // Throttle serial debug
//   unsigned long now = millis();
//   if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
//     Serial.printf("Pub Acc[%.2f %.2f %.2f] Ang[%.2f %.2f %.2f] Servo=%.1f°\n",
//                   fAcc[0], fAcc[1], fAcc[2],
//                   fAngle[0], fAngle[1], fAngle[2],
//                   servo_angle);
//     lastPrintMs = now;
//   }

//   // consume the data
//   newSensorData = false;
// }

// // ---------------- FreeRTOS Tasks ----------------
// void imuTask(void *pvParameters) {
//   (void) pvParameters;
//   for (;;) {
//     if (Serial1.available()) {
//       WitSerialDataIn(Serial1.read());
//     }

//     if(s_cDataUpdate) {
//       for(int i = 0; i < 3; i++) {
//         fAcc[i]   = sReg[AX+i] / 32768.0f * 16.0f;
//         fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
//       }
//       s_cDataUpdate = 0;
//       newSensorData = true;
//     }

//     vTaskDelay(1); // yield to other tasks
//   }
// }

// void rosTask(void *pvParameters) {
//   (void) pvParameters;
//   for (;;) {
//     RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
//     vTaskDelay(1);
//   }
// }

// // ---------------- Setup ----------------
// void setup() {
//   Serial.begin(115200);

//   // Servo init
//   myServo.attach(servoPin, 500, 2500);
//   myServo.write(90); // center

//   // Wi-Fi
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWi-Fi connected: " + WiFi.localIP().toString());

//   // micro-ROS transport
//   set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

//   delay(1000);

//   // micro-ROS init
//   allocator = rcl_get_default_allocator();
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//   RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

//   RCCHECK(rclc_publisher_init_best_effort(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
//     "imu_data"));

//   sensor_msg.data.capacity = 6;
//   sensor_msg.data.size = 6;
//   sensor_msg.data.data = (float*)malloc(6 * sizeof(float));

//   // Timer @ 50 Hz
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(20),
//     timer_callback));

//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   // WT901 init
//   WitInit(WIT_PROTOCOL_NORMAL, 0x50);
//   WitSerialWriteRegister([](uint8_t *p_data, uint32_t uiSize){ Serial1.write(p_data, uiSize); });
//   WitRegisterCallBack([](uint32_t uiReg, uint32_t uiRegNum){
//     for(int i=0;i<uiRegNum;i++){
//       if(uiReg==AZ) s_cDataUpdate|=ACC_UPDATE;
//       if(uiReg==Yaw) s_cDataUpdate|=ANGLE_UPDATE;
//       uiReg++;
//     }
//   });
//   WitDelayMsRegister([](uint16_t ms){ delay(ms); });

//   // Auto scan sensor baud
//   for(int i=0;i<sizeof(c_uiBaud)/sizeof(c_uiBaud[0]);i++){
//     Serial1.begin(c_uiBaud[i], SERIAL_8N1, RX1, TX1);
//     delay(200);
//     WitReadReg(AX, 3);
//     delay(200);
//     if(Serial1.available()){ 
//       Serial.println("Sensor detected."); 
//       break; 
//     }
//   }

//   Serial.println("Ready!");

//   // Create FreeRTOS tasks
//   xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 1, NULL, 1);
//   xTaskCreatePinnedToCore(rosTask, "ROS Task", 8192, NULL, 1, NULL, 0);
// }

// // ---------------- Loop ----------------
// void loop() {
//   // Nothing here, everything runs in tasks
// }




#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <ESP32Servo.h>
#include <wit_c_sdk.h>

// ---------------- Wi-Fi ----------------
char* ssid = "Alexandre_iphone";
char* password = "hehexdlmao";
char* agent_ip = "172.20.10.8";
int agent_port = 8888;

// ---------------- ROS ----------------
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Float32MultiArray sensor_msg;

// ---------------- WT901 ----------------
#define ACC_UPDATE   0x01
#define ANGLE_UPDATE 0x04
#define RX1 16
#define TX1 17

static volatile char s_cDataUpdate = 0;
float fAcc[3], fAngle[3];
const uint32_t c_uiBaud[8] = {0,4800,9600,19200,38400,57600,115200,230400};

// ---------------- Servo ----------------
Servo myServo;
const int servoPin = 13;  // Use any PWM-capable GPIO

// ---------------- Error Handling ----------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

// ---------------- Helpers ----------------
void servoWriteAngle(float angle) {
  angle = constrain(angle, 60, 120);
  myServo.write(angle);
}

// ---------------- Data freshness ----------------
volatile bool newSensorData = false;
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 200;

// ---------------- Timer Callback ----------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  Serial.printf("callback!!!! - \n");
  RCLC_UNUSED(last_call_time);

  if (!newSensorData) {
    return; // nothing fresh yet
  }

  // Servo stabilization: use roll
  float roll = fAngle[0];
  // float servo_angle = 90 - roll;   // invert roll so servo corrects tilt
  float servo_angle = (90 - roll);   // invert roll so servo corrects tilt
  servo_angle = 180 - servo_angle;
  servoWriteAngle(servo_angle);

  // Publish ROS message
  sensor_msg.data.data[0] = fAcc[0];
  sensor_msg.data.data[1] = fAcc[1];
  sensor_msg.data.data[2] = fAcc[2];
  sensor_msg.data.data[3] = fAngle[0];
  sensor_msg.data.data[4] = fAngle[1];
  sensor_msg.data.data[5] = fAngle[2];

  RCSOFTCHECK(rcl_publish(&publisher, &sensor_msg, NULL));

  // Throttle serial debug
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    Serial.printf("Pub Acc[%.2f %.2f %.2f] Ang[%.2f %.2f %.2f] Servo=%.1f°\n",
                  fAcc[0], fAcc[1], fAcc[2],
                  fAngle[0], fAngle[1], fAngle[2],
                  servo_angle);
    lastPrintMs = now;
  }

  // consume the data
  newSensorData = false;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  // Servo init
  myServo.attach(servoPin, 500, 2500);  // with min/max pulse widths
  myServo.write(90);                    // center 

  // Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected: " + WiFi.localIP().toString());

  // micro-ROS transport
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  delay(1000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "imu_data"));

  sensor_msg.data.capacity = 6;
  sensor_msg.data.size = 6;
  sensor_msg.data.data = (float*)malloc(6 * sizeof(float));

  // Timer @ 50 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // WT901 init
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister([](uint8_t *p_data, uint32_t uiSize){ Serial1.write(p_data, uiSize); });
  WitRegisterCallBack([](uint32_t uiReg, uint32_t uiRegNum){
    for(int i=0;i<uiRegNum;i++){
      if(uiReg==AZ) s_cDataUpdate|=ACC_UPDATE;
      if(uiReg==Yaw) s_cDataUpdate|=ANGLE_UPDATE;
      uiReg++;
    }
  });
  WitDelayMsRegister([](uint16_t ms){ delay(ms); });

  // Auto scan sensor baud
  for(int i=0;i<sizeof(c_uiBaud)/sizeof(c_uiBaud[0]);i++){
    Serial1.begin(c_uiBaud[i], SERIAL_8N1, RX1, TX1);
    delay(200);
    WitReadReg(AX, 3);
    delay(200);
    if(Serial1.available()){ 
      Serial.println("Sensor detected."); 
      break; 
    }
  }

  Serial.println("Ready!");
}

// ---------------- Loop ----------------
void loop() {
  if (Serial1.available()) {
    WitSerialDataIn(Serial1.read());
  }

  if(s_cDataUpdate) {
    for(int i = 0; i < 3; i++) {
      fAcc[i]   = sReg[AX+i] / 32768.0f * 16.0f;
      fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
    }
    s_cDataUpdate = 0;
    newSensorData = true;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  // delay(1); // yield
}
