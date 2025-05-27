#include "DRV8871.h"

// Define pins and channels
#define MOTOR_IN0  14
#define MOTOR_IN1  12
#define PWM_CHANNEL0  0
#define PWM_CHANNEL1  1

// Create motor controller object
DRV8871 motor(MOTOR_IN0, MOTOR_IN1, PWM_CHANNEL0, PWM_CHANNEL1);

void setup() {
    Serial.begin(9600);
    Serial.println("DRV8871 Motor Test Begin");

    // Initialize the motor driver
    motor.init();
}

void loop() {
    Serial.println("Motor running FORWARD at 75%");
    motor.setMotor(MOTOR_FORWARD, 75);
    delay(3000);

    Serial.println("Motor STOP");
    motor.stopMotor();
    delay(1000);

    Serial.println("Motor running REVERSE at 50%");
    motor.setMotor(MOTOR_REVERSE, 50);
    delay(3000);

    Serial.println("Motor STOP");
    motor.stopMotor();
    delay(2000);
}
