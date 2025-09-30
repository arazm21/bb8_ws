#ifndef DRV8871_H
#define DRV8871_H

#include <Arduino.h>
#include <esp32-hal-ledc.h>

#define MOTOR_FORWARD 0
#define MOTOR_REVERSE 1

class DRV8871 {
public:   
    // Constructor: Accepts the two motor GPIO PWM pins, two PWM channels, PWM frequency (Hz), and resolution (in bits)
    DRV8871(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmChannel1, uint8_t pwmChannel2, uint32_t freq = 5000, uint8_t res = 10);

    // Initializes PWM and pin configurations. Must be called in setup().
    void init();

    // Sets motor direction (FORWARD or REVERSE) with speed as a percentage (0 - 100)
    void setMotor(uint8_t direction, uint8_t speedPercent);

    // Stops the motor by turning off both PWM channels.
    void stopMotor();

private:
    uint8_t _in1Pin, _in2Pin;
    uint8_t _pwmChannel1, _pwmChannel2;
    uint32_t _frequency;
    uint8_t _resolution;
    uint32_t _maxDutyCycle;
    uint32_t _minDutyCycle;
    uint8_t _minSpeedThreshold;  // <-- Add this declaration

};

#endif // DRV8871_H
