#include "DRV8871.h"

DRV8871::DRV8871(uint8_t in1Pin, uint8_t in2Pin,
                 uint8_t pwmChannel1, uint8_t pwmChannel2,
                 uint32_t freq, uint8_t res)
  : _in1Pin(in1Pin), _in2Pin(in2Pin),
    _pwmChannel1(pwmChannel1), _pwmChannel2(pwmChannel2),
    _frequency(freq), _resolution(res)
{
    _maxDutyCycle = (1UL << _resolution) - 1;
    _minDutyCycle = 0.45 * _maxDutyCycle; // at 12 V supply
    _minSpeedThreshold = 5; // percent
}

void DRV8871::init() {
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);

    ledcAttachChannel(_in1Pin, _frequency, _resolution, _pwmChannel1);
    ledcAttachChannel(_in2Pin, _frequency, _resolution, _pwmChannel2);
}

void DRV8871::setMotor(uint8_t direction, uint8_t speedPercent) {
    speedPercent = constrain(speedPercent, 0, 100);

    // Stay stopped if speed is below threshold
    if (speedPercent < _minSpeedThreshold) {
        stopMotor();
        return;
    }

    uint32_t dutyCycle = map(speedPercent, 0, 100, _minDutyCycle, _maxDutyCycle);

    if (direction == MOTOR_FORWARD) {
        ledcWrite(_in1Pin, dutyCycle);
        ledcWrite(_in2Pin, 0);
    }
    else if (direction == MOTOR_REVERSE) {
        ledcWrite(_in2Pin, dutyCycle);
        ledcWrite(_in1Pin, 0);
    }
}

void DRV8871::stopMotor() {
    ledcWrite(_in1Pin, 0);
    ledcWrite(_in2Pin, 0);
}
