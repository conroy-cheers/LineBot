#include "Arduino.h"
#include <stdio.h>

#include "PID/PID_v1.h"
#include "L298N/L298N.h"
#include "Filters/Filters.h"

#include "rgb.h"
#include "TCS230.h"

#define LEFT_S0 12
#define LEFT_S1 A1
#define LEFT_S2 9
#define LEFT_S3 8
#define LEFT_OUT 10

#define RIGHT_S0 7
#define RIGHT_S1 A0
#define RIGHT_S2 3
#define RIGHT_S3 2
#define RIGHT_OUT 5

#define MOT_LEFT_EN 11
#define MOT_LEFT_IN1 A4
#define MOT_LEFT_IN2 A3

#define MOT_RIGHT_EN 6
#define MOT_RIGHT_IN1 4
#define MOT_RIGHT_IN2 13

#define MOT_MAX_SPEED_LEFT 86
#define MOT_MAX_SPEED_RGHT 92
//#define MOT_MAX_SPEED_LEFT 140
//#define MOT_MAX_SPEED_RGHT 150

const TCS230::PinMapping leftPins{LEFT_S0, LEFT_S1, LEFT_S2, LEFT_S3, LEFT_OUT};
const TCS230::PinMapping rightPins{RIGHT_S0, RIGHT_S1, RIGHT_S2, RIGHT_S3, RIGHT_OUT};

//#define PID_KP 0.48
//#define PID_KI 1.8
//#define PID_KD 0.08
#define PID_KP 0.6
#define PID_KI 0.1
#define PID_KD 0.06

//Define Variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

// Specify the links and initial tuning parameters
PID myPID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, P_ON_E, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

L298N leftMotor(MOT_LEFT_EN, MOT_LEFT_IN1, MOT_LEFT_IN2);
L298N rightMotor(MOT_RIGHT_EN, MOT_RIGHT_IN1, MOT_RIGHT_IN2);

FilterOnePole leftBrightnessFilter(LOWPASS, 200.0);  // Hz
FilterOnePole rghtBrightnessFilter(LOWPASS, 200.0);  // Hz

void setup() {
    pidInput = 0.0f;
    pidSetpoint = 0.0f;

    Serial.begin(115200);

    TCS230::sensorSetup(leftPins);
    TCS230::sensorSetup(rightPins);

    myPID.SetSampleTime(10);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-1.0f, 1.0f);

    myPID.Compute();
}

uint8_t lineBrightnessScaled(uint8_t const &brightness) {
    return map(brightness, 180, 255, 0, 255);
}

/**
 * @param speed Speed to set. -1 for full reverse, +1 for full forward.
 */
void setMotorSpeed(L298N &motor, float speed, uint8_t scale) {
    uint8_t targetSpeed = (uint8_t) abs(speed) * (uint8_t) scale;
    motor.setSpeed(targetSpeed);
    if (speed > 0.0f) {
        motor.forward();
    } else {
        motor.backward();
    }
}

/**
 * target should be between -1 and +1.
 * -1 corresponds to full right turn (on spot),
 * +1 corresponds to full left turn (on spot).
 * 0 corresponds to full speed ahead.
 * linear mapping between these.
 */
void setMotorSpeeds(double &target) {
    const float ohShit = 1.0f;
    target *= 1.6;

    float hecticScaling;
    float aTarget = abs(target);
    if (aTarget > 0.1) {
        hecticScaling = 1.0 - 0.8 * (aTarget - 0.1);
    } else {
        hecticScaling = 1.0;
    }

    // turn more sharply if required
    if (aTarget > 0.8) {
        target *= 2;
    }

    auto leftMax = (uint8_t) (MOT_MAX_SPEED_LEFT * hecticScaling);
    auto rghtMax = (uint8_t) (MOT_MAX_SPEED_RGHT * hecticScaling);

    if (target < 0) {
        setMotorSpeed(leftMotor, 1.0f, leftMax);
        setMotorSpeed(rightMotor, (ohShit * target + 1.0f), rghtMax);
    } else {
        setMotorSpeed(rightMotor, 1.0f, rghtMax);
        setMotorSpeed(leftMotor, (-1 * ohShit * target + 1.0f), leftMax);
    }
}

void loop() {
    // Read colours from left and right sensors
    rgb::RGB leftReading = TCS230::readSensor(leftPins);
    rgb::RGB rightReading = TCS230::readSensor(rightPins);
//    char outStr[20];
//    sprintf(outStr, "LEFT %d %d %d", leftReading.red, leftReading.green, leftReading.blue);
//    Serial.println(outStr);
//
//    sprintf(outStr, "RGHT %d %d %d", rightReading.red, rightReading.green, rightReading.blue);
//    Serial.println(outStr);

    // Get left and right brightness values
    uint8_t leftBrightness = (uint8_t) map(lineBrightnessScaled(rgb::getBrightness(leftReading)), 0, 255, 0, 225);
    uint8_t rightBrightness = lineBrightnessScaled(rgb::getBrightness(rightReading));

    leftBrightnessFilter.input(leftBrightness);
    rghtBrightnessFilter.input(rightBrightness);

    float lOutNew = leftBrightnessFilter.output() / 255;
    float rOutNew = rghtBrightnessFilter.output() / 255;
    float lOutOld = leftBrightnessFilter.Ylast;
    float rOutOld = rghtBrightnessFilter.Ylast;

    float dLeft = lOutNew - lOutOld;
    float dRight = rOutNew - rOutOld;

    // Scale position to between -1 and +1, where -1 is line on left, +1 is line on right, 0 is line in centre
    if (dLeft > 0.03 && rOutNew > 0.99) {
        // far left (probably)
    } else if (dRight > 0.03 && lOutNew > 0.99) {
        // far right (probably)
    } else {
        pidInput = (lOutNew) - (rOutNew);
    }

    // run PID
    myPID.Compute();

    // make motors spinny
    setMotorSpeeds(pidOutput);

//    char outputStr[20];
//    sprintf(outputStr, "%d %d", leftBrightness, rightBrightness);
//    Serial.println(outputStr);

    // Print output for now
    char outputStr[20];
    sprintf(outputStr, "%ld %ld", (long) (pidInput * 100), (long) (pidOutput * 100));
    Serial.println(outputStr);
}
