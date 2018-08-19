#ifndef LINEBOT_TCS230_H
#define LINEBOT_TCS230_H

#include <Arduino.h>
#include <stdio.h>
#include "rgb.h"

namespace TCS230 {
    typedef struct {
        uint8_t s0;
        uint8_t s1;
        uint8_t s2;
        uint8_t s3;
        uint8_t out;
    } PinMapping;

    typedef struct {
        uint8_t s2;
        uint8_t s3;
    } sensorConfig;

    const sensorConfig sensors[3] = {
            sensorConfig{LOW, LOW},
            sensorConfig{HIGH, HIGH},
            sensorConfig{LOW, HIGH},
    };

    typedef struct {
        uint8_t minBrightness;
        uint8_t maxBrightness;
    } colourMap;

    const rgb::RGB red{0, 0, 0};
    const rgb::RGB green{0, 0, 0};
    const rgb::RGB blue{0, 0, 0};

    const colourMap colourMappings[3] = {
            colourMap{130, 22},
            colourMap{160, 23},
            colourMap{125, 18}
    };

    uint8_t readColour(PinMapping const &pins, colourMap const &cMap, sensorConfig const &config) {
        digitalWrite(pins.s2, config.s2);
        digitalWrite(pins.s3, config.s3);
        uint8_t frequency = pulseIn(pins.out, LOW, 1000);

//        char niceMeme[16];
//        sprintf(niceMeme, "raw %d", frequency);
//        Serial.println(niceMeme);

//        return frequency;

        if (frequency < cMap.maxBrightness) frequency = cMap.maxBrightness;
        if (frequency > cMap.minBrightness) frequency = cMap.minBrightness;

        return map(frequency, cMap.minBrightness, cMap.maxBrightness, 0, 255);
    }

    void sensorSetup(PinMapping const &pins) {
        // setup pins as outputs
        pinMode(pins.s0, OUTPUT);
        pinMode(pins.s1, OUTPUT);
        pinMode(pins.s2, OUTPUT);
        pinMode(pins.s3, OUTPUT);
        pinMode(pins.out, INPUT);

        // Specify 1:5 PWM divider
        digitalWrite(pins.s0, HIGH);
        digitalWrite(pins.s1, LOW);
    }

    rgb::RGB readSensor(PinMapping const &pins) {
        rgb::RGB rgbOut = rgb::RGB{0, 0, 0};
        rgbOut.red = readColour(pins, colourMappings[0], sensors[0]);
        rgbOut.green = readColour(pins, colourMappings[1], sensors[1]);
        rgbOut.blue = readColour(pins, colourMappings[2], sensors[2]);

        return rgbOut;
    }
}

#endif //LINEBOT_TCS230_H
