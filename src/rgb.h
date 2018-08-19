#ifndef LINEBOT_RGB_H
#define LINEBOT_RGB_H

namespace rgb {
    typedef struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } RGB;

    uint8_t getBrightness(RGB &colour) {
        uint16_t sum = colour.red + colour.green + colour.blue;
        return (uint8_t) (sum / 3);
    }
}

#endif //LINEBOT_RGB_H
