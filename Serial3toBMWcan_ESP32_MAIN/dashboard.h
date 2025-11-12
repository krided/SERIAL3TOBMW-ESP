#pragma once
#include <Arduino.h>
#include "config.h"

class Dashboard {
public:
    void begin();
    void update();
    void setValue(const char* key, float value);

private:
    void sendBT();
};
