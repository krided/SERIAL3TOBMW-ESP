#include "dashboard.h"
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
unsigned long lastUpdate = 0;

// Initialize the Bluetooth dashboard
void Dashboard::begin() {
    SerialBT.begin(BT_NAME);
    Serial.println("Dashboard BT started: " + String(BT_NAME));
}

// Update function to be called in the main loop
void Dashboard::update() {
    if (millis() - lastUpdate >= DASH_UPDATE_MS) {
        lastUpdate = millis();
        sendBT();
    }
}

// Set a value to be sent to the dashboard
void Dashboard::setValue(const char* key, float value) {
    for (int i = 0; i < MAX_DASH_VALUES; i++) {
        if (strcmp(DASH_KEYS[i], key) == 0) {
            DASH_VALUES[i] = value;
            break;
        }
    }
}

// Send all values via Bluetooth
void Dashboard::sendBT() {
    // send all values as key=value;key=value;...
    String payload = "";
    for (int i = 0; i < MAX_DASH_VALUES; i++) {
        if (DASH_KEYS[i] == nullptr || strlen(DASH_KEYS[i]) == 0) continue;
        payload += String(DASH_KEYS[i]) + "=" + String(DASH_VALUES[i],2) + ";";
    }
    SerialBT.println(payload);
}
