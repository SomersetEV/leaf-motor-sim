#include "status_led.h"

#include <Arduino.h>

#include "config.h"
#include "pins.h"

void status_led_update(const SimSnapshot &s, bool can_bus_off_recent) {
    static uint32_t last = 0xFFFFFFFF;
    uint8_t r = 0, g = 0, b = 0;
    if (can_bus_off_recent || s.stats.window_bad()) {
        r = LED_LEVEL;
    } else if (s.profile_state == loads::ProfileState::Running) {
        r = LED_LEVEL;
        g = LED_LEVEL;
    } else if (s.cmd_fresh) {
        g = LED_LEVEL;
    } else {
        b = LED_LEVEL;
    }
    uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (rgb == last) return;  // only write on change
    last = rgb;
    rgbLedWrite(WS2812_PIN, r, g, b);
}
