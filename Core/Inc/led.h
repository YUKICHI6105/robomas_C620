#pragma once
#include <stdint.h>

// How long the LED stays on after led_on() is called.
#define LED_INTERVAL 300 // micro sec

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum _led
    {
        can,
        green,
        yellow,
        red
    } led;

    void led_on(led name);

    // Turns off when there is an LED that stays on for a long time.
    void led_process(void);

#ifdef __cplusplus
}
#endif
