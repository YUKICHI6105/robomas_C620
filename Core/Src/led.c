#include "main.h"
#include "led.h"

struct led_status
{
    GPIO_TypeDef *gpio;
    uint16_t pin;
    uint32_t before_tick;
    uint8_t is_high;
};

#define LED_LIST_SIZE 4
struct led_status led_list[] =
    {
        {LED_CAN_GPIO_Port, LED_CAN_Pin, 0, 0},
        {LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0, 0},
        {LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0, 0},
        {LED_RED_GPIO_Port, LED_RED_Pin, 0, 0},
};

void led_on(led name)
{
    // if the led is off.
    if (led_list[name].is_high == 0)
    {
        HAL_GPIO_WritePin(led_list[name].gpio, led_list[name].pin, GPIO_PIN_SET);
        led_list[name].before_tick = HAL_GetTick();
        led_list[name].is_high = 1;
    }
}

void led_process(void)
{
    for (int i = 0; i < LED_LIST_SIZE; i++)
    {
        if (led_list[i].is_high && ((HAL_GetTick() - led_list[i].before_tick) > LED_INTERVAL))
        {
            HAL_GPIO_WritePin(led_list[i].gpio, led_list[i].pin, GPIO_PIN_RESET);
            led_list[i].is_high = 0;
        }
    }
}
