#include "bsp_led.h"

#include "main.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state;
} LedHw;

static const LedHw g_leds[BSP_LED_COUNT] = {
    [BSP_LED_0] = {LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET},
    [BSP_LED_1] = {LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET},
    [BSP_LED_2] = {LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET},
    [BSP_LED_3] = {LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET},
};

static bool led_valid(BspLedId id)
{
    return id >= BSP_LED_0 && id < BSP_LED_COUNT;
}

void BspLed_Init(void)
{
    BspLed_AllOff();
}

void BspLed_Set(BspLedId id, bool on)
{
    if (!led_valid(id)) {
        return;
    }

    GPIO_PinState state = on ? g_leds[id].active_state : (g_leds[id].active_state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(g_leds[id].port, g_leds[id].pin, state);
}

void BspLed_Toggle(BspLedId id)
{
    if (led_valid(id)) {
        HAL_GPIO_TogglePin(g_leds[id].port, g_leds[id].pin);
    }
}

void BspLed_AllOff(void)
{
    for (uint32_t i = 0; i < BSP_LED_COUNT; ++i) {
        BspLed_Set((BspLedId)i, false);
    }
}

void BspLed_AllOn(void)
{
    for (uint32_t i = 0; i < BSP_LED_COUNT; ++i) {
        BspLed_Set((BspLedId)i, true);
    }
}
