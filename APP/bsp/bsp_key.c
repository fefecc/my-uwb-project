#include "bsp_key.h"

#include "main.h"

#define BSP_KEY_DEBOUNCE_MS (20U)

void BspKey_Init(BspKeyState *state)
{
    if (state != NULL) {
        state->pressed = BspKey_IsPressed();
        state->long_reported = false;
        state->press_tick = HAL_GetTick();
        state->debounce_tick = HAL_GetTick();
    }
}

bool BspKey_IsPressed(void)
{
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET;
}

BspKeyEvent BspKey_Poll(BspKeyState *state, uint32_t long_press_ms)
{
    if (state == NULL) {
        return BSP_KEY_EVENT_NONE;
    }

    uint32_t now = HAL_GetTick();
    bool raw_pressed = BspKey_IsPressed();

    if (raw_pressed != state->pressed) {
        if ((now - state->debounce_tick) < BSP_KEY_DEBOUNCE_MS) {
            return BSP_KEY_EVENT_NONE;
        }

        state->debounce_tick = now;
        state->pressed = raw_pressed;

        if (raw_pressed) {
            state->press_tick = now;
            state->long_reported = false;
            return BSP_KEY_EVENT_PRESS;
        }

        bool was_long = state->long_reported ||
                        ((now - state->press_tick) >= long_press_ms);
        state->long_reported = false;
        return was_long ? BSP_KEY_EVENT_LONG_RELEASE :
                          BSP_KEY_EVENT_SHORT_PRESS;
    }

    if (state->pressed && !state->long_reported &&
        (now - state->press_tick) >= long_press_ms) {
        state->long_reported = true;
        return BSP_KEY_EVENT_LONG_PRESS;
    }

    return BSP_KEY_EVENT_NONE;
}
