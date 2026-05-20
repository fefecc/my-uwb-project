#include "bsp_key.h"

#include "main.h"

#define BSP_KEY_DEBOUNCE_MS (20U)
#define BSP_KEY_ACTIVE_LEVEL GPIO_PIN_RESET

void BspKey_Init(BspKeyState *state)
{
    if (state != NULL) {
        uint32_t now = HAL_GetTick();
        bool pressed = BspKey_IsPressed();

        state->pressed = pressed;
        state->raw_pressed = pressed;
        state->long_reported = false;
        state->press_tick = now;
        state->debounce_tick = now;
    }
}

bool BspKey_IsPressed(void)
{
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) ==
           BSP_KEY_ACTIVE_LEVEL;
}

BspKeyEvent BspKey_Poll(BspKeyState *state,
                         uint32_t short_press_ms,
                         uint32_t long_press_ms)
{
    if (state == NULL) {
        return BSP_KEY_EVENT_NONE;
    }

    uint32_t now = HAL_GetTick();
    bool raw_pressed = BspKey_IsPressed();

    if (raw_pressed != state->raw_pressed) {
        state->raw_pressed = raw_pressed;
        state->debounce_tick = now;
        return BSP_KEY_EVENT_NONE;
    }

    if (raw_pressed != state->pressed) {
        if ((now - state->debounce_tick) < BSP_KEY_DEBOUNCE_MS) {
            return BSP_KEY_EVENT_NONE;
        }

        state->pressed = raw_pressed;

        if (raw_pressed) {
            state->press_tick = now;
            state->long_reported = false;
            return BSP_KEY_EVENT_PRESS;
        }

        uint32_t pressed_ms = now - state->press_tick;
        bool was_long = state->long_reported ||
                        (pressed_ms >= long_press_ms);
        state->long_reported = false;

        if (was_long) {
            return BSP_KEY_EVENT_LONG_RELEASE;
        }

        if (pressed_ms <= short_press_ms) {
            return BSP_KEY_EVENT_SHORT_PRESS;
        }

        return BSP_KEY_EVENT_NONE;
    }

    if (state->pressed && !state->long_reported &&
        (now - state->press_tick) >= long_press_ms) {
        state->long_reported = true;
        return BSP_KEY_EVENT_LONG_PRESS;
    }

    return BSP_KEY_EVENT_NONE;
}
