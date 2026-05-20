#ifndef APP_BSP_BSP_KEY_H_
#define APP_BSP_BSP_KEY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_KEY_EVENT_NONE = 0,
    BSP_KEY_EVENT_PRESS,
    BSP_KEY_EVENT_RELEASE,
    BSP_KEY_EVENT_SHORT_PRESS,
    BSP_KEY_EVENT_LONG_PRESS,
    BSP_KEY_EVENT_LONG_RELEASE,
} BspKeyEvent;

typedef struct {
    bool pressed;
    bool raw_pressed;
    bool long_reported;
    uint32_t press_tick;
    uint32_t debounce_tick;
} BspKeyState;

void BspKey_Init(BspKeyState *state);
bool BspKey_IsPressed(void);
BspKeyEvent BspKey_Poll(BspKeyState *state,
                         uint32_t short_press_ms,
                         uint32_t long_press_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_BSP_KEY_H_ */
