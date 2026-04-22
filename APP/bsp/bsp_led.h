#ifndef APP_BSP_BSP_LED_H_
#define APP_BSP_BSP_LED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_LED_0 = 0,
    BSP_LED_1,
    BSP_LED_2,
    BSP_LED_3,
    BSP_LED_COUNT,
} BspLedId;

void BspLed_Init(void);
void BspLed_Set(BspLedId id, bool on);
void BspLed_Toggle(BspLedId id);
void BspLed_AllOff(void);
void BspLed_AllOn(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_BSP_LED_H_ */
