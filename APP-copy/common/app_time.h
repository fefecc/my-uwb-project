#ifndef APP_COMMON_APP_TIME_H_
#define APP_COMMON_APP_TIME_H_

#include <stdint.h>

typedef struct {
    uint64_t utc_ms;
    uint32_t tick_ms;
} app_timepoint_t;

void AppTime_Init(void);
void AppTime_UpdateUtc(uint64_t utc_ms);
app_timepoint_t AppTime_Now(void);

#endif /* APP_COMMON_APP_TIME_H_ */
