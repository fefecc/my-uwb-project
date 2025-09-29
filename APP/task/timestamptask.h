#ifndef _TIME_STAMP_H_
#define _TIME_STAMP_H_

#include "main.h"

typedef struct {
  uint64_t sec;
  float _50us;
} timestamp_def;

timestamp_def GetCurrentTimestamp(void);

void TIM_Call_Callback(void);

#endif /* _TIME_STAMP_H_ */
