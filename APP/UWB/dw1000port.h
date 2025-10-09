#ifndef _DW1000_H_
#define _DW1000_H_
#include "main.h"

#define DWRSTnGPIOx    GPIOD
#define DWRSTnGPIOPINx GPIO_PIN_9

void reset_DW1000(void);

void spi_set_rate_high(void);

void spi_set_rate_low(void);

#endif