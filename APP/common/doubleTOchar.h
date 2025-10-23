#ifndef _DOUBLETOCHAR_H_
#define _DOUBLETOCHAR_H_

#include "main.h"

int double_to_bytes(double d, uint8_t *bytes_out, size_t buffer_size);
double bytes_to_double(const uint8_t *bytes_in, size_t buffer_size,
                       int *success);
#endif