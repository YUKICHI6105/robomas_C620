#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
void cobs_encode(const uint8_t data[], uint8_t return_data[], int data_size);
// Consistent Overhead Byte Suffiing
// data_size is data array size
// return data array size should be data_size - 2
void cobs_decode(const uint8_t data[], uint8_t return_data[], int data_size);

#ifdef __cplusplus
}
#endif
