#pragma once

#include <main.h>
#ifdef __cplusplus
extern "C"
{
#endif

    // it process all usb messages
    void usb_process(uint8_t usb_msg[], const uint8_t len);

    // it process all can messages
    void can_process(const CAN_RxHeaderTypeDef *RxHeader, uint8_t Data[]);

    // void seliarize_canheader(const CAN_RxHeaderTypeDef *RxHeader,uint8_t Data[]);
    // void send_to_usb(uint8_t RawData[],uint8_t len);

#ifdef __cplusplus
}
#endif
