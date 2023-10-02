#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "led.h"
#include "can_usb.h"
#include "MotorCtrl.hpp"

MotorCtrl motor;

extern"C"{
	extern TIM_HandleTypeDef htim3;
	extern CAN_HandleTypeDef hcan;
	extern CAN_TxHeaderTypeDef TxHeader;
	extern USBD_HandleTypeDef hUsbDeviceFS;
}

CAN_TxHeaderTypeDef TxHeader1;
CAN_TxHeaderTypeDef TxHeader2;

void main_cpp()
{
	TxHeader1.IDE = CAN_ID_STD;
	TxHeader1.RTR = CAN_RTR_DATA;
	TxHeader1.StdId = 0x200;
	TxHeader1.DLC = 8;
	TxHeader1.TransmitGlobalTime = DISABLE;
	TxHeader2.IDE = CAN_ID_STD;
	TxHeader2.RTR = CAN_RTR_DATA;
	TxHeader2.StdId = 0x1ff;
	TxHeader2.DLC = 8;
	TxHeader2.TransmitGlobalTime = DISABLE;

	HAL_CAN_Start(&hcan);
	HAL_TIM_Base_Start_IT(&htim3);

	uint8_t debug_state = 0;
	while (true)
	{
		if (READ_BIT(CAN->MSR, CAN_MSR_INAK) == 1)
		{
			// can start
			CLEAR_BIT(CAN->MCR, CAN_MCR_INRQ);
			led_on(green);
		}
		if (debug_state == 0)
		{
			switch (hUsbDeviceFS.dev_state)
			{
			case USBD_STATE_DEFAULT:
				led_on(red);
				break;
			case USBD_STATE_ADDRESSED:
				led_on(yellow);
				break;
			case USBD_STATE_SUSPENDED:
				led_on(yellow);
				led_on(red);
				break;
			case USBD_STATE_CONFIGURED:
				led_on(green);
				debug_state = 1;
				break;
			default:
				break;
			}
		}
		else if (debug_state == 1)
		{
			switch (hcan.ErrorCode)
			{
			case HAL_CAN_ERROR_BD: // bit dominant error
				led_on(red);
				break;
			case HAL_CAN_ERROR_ACK: // acknowledge error
				led_on(yellow);
				break;
			default:
				break;
			}
		}

		led_process(); //turn off all led
	}
}
