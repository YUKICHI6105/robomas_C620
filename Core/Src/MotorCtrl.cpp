/*
 *  MoterCtrl.cpp
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */

#include "MotorCtrl.hpp"

void MotorCtrl::setAng(uint16_t data, uint32_t receiveID){
	param.mechanical_angle[receiveID-0x201] = 360.0*data/8191;
}

void MotorCtrl::setVel(uint16_t data, uint32_t receiveID){
	if(data < 0x8000){
		param.velocity[receiveID-0x201] = data*3.141592/60.0;
	}else{
		data =~ data;
		param.velocity[receiveID-0x201] = -1*data*3.141592/60.0;
	}
}

void MotorCtrl::setCur(uint16_t data, uint32_t receiveID){
	if((data & 0x8000) == 0x8000){
		data =~ data;
		param.current[receiveID-0x201] = -20*data/16384;
	}else{
		param.current[receiveID-0x201] = 20*data/16384;
	}
}

bool MotorCtrl::update(uint32_t ReceiveID,uint8_t receiveData[8]){
	if(ReceiveID<0x201||ReceiveID>0x208){return false;}
	setAng(((static_cast<uint16_t>(receiveData[0]) << 8) | receiveData[1]), ReceiveID);
	setVel(((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]), ReceiveID);
	setCur(((static_cast<uint16_t>(receiveData[4]) << 8) | receiveData[5]), ReceiveID);
	param.temp[ReceiveID-0x201] = receiveData[6];
	//vel = ((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]);
	if(param.mode[ReceiveID-0x201] == Mode::dis){
		reset(ReceiveID-0x201);
	}
	if(param.mode[ReceiveID-0x201] == Mode::pos){
		e = param.target[ReceiveID-0x201] - param.mechanical_angle[ReceiveID-0x201];
		param.gool[ReceiveID-0x201] = param.gool[ReceiveID-0x201]+param.Kp[ReceiveID-0x201]*e+param.Ki[ReceiveID-0x201]*(e+param.e_pre[ReceiveID-0x201])*0.001/2+param.Kd[ReceiveID-0x201]*(e-param.e_pre[ReceiveID-0x201])/0.001;
		param.e_pre[ReceiveID-0x201] = e;
	}
	if(param.mode[ReceiveID-0x201] == Mode::vel){
		e = param.target[ReceiveID-0x201] - param.velocity[ReceiveID-0x201];
		param.gool[ReceiveID-0x201] = param.gool[ReceiveID-0x201]+param.Kp[ReceiveID-0x201]*e+param.Ki[ReceiveID-0x201]*(e+param.e_pre[ReceiveID-0x201])*0.001/2+param.Kd[ReceiveID-0x201]*(e-param.e_pre[ReceiveID-0x201])/0.001;
		param.e_pre[ReceiveID-0x201] = e;
	}
	if(param.mode[ReceiveID-0x201] == Mode::hom){

	}
	return true;
}

void MotorCtrl::reset(uint8_t i){
		value1[i]=0;
		value2[i]=0;
		param.target[i]=0;
		param.mode[i]=Mode::dis;
		param.gool[i]=0;
		param.e_pre[i]=0;
}

void MotorCtrl::setMode(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		if(usb_msg[i]==0){
			param.mode[i] = Mode::dis;
			reset(i);
		}else if(usb_msg[i]==1){
			param.mode[i] = Mode::vel;
			reset(i);
		}else if(usb_msg[i]==2){
			param.mode[i] = Mode::pos;
			reset(i);
		}else if(usb_msg[i]==3){
			param.mode[i] = Mode::hom;
			reset(i);
		}
	}
	if(usb_msg[8] == 1){
		diag=1;
	}else if(usb_msg[8] == 1){
		diag=0;
	}
}

void MotorCtrl::setLimitTemp(uint8_t usb_msg[]){
	for(int i = 0;i<8;i++){
		param.limitTemp[i]=usb_msg[i];
	}
}

void MotorCtrl::setTarget(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.target[i],&buf,1);
	}
}

void MotorCtrl::setKp(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.Kp[i],&buf,1);
	}
}

void MotorCtrl::setKi(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.Ki[i],&buf,1);
	}
}

void MotorCtrl::setKd(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.Kp[i],&buf,1);
	}
}

uint16_t changeValue(float target){
	uint16_t value;
	if(target < 0.0){
		target = -target;
		if(target < 20.0){
			value = target/20*16384;
			value =~ value;
		}else{
			value = 16384;
			value =~ value;
		}
	}else{
		if(target < 20.0){
			value = target/20*16384;
		}else{
			value = 16384;
		}
	}
	return value;
}

void MotorCtrl::transmit1(){
	for(int i=0;i<4;i++){
		if(param.temp[i] < param.limitTemp[i]){
			value1[2*i] = static_cast<uint8_t>(changeValue(param.gool[i]) >> 8);
			value1[2*i+1] = static_cast<uint8_t>(changeValue(param.gool[i]) & 0xFF);
		}
	}
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
	    {
	        led_on(can);
	        HAL_CAN_AddTxMessage(&hcan, &TxHeader1, value1, &TxMailbox);
	    }
}

void MotorCtrl::transmit2(){
	for(int i=4;i<8;i++){
			value2[2*(i-4)] = static_cast<uint8_t>(changeValue(param.gool[i]) >> 8);
			value2[2*(i-4)+1] = static_cast<uint8_t>(changeValue(param.gool[i]) & 0xFF);
		}
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
	    {
	        led_on(can);
	        HAL_CAN_AddTxMessage(&hcan, &TxHeader2, value2, &TxMailbox);
	    }
}

void MotorCtrl::ems(){
	uint8_t value3[8] = {0,0,0,0,0,0,0,0};
	led_on(can);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader1, value3, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader2, value3, &TxMailbox);
}
