/*
 *  MoterCtrl.cpp
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */

#include "MotorCtrl.hpp"

void MotorCtrl::setAng(uint16_t data, uint32_t receiveID){
	param.mechanical_angle[receiveID-0x201] = 360.0*data/8191 - 180.0f;
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
	uint8_t number = ReceiveID - 0x201;
	setAng(((static_cast<uint16_t>(receiveData[0]) << 8) | receiveData[1]), ReceiveID);
	setVel(((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]), ReceiveID);
	setCur(((static_cast<uint16_t>(receiveData[4]) << 8) | receiveData[5]), ReceiveID);
	param.temp[number] = receiveData[6];
	//vel = ((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]);
	if(param.mode[number] == Mode::dis){
		reset(number);
	}
	if(param.mode[number] == Mode::vel){
		e = param.target[number] - param.velocity[number];
		param.ie[number] = param.ie[number] + e;
		if(param.ie[number]>param.limitIe[number]){
			param.ie[number] = param.limitIe[number];
		}else if(param.ie[number]<-1*param.limitIe[number]){
			param.ie[number] = -1*param.limitIe[number];
		}
		param.goal[number] = param.goal[number]+param.Kp[number]*0.0001*e+param.Ki[number]*0.0001*param.ie[number]*0.001/2+(param.Kd[number]*0.0001*(e-param.e_pre[number])/0.001)/2;
		param.e_pre[number] = e;
	}
	if(param.mode[number] == Mode::pos){
//		if(param.mechanical_angle[number] >= 0){
//			param.revolution[number] = param.mechanical_angle[number] + 360*static_cast<uint32_t>((param.revolution[number] + param.velocity[number]*360/(2*3.141592)*0.001)/360);
//		}else{
//			param.revolution[number] = param.mechanical_angle[number] + 360*(static_cast<uint32_t>((param.revolution[number] + param.velocity[number]*360/(2*3.141592)*0.001)/360)+1);
//		}
		param.revolution[number] = param.revolution[number] + param.velocity[number]*360/(2*3.141592)*0.001;
		e = param.target[number] - param.revolution[number];
		param.ie[number] = param.ie[number] + e*0.001;
		if(param.ie[number]>param.limitIe[number]){
			param.ie[number] = param.limitIe[number];
		}else if(param.ie[number]<-1*param.limitIe[number]){
			param.ie[number] = -1*param.limitIe[number];
		}
		param.goal[number] = param.Kp[number]*0.0001*e+param.Ki[number]*0.0001*param.ie[number]*0.001/2+(param.Kd[number]*0.0001*(e-param.e_pre[number])/0.001)/2;
		param.e_pre[number] = e;
	}
	if(param.mode[number] == Mode::hom){

	}
	return true;
}

void MotorCtrl::reset(uint8_t i){
		value1[i]=0;
		value2[i]=0;
		param.target[i]=0;
		param.mode[i]=Mode::dis;
		param.goal[i]=0;
		param.e_pre[i]=0;
		param.ie[i]=0;
		param.revolution[i]=0;
}

void MotorCtrl::setMode(uint8_t usb_msg[]){
	for(int i = 0;i<8;i++){
		if(usb_msg[i+1]==0){
			param.mode[i] = Mode::dis;
			reset(i);
		}else if(usb_msg[i+1]==1){
			param.mode[i] = Mode::vel;
			reset(i);
		}else if(usb_msg[i+1]==2){
			param.mode[i] = Mode::pos;
			//param.revolution[i] = param.mechanical_angle[i] - 180.0f;
			reset(i);
		}else if(usb_msg[i+1]==3){
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

void MotorCtrl::setLimitIe(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.limitIe[i],&buf,1);
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
			value1[2*i] = static_cast<uint8_t>(changeValue(param.goal[i]) >> 8);
			value1[2*i+1] = static_cast<uint8_t>(changeValue(param.goal[i]) & 0xFF);
		}else{
			for(int i = 0;i<8;i++){
				value1[i]=0;
			}
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
		if(param.temp[i] < param.limitTemp[i]){
			value2[2*(i-4)] = static_cast<uint8_t>(changeValue(param.goal[i]) >> 8);
			value2[2*(i-4)+1] = static_cast<uint8_t>(changeValue(param.goal[i]) & 0xFF);
		}else{
			for(int i = 0;i<8;i++){
				value2[i]=0;
			}
		}
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
	    {
	        led_on(can);
	        HAL_CAN_AddTxMessage(&hcan, &TxHeader2, value2, &TxMailbox);
	    }
}
}

void MotorCtrl::ems(){
	uint8_t value3[8] = {0,0,0,0,0,0,0,0};
	led_on(can);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader1, value3, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader2, value3, &TxMailbox);
}
