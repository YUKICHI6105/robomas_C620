/*
 *  MoterCtrl.cpp
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */

#include "MotorCtrl.hpp"
#include <math.h>

void MotorCtrl::setAng(uint16_t data, uint32_t receiveID){
	param.mechanical_angle[receiveID-0x201] = 360.0*data/8191;
}

void MotorCtrl::setVel(uint16_t data, uint32_t receiveID){
	if(data < 0x8000){
		param.velocity[receiveID-0x201] = data*2*3.141592/60.0;
	}else{
		data =~ data;
		param.velocity[receiveID-0x201] = -1*data*2*3.141592/60.0;
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

void MotorCtrl::pachiReset(uint8_t i){
	value1[i]=0;
	value2[i]=0;
	param.goal[i]=0;
	param.e_pre[i]=0;
	param.ie[i]=0;
	param.rotation[i] = param.rotation[i]-param.pachi_pos_target[i];
	param.rotation[i] = param.revolution[i]/360;
	if(param.revolution[i] < 0){
		param.rotation[i]--;
	}
	param.target[i]=0;
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
		//param.revolution_vel[number] = param.revolution_vel[number] + param.velocity[number]*360.0*0.001/(2*3.141592);
		if((param.mechanical_angle[number] - param.pre_mechanical_angle[number]) > 180){
			param.rotation[number]--;
		}else if((param.mechanical_angle[number] - param.pre_mechanical_angle[number]) < -180){
			param.rotation[number]++;
		}
		param.revolution[number] = param.rotation[number]*360.0 + param.mechanical_angle[number];
		param.pre_mechanical_angle[number] = param.mechanical_angle[number];
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
	if(param.mode[number] == Mode::toyopachi){
		if((param.mechanical_angle[number] - param.pre_mechanical_angle[number]) > 180){
			param.rotation[number]--;
		}else if((param.mechanical_angle[number] - param.pre_mechanical_angle[number]) < -180){
			param.rotation[number]++;
		}
		param.revolution[number] = param.rotation[number]*360.0 + param.mechanical_angle[number];
		param.pre_mechanical_angle[number] = param.mechanical_angle[number];
		if(param.target[number] == 1){
			if(param.revolution[number] < param.pachi_pos_target[number]/2){//vel_ctrl
				e = param.pachi_vel_target[number] - param.velocity[number];
				param.ie[number] = param.ie[number] + e;
				if(param.ie[number]>param.limitIe[number]){
					param.ie[number] = param.limitIe[number];
				}else if(param.ie[number]<-1*param.limitIe[number]){
					param.ie[number] = -1*param.limitIe[number];
				}
				param.goal[number] = param.goal[number]+param.pachiVelKp[number]*0.0001*e+param.pachiVelKi[number]*0.0001*param.ie[number]*0.001/2+(param.pachiVelKd[number]*0.0001*(e-param.e_pre[number])/0.001)/2;
				param.e_pre[number] = e;
				}else{//pos_ctrl
				e = param.pachi_pos_target[number] - param.revolution[number];
				param.ie[number] = param.ie[number] + e*0.001;
				if(param.ie[number]>param.limitIe[number]){
					param.ie[number] = param.limitIe[number];
				}else if(param.ie[number]<-1*param.limitIe[number]){
					param.ie[number] = -1*param.limitIe[number];
				}
				param.goal[number] = param.pachiPosKp[number]*0.0001*e+param.pachiPosKi[number]*0.0001*param.ie[number]*0.001/2+(param.pachiPosKd[number]*0.0001*(e-param.e_pre[number])/0.001)/2;
				param.e_pre[number] = e;
				if((param.pachi_pos_target[number]-5) < param.revolution[number]){
					pachiReset(number);
				}
			}
		}
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
		param.rotation[i]=0;
		param.pre_mechanical_angle[i]=0;
}

void MotorCtrl::setFrame(uint8_t usb_msg[]){
	uint8_t number = (usb_msg[0] - 0x30);
	if(usb_msg[1]==0){
		reset(number);
		param.mode[number] = Mode::dis;
	}else if(usb_msg[1]==1){
		reset(number);
		param.mode[number] = Mode::vel;
	}else if(usb_msg[1]==2){
		reset(number);
		param.mode[number] = Mode::pos;
		param.revolution[number] = param.mechanical_angle[number];
		param.pre_mechanical_angle[number] = param.mechanical_angle[number];
	}else if(usb_msg[1]==3){
		reset(number);
		param.mode[number] = Mode::toyopachi;
	}
	/*if(usb_msg[8] == 1){
		diag=1;
	}else if(usb_msg[8] == 1){
		diag=0;
	}*/
	param.limitTemp[number] = usb_msg[2];
	std::memcpy(&param.Kp[number],usb_msg + 3,sizeof(float));
	std::memcpy(&param.Ki[number],usb_msg + 7,sizeof(float));
	std::memcpy(&param.Kd[number],usb_msg + 11,sizeof(float));
	std::memcpy(&param.limitIe[number],usb_msg + 15,sizeof(float));
}

void MotorCtrl::setTarget(uint8_t usb_msg[]){
	uint8_t number = (usb_msg[0]&0x0f)-8;
	std::memcpy(&param.target[number],usb_msg + 1,sizeof(float));
	if(param.mode[number] == Mode::vel){
		if(param.target[number] < -932){
			param.target[number] = -932;
		}else if(param.target[number] > 932){
			param.target[number] = 932;
		}
	}
}

/*
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
			param.revolution[i] = param.mechanical_angle[i];
			param.pre_mechanical_angle[i] = param.mechanical_angle[i];
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
		if(param.mode == Mode::vel){
			if(param.target[i] < -932){
				param.target[i] = -932;
			}else if(param.target[i] > 932){
				param.target[i] = 932;
			}
		}
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
*/
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
