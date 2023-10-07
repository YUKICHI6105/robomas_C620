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

void MotorCtrl::setCurC610(uint16_t data, uint32_t receiveID){
	if((data & 0x8000) == 0x8000){
		data =~ data;
		param.current[receiveID-0x201] = -10*data/10000;
	}else{
		param.current[receiveID-0x201] = 10*data/10000;
	}
}

void MotorCtrl::setCurC620(uint16_t data, uint32_t receiveID){
	if((data & 0x8000) == 0x8000){
		data =~ data;
		param.current[receiveID-0x201] = -20*data/16384;
	}else{
		param.current[receiveID-0x201] = 20*data/16384;
	}
}

void MotorCtrl::setRevolution(uint8_t& number){
	//param.revolution_vel[number] = param.revolution_vel[number] + param.velocity[number]*360.0*0.001/(2*3.141592);
	if((param.mechanical_angle[number] - param.pre_mechanical_angle[number]) > 180){
		param.rotation[number]--;
	}else if((param.mechanical_angle[number] - param.pre_mechanical_angle[number]) < -180){
		param.rotation[number]++;
	}
	param.revolution[number] = param.rotation[number]*360.0 + param.mechanical_angle[number] - param.init_mechanical_angle[number];
	param.pre_mechanical_angle[number] = param.mechanical_angle[number];
}

void MotorCtrl::velPID(uint8_t& number, float& target){
	e = target - param.velocity[number];
	velPIDParam.ie[number] = velPIDParam.ie[number] + (e+velPIDParam.e_pre[number])*0.001/2;
	if(velPIDParam.ie[number]>velPIDParam.limitIe[number]){
		velPIDParam.ie[number] = velPIDParam.limitIe[number];
	}else if(velPIDParam.ie[number]<-1*velPIDParam.limitIe[number]){
		velPIDParam.ie[number] = -1*velPIDParam.limitIe[number];
	}
	param.goal[number] = param.goal[number]+velPIDParam.Kp[number]*0.0001*e+velPIDParam.Ki[number]*0.0001*velPIDParam.ie[number]+(velPIDParam.Kd[number]*0.0001*(e-velPIDParam.e_pre[number])/0.001);
	if(param.motorKinds[number]== Motor::C610){
		if(param.goal[number] > 10){
			param.goal[number] = 10;
		}else if(param.goal[number] < -10){
			param.goal[number] = -10;
		}
	}else{
		if(param.goal[number] > 20){
			param.goal[number] = 20;
		}else if(param.goal[number] < -20){
			param.goal[number] = -20;
		}
	}
	velPIDParam.e_pre[number] = e;
}

float MotorCtrl::posPID(uint8_t& number, float& target){
	e = target - param.revolution[number];
	posPIDParam.ie[number] = posPIDParam.ie[number] + (e+posPIDParam.e_pre[number])*0.001/2;
	if(posPIDParam.ie[number]>posPIDParam.limitIe[number]){
		posPIDParam.ie[number] = posPIDParam.limitIe[number];
	}else if(posPIDParam.ie[number]<-1*posPIDParam.limitIe[number]){
		posPIDParam.ie[number] = -1*posPIDParam.limitIe[number];
	}
	float value = posPIDParam.Kp[number]*0.0001*e+posPIDParam.Ki[number]*0.0001*posPIDParam.ie[number]+(posPIDParam.Kd[number]*0.0001*(e-posPIDParam.e_pre[number])/0.001);
	posPIDParam.e_pre[number] = e;
	return value;
}

void MotorCtrl::stableposPID(uint8_t& number, float& target){
	float value = posPID(number,target);
	if(value < param.stableLimitVel[number]){
		value = param.stableLimitVel[number];
	}else if(value > -1*param.stableLimitVel[number]){
		value = -1*param.stableLimitVel[number];
	}
	velPID(number, value);
}

void MotorCtrl::tyokuReset(uint8_t i){
	value1[i]=0;
	value2[i]=0;
	param.goal[i]=0;
	velPIDParam.e_pre[i]=0;
	posPIDParam.e_pre[i]=0;
	velPIDParam.ie[i]=0;
	posPIDParam.ie[i]=0;
	param.revolution[i] = param.revolution[i]-param.tyoku_pos_target[i];
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
	if(param.motorKinds[number] == Motor::C610){
		setCurC610(((static_cast<uint16_t>(receiveData[4]) << 8) | receiveData[5]), ReceiveID);
	}else{
		setCurC620(((static_cast<uint16_t>(receiveData[4]) << 8) | receiveData[5]), ReceiveID);
		param.temp[number] = receiveData[6];
	}
	//vel = ((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]);
	if(param.mode[number] == Mode::dis){
		reset(number);
	}
	if(param.mode[number] == Mode::vel){
		velPID(number, param.target[number]);
	}
	if(param.mode[number] == Mode::pos){
		setRevolution(number);
		param.goal[number] = posPID(number, param.target[number]);
	}
	if(param.mode[number] == Mode::berutyoku){
		setRevolution(number);
		if(param.target[number] == 1){
			if(param.revolution[number] < param.tyoku_pos_target[number]/2){//vel_ctrl
				velPID(number, param.tyoku_vel_target[number]);
			}else{//pos_ctrl
				param.goal[number] = posPID(number, param.tyoku_pos_target[number]);
				if((param.velocity[number] == 0.0) && (param.revolution[number] > (param.tyoku_pos_target[number]-5)) && (param.revolution[number] < (param.tyoku_pos_target[number]+5))){
					tyokuReset(number);
				}
			}
		}
	}
	if(param.mode[number] == Mode::stablepos){
		setRevolution(number);
		stableposPID(number,param.target[number]);
	}
	return true;
}

void MotorCtrl::reset(uint8_t i){
		value1[i]=0;
		value2[i]=0;
		param.target[i]=0;
		param.mode[i]=Mode::dis;
		param.goal[i]=0;
		velPIDParam.e_pre[i]=0;
		velPIDParam.ie[i]=0;
		posPIDParam.e_pre[i]=0;
		posPIDParam.ie[i]=0;
		param.revolution[i]=0;
		param.rotation[i]=0;
		param.pre_mechanical_angle[i]=0;
}

void MotorCtrl::setFrame(uint8_t usb_msg[], const uint8_t len){
	if((len < 31) && ((usb_msg[1] & 0x7f)==3)){return;}
	uint8_t number = (usb_msg[0] & 0x07);
	if((usb_msg[1] & 0x80) == 0){
		param.motorKinds[number] = Motor::C610;
	}else{
		param.motorKinds[number] = Motor::C620;
	}
	if((usb_msg[1] & 0x7f)==0){
		reset(number);
		param.mode[number] = Mode::dis;
	}else if((usb_msg[1] & 0x7f)==1){
		reset(number);
		param.mode[number] = Mode::vel;
		std::memcpy(&velPIDParam.Kp[number],usb_msg + 3,sizeof(float));
		std::memcpy(&velPIDParam.Ki[number],usb_msg + 7,sizeof(float));
		std::memcpy(&velPIDParam.Kd[number],usb_msg + 11,sizeof(float));
		std::memcpy(&velPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
	}else if((usb_msg[1] & 0x7f)==2){
		reset(number);
		param.mode[number] = Mode::pos;
		std::memcpy(&posPIDParam.Kp[number],usb_msg + 3,sizeof(float));
		std::memcpy(&posPIDParam.Ki[number],usb_msg + 7,sizeof(float));
		std::memcpy(&posPIDParam.Kd[number],usb_msg + 11,sizeof(float));
		std::memcpy(&posPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
		//param.revolution[number] = param.mechanical_angle[number];
		param.pre_mechanical_angle[number] = param.mechanical_angle[number];
		param.init_mechanical_angle[number] = param.mechanical_angle[number];
	}else if((usb_msg[1] & 0x7f)==3){
		if(param.mode[number] != Mode::berutyoku){
			reset(number);
			param.mode[number] = Mode::berutyoku;
		}
		std::memcpy(&velPIDParam.Kp[number],usb_msg + 3,sizeof(float));
		std::memcpy(&velPIDParam.Ki[number],usb_msg + 7,sizeof(float));
		std::memcpy(&velPIDParam.Kd[number],usb_msg + 11,sizeof(float));
		std::memcpy(&velPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
		std::memcpy(&posPIDParam.Kp[number],usb_msg + 19,sizeof(float));
		std::memcpy(&posPIDParam.Ki[number],usb_msg + 23,sizeof(float));
		std::memcpy(&posPIDParam.Kd[number],usb_msg + 27,sizeof(float));
		std::memcpy(&posPIDParam.limitIe[number],usb_msg + 31,sizeof(float));
		std::memcpy(&param.tyoku_vel_target[number],usb_msg + 35,sizeof(float));
		std::memcpy(&param.tyoku_pos_target[number],usb_msg + 39,sizeof(float));
	}else if((usb_msg[1] & 0x7f)==4){
		reset(number);
		param.mode[number] = Mode::stablepos;
		std::memcpy(&velPIDParam.Kp[number],usb_msg + 3,sizeof(float));
		std::memcpy(&velPIDParam.Ki[number],usb_msg + 7,sizeof(float));
		std::memcpy(&velPIDParam.Kd[number],usb_msg + 11,sizeof(float));
		std::memcpy(&velPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
		std::memcpy(&posPIDParam.Kp[number],usb_msg + 19,sizeof(float));
		std::memcpy(&posPIDParam.Ki[number],usb_msg + 23,sizeof(float));
		std::memcpy(&posPIDParam.Kd[number],usb_msg + 27,sizeof(float));
		std::memcpy(&posPIDParam.limitIe[number],usb_msg + 31,sizeof(float));
	}
	/*if(usb_msg[8] == 1){
		diag=1;
	}else if(usb_msg[8] == 1){
		diag=0;
	}*/
	param.limitTemp[number] = usb_msg[2];
}

void MotorCtrl::setTarget(uint8_t usb_msg[]){
	uint8_t number = (usb_msg[0]&0x07);
	std::memcpy(&param.target[number],usb_msg + 1,sizeof(float));
	if(param.mode[number] == Mode::vel){
		if(param.motorKinds[number] == Motor::C610){
			if(param.target[number] < -1885){
				param.target[number] = -1885;
			}else if(param.target[number] > 1885){
				param.target[number] = 1885;
			}
		}else if(param.motorKinds[number] == Motor::C620){
			if(param.target[number] < -943){
				param.target[number] = -943;
			}else if(param.target[number] > 943){
				param.target[number] = 943;
			}
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
uint16_t getC620Value(float target){
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

uint16_t getC610Value(float target){
	uint16_t value;
	if(target < 0.0){
		target = -target;
		if(target < 10.0){
			value = target/10*10000;
			value =~ value;
		}else{
			value = 10000;
			value =~ value;
		}
	}else{
		if(target < 10.0){
			value = target/10*10000;
		}else{
			value = 10000;
		}
	}
	return value;
}

void MotorCtrl::transmit1(){
	for(int i=0;i<4;i++){
		if(param.temp[i] < param.limitTemp[i]){
			if(param.motorKinds[i] == Motor::C620){
				value1[2*i] = static_cast<uint8_t>(getC620Value(param.goal[i]) >> 8);
				value1[2*i+1] = static_cast<uint8_t>(getC620Value(param.goal[i]) & 0xFF);
			}else{
				value1[2*i] = static_cast<uint8_t>(getC610Value(param.goal[i]) >> 8);
				value1[2*i+1] = static_cast<uint8_t>(getC610Value(param.goal[i]) & 0xFF);
			}
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
			if(param.motorKinds[i] == Motor::C620){
				value2[2*(i-4)] = static_cast<uint8_t>(getC620Value(param.goal[i]) >> 8);
				value2[2*(i-4)+1] = static_cast<uint8_t>(getC620Value(param.goal[i]) & 0xFF);
			}else{
				value2[2*(i-4)] = static_cast<uint8_t>(getC610Value(param.goal[i]) >> 8);
				value2[2*(i-4)+1] = static_cast<uint8_t>(getC610Value(param.goal[i]) & 0xFF);
			}
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
