/*
 *  MoterCtrl.hpp
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */

#include <cstring>
#include <main.h>
#include "stm32f3xx_hal.h"
#include "led.h"

extern"C"{
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader1;
extern CAN_TxHeaderTypeDef TxHeader2;
extern uint32_t TxMailbox;
}

enum class Mode{
	dis,
	vel,
	pos,
	berutyoku,
	stablepos,
};

enum class Motor{
	C610,
	C620,
};

struct PIDParam{
	float Kp[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float Ki[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float Kd[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float e_pre[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};//de
	float ie[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float limitIe[8] = {100000.0,100000.0,100000.0,100000.0,100000.0,100000.0,100000.0,100000.0};
};

struct MotorParam{
	Motor motorKinds[8] = {Motor::C620,Motor::C620,Motor::C620,Motor::C620,Motor::C620,Motor::C620,Motor::C620,Motor::C620};
	Mode mode[8] = {Mode::dis,Mode::dis,Mode::dis,Mode::dis,Mode::dis,Mode::dis,Mode::dis,Mode::dis};
	float goal[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//PID処理後の操作量
	float target[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};//目標値
	float tyoku_vel_target[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float tyoku_pos_target[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float stableLimitVel[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float mechanical_angle[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};//機械角
	float pre_mechanical_angle[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float init_mechanical_angle[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	float velocity[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};//rpm->rad/s
	float current[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	uint8_t temp[8] = {0,0,0,0,0,0,0,0};
	uint8_t limitTemp[8] = {50,50,50,50,50,50,50,50};
	float revolution[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	int32_t rotation[8] = {0,0,0,0,0,0,0,0};
	//float revolution_vel[8] = {0,0,0,0,0,0,0,0};
};

class MotorCtrl{
private:
	MotorParam param;
	PIDParam posPIDParam;
	PIDParam velPIDParam;
	float e = 0.0;
	//uint16_t vel = 0;
public:
	void setAng(uint16_t data, uint32_t receiveID);
	void setVel(uint16_t data, uint32_t receiveID);
	void setCurC610(uint16_t data, uint32_t receiveID);
	void setCurC620(uint16_t data, uint32_t receiveID);
	void setRevolution(uint8_t& number);
	void velPID(uint8_t& number, float& target);
	float posPID(uint8_t& number, float& target);
	void tyokuReset(uint8_t i);
	void stableposPID(uint8_t& number, float& target);
	bool update(uint32_t ReceiveID,uint8_t receiveData[8]);
	void reset(uint8_t i);
	void setFrame(uint8_t usb_msg[], const uint8_t len);
	void setTarget(uint8_t usb_msg[]);
	//void setMode(uint8_t usb_msg[]);
	//void setLimitTemp(uint8_t usb_msg[]);
	//void setTarget(uint8_t usb_msg[]);
	//void setKp(uint8_t usb_msg[]);
	//void setKi(uint8_t usb_msg[]);
	//void setKd(uint8_t usb_msg[]);
	//void setLimitIe(uint8_t usb_msg[]);
	uint8_t value1[8];
	uint8_t value2[8];
	void transmit1();
	void transmit2();
	void ems();
	uint8_t diag = 0;
	uint8_t data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t usb_data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	void test_usb(uint8_t usb_msg[]);
	void usb(uint8_t usb_msg[]);
};

inline void MotorCtrl::test_usb(uint8_t usb_msg[]){
	for(int i = 0; i < 20; i++){
		data[i] = usb_msg[i];
	}
}

inline void MotorCtrl::usb(uint8_t usb_msg[]){
	for(int i = 0; i < 20; i++){
		usb_data[i] = usb_msg[i];
	}
}
