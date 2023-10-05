#ifndef __ENCODER_CONTROL_H__
#define __ENCODER_CONTROL_H__


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"


typedef struct _Wheel
{
	TIM_HandleTypeDef* PwmTim;//TIM for pwm output
	uint32_t PwmChannel;//
	TIM_HandleTypeDef* EncoderTim;//Tim for encoder
	GPIO_TypeDef* IN_GPIO_Port;
	uint16_t      IN1;
	uint16_t      IN2;
	int nErrorPrev;//��һ��ƫ��ֵ
	int nPwm;//PWM������PWM����
	int nEncoderTarget;
	int nEncoderPulse;
	float fSpeedTarget;
	float fSpeedActual;
	
} Wheel; //define wheel's type 
void WheelsInit(void);
int GetEncoderPulse(Wheel* wheel);
void SpeedInnerControl(Wheel* wheel);
void WheelControlCallback(Wheel* wheel);
//void SetMotorVoltageAndDirection(Wheel* wheel);
int Speed2Pulse(float fSpeedTarget);
float Pulse2Speed(int nPulse);
#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_CONTROL__ */