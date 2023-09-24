#include "encoder_control.h"
#include "tim.h"
#include <math.h>
#include <string.h>
const float fRadius=0.0375;//unit [m], wheel's radius 
const float fSp=0.005;//unit [s],sampling period
const float pi=3.1415926f;
int nPpr=1560;//Number of Pulse per round of motor
float fKp = 45.0;
float fKi = 3.0;//���������ͻ��ֲ���
Wheel wheelRB,wheelLB,wheelRF,wheelLF;//define right behind wheel, left behind wheel, right front wheel, left front wheel
void WheelsInit(void)//the global variable can only defined outside a fuction. It can't be assigned outside a function, so it needs a init function.
{
	wheelRB.nEncoderTarget=0;
	wheelRB.EncoderTim = &htim2;
	wheelRB.PwmTim= &htim9;
	wheelRB.PwmChannel=TIM_CHANNEL_1;
	wheelRB.IN_GPIO_Port=GPIOF;
	wheelRB.IN1=GPIO_PIN_1;
	wheelRB.IN2=GPIO_PIN_0;
	wheelRB.fSpeedTarget=0.0;
	wheelRB.nEncoderTarget=0;
	
	wheelLB.nEncoderTarget=0;
	wheelLB.EncoderTim = &htim8;
	wheelLB.PwmTim= &htim9;
	wheelLB.PwmChannel=TIM_CHANNEL_2;
	wheelLB.IN_GPIO_Port=GPIOC;
	wheelLB.IN1=GPIO_PIN_4;
	wheelLB.IN2=GPIO_PIN_0;
	wheelLB.fSpeedTarget=0.0;
	wheelLB.nEncoderTarget=0;
	
}



int Speed2Pulse(float fSpeedTarget)//the maximum speed of single wheel is about 1.3m/s
{
	
	int nPulseTarget;//target pulse in 5ms
	nPulseTarget = (int) round(fSpeedTarget*fSp*nPpr/(2*pi*fRadius));
	return nPulseTarget;//the maximum number pulse can reach about 45 per 0.005s
}

float Pulse2Speed(int nPulse)
{
	float fSpeed;
	fSpeed = nPulse*(2*pi*fRadius)/(fSp*nPpr);
	return fSpeed;
}

int GetEncoderPulse(Wheel* wheel)//��ȡ��Ӧ���Ӷ������ı���������
{
		int nPulse;//��Ŵ�TIM4��ʱ���������ı���������
    nPulse = (short)(__HAL_TIM_GET_COUNTER((*wheel).EncoderTim));//�ȶ�ȡ�����������͵���short������������ֻ᲻�ԣ�����bug
    __HAL_TIM_SET_COUNTER((*wheel).EncoderTim,0);//�ټ���������
    return nPulse;//����������
}

void SpeedInnerControl(Wheel* wheel)//�ٶ��ڻ�����
{
    int nError;//ƫ��
	  int nPwmBais;
	//PID
    nError = (*wheel).nEncoderTarget-(*wheel).nEncoderPulse;//ƫ�� = Ŀ���ٶ� - ʵ���ٶȣ������nTargetӦ�ø�pulse��������һ���ģ�Ҳ����target����Ŀ��pulse

    nPwmBais = fKp * (nError - (*wheel).nErrorPrev) + fKi * nError;//����ʽPI������

    (*wheel).nErrorPrev = nError;//������һ��ƫ��

    (*wheel).nPwm += nPwmBais;//�������

    if((*wheel).nPwm > 999) (*wheel).nPwm = 999;//�������ޣ���ֹ����PWM����
    if((*wheel).nPwm <-999) (*wheel).nPwm =-999;
	
	//Set pwm and direction
		if((*wheel).nPwm < 0)//reverse rotation
        {
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN2, GPIO_PIN_SET);//AIN2,high
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN1, GPIO_PIN_RESET);//AIN1,low
            __HAL_TIM_SET_COMPARE((*wheel).PwmTim, (*wheel).PwmChannel, -(*wheel).nPwm);//set pwm
        }else
        {//forward rotation
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port,(*wheel).IN1, GPIO_PIN_SET);//AIN1,high
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN2, GPIO_PIN_RESET);//AIN2,low
            __HAL_TIM_SET_COMPARE((*wheel).PwmTim,(*wheel).PwmChannel, (*wheel).nPwm);
        }			

 
    return;//�������ֵ
}

void WheelControlCallback(Wheel* wheel)//This function is used in HAL_TIM_PeriodElapsedCallback to control the motor every interaption
{
	if ((*wheel).fSpeedTarget==0.0)//control left behind wheel
	{
		HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN1, GPIO_PIN_RESET);//ȫ�����ɵ͵�ƽ,���رյ��
		HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN2, GPIO_PIN_RESET);
		(*wheel).fSpeedTarget = 0.0;
		(*wheel).nEncoderTarget=Speed2Pulse((*wheel).fSpeedTarget);
		(*wheel).nEncoderPulse = GetEncoderPulse(wheel);
		(*wheel).nPwm=0;}
	else{
		(*wheel).nEncoderTarget=Speed2Pulse((*wheel).fSpeedTarget);
		(*wheel).nEncoderPulse = GetEncoderPulse(wheel);
		SpeedInnerControl(wheel);
		}
}
/*
void SetMotorVoltageAndDirection(Wheel* wheel)//Right behind wheel
{
    if((*wheel).nPwm < 0)//reverse rotation
        {
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN2, GPIO_PIN_SET);//AIN2,high
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN1, GPIO_PIN_RESET);//AIN1,low
            (*wheel).nPwm = -((*wheel).nPwm);//must be positive value
            __HAL_TIM_SET_COMPARE((*wheel).PwmTim, (*wheel).PwmChannel, (*wheel).nPwm);//set pwm
        }else
        {//forward rotation
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port,(*wheel).IN1, GPIO_PIN_SET);//AIN1,high
            HAL_GPIO_WritePin((*wheel).IN_GPIO_Port, (*wheel).IN2, GPIO_PIN_RESET);//AIN2,low
            __HAL_TIM_SET_COMPARE((*wheel).PwmTim,(*wheel).PwmChannel, (*wheel).nPwm);
        }			
}
*/


