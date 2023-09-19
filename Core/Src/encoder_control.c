#include "encoder_control.h"
#include "tim.h"
#include "encoder_control.h"
int nErrorPrev;//��һ��ƫ��ֵ
int nPwmBais,nPwm;//PWM������PWM����
int nEncoderTarget=0;
int nEncoderPulse;
float fKp = 45.0;
float fKi = 3.0;//���������ͻ��ֲ���
int GetEncoderPulse(void)//��ȡTIM2��ʱ���������ı���������
{
		int nPulse;//��Ŵ�TIM4��ʱ���������ı���������
    nPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));//�ȶ�ȡ������
    __HAL_TIM_SET_COUNTER(&htim2,0);//�ټ���������
    return nPulse;//����������
}

void SpeedInnerControl(int nPulse,int nTarget)//�ٶ��ڻ�����
{
    int nError;//ƫ��
		
    nError = nTarget-nPulse;//ƫ�� = Ŀ���ٶ� - ʵ���ٶȣ������nTargetӦ�ø�pulse��������һ���ģ�Ҳ����target����Ŀ��pulse

    nPwmBais = fKp * (nError - nErrorPrev) + fKi * nError;//����ʽPI������

    nErrorPrev = nError;//������һ��ƫ��

    nPwm += nPwmBais;//�������

    if(nPwm > 999) nPwm = 999;//�������ޣ���ֹ����PWM����
    if(nPwm <-999) nPwm =-999;

 
    return;//�������ֵ
}

void SetMotorVoltageAndDirection(int nMotorPwm)//���õ����ѹ�ͷ���
{
    if(nMotorPwm < 0)//reverse rotation
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);//AIN2,high
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);//AIN1,low
            nMotorPwm = (-nMotorPwm);//must be positive value
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, nMotorPwm);//set pwm
        }else
        {//forward rotation
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);//AIN1,high
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);//AIN2,low
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, nMotorPwm);
        }
}

