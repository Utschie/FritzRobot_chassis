#include "encoder_control.h"
#include "tim.h"
#include "encoder_control.h"
int nErrorPrev;//上一次偏差值
int nPwmBais,nPwm;//PWM增量，PWM总量
int nEncoderTarget=0;
int nEncoderPulse;
float fKp = 45.0;
float fKi = 3.0;//比例参数和积分参数
int GetEncoderPulse(void)//获取TIM2定时器读出来的编码器脉冲
{
		int nPulse;//存放从TIM4定时器读出来的编码器脉冲
    nPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));//先读取脉冲数
    __HAL_TIM_SET_COUNTER(&htim2,0);//再计数器清零
    return nPulse;//返回脉冲数
}

void SpeedInnerControl(int nPulse,int nTarget)//速度内环控制
{
    int nError;//偏差
		
    nError = nTarget-nPulse;//偏差 = 目标速度 - 实际速度，这里的nTarget应该跟pulse的量纲是一样的，也就是target就是目标pulse

    nPwmBais = fKp * (nError - nErrorPrev) + fKi * nError;//增量式PI控制器

    nErrorPrev = nError;//保存上一次偏差

    nPwm += nPwmBais;//增量输出

    if(nPwm > 999) nPwm = 999;//限制上限，防止超出PWM量程
    if(nPwm <-999) nPwm =-999;

 
    return;//返回输出值
}

void SetMotorVoltageAndDirection(int nMotorPwm)//设置电机电压和方向
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

