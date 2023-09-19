#ifndef __ENCODER_CONTROL_H__
#define __ENCODER_CONTROL_H__


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
int GetEncoderPulse(void);
void SpeedInnerControl(int nPulse,int nTarget);
void SetMotorVoltageAndDirection(int nMotorPwm);


#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_CONTROL__ */