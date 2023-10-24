#ifndef __MECANUM_H__
#define __MECANUM_H__


#ifdef __cplusplus
extern "C" {
#endif
//float CarSpeedTarget[3];
//float CarSpeedActual[3];
void Speed2Wheels(float Vx, float Vy, float omega_z);
void Wheels2Speed(float* output);
#ifdef __cplusplus
}
#endif

#endif /* __MECANUM__ */

