#include "encoder_control.h"//需要先include这个，后include mecanum.h,否则会找不到Wheel类型
#include "mecanum.h"
const float rx = 0.085;//unit [m]
const float ry = 0.096;//unit [m]
float CarSpeedTarget[3] = {0.0, 0.0, 0.0};
float CarSpeedActual[3]={0.0, 0.0, 0.0};
extern Wheel wheelRB,wheelLB,wheelRF,wheelLF;
void Speed2Wheels(float Vx, float Vy, float omega_z)//行车方向为x，左边为y
{
	wheelLF.fSpeedTarget = -(Vx - Vy - (rx + ry)*omega_z);//因为左边的轮的电机正转是车的反方向
	wheelRF.fSpeedTarget = Vx + Vy + (rx + ry)*omega_z;
	wheelLB.fSpeedTarget = -(Vx + Vy - (rx + ry)*omega_z);//因为左边的轮的电机正转是车的反方向
	wheelRB.fSpeedTarget = Vx - Vy + (rx + ry)*omega_z;
}

void Wheels2Speed(float* output)
{
	
	output[0]=(-wheelLB.fSpeedActual+wheelRB.fSpeedActual)/2;//Vx,注意这里所有左轮的值都经过了一个负号变化，因此速度解算公式不一样
	output[1]=(-wheelLB.fSpeedActual+wheelLF.fSpeedActual)/2;//Vy
	output[2]=(wheelRF.fSpeedActual+wheelLB.fSpeedActual)/(2*(rx+ry));//omega_z
	
}