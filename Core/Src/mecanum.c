#include "encoder_control.h"//��Ҫ��include�������include mecanum.h,������Ҳ���Wheel����
#include "mecanum.h"
const float rx = 0.085;//unit [m]
const float ry = 0.096;//unit [m]
float CarSpeedTarget[3] = {0.0, 0.0, 0.0};
float CarSpeedActual[3]={0.0, 0.0, 0.0};
extern Wheel wheelRB,wheelLB,wheelRF,wheelLF;
void Speed2Wheels(float Vx, float Vy, float omega_z)//�г�����Ϊx�����Ϊy
{
	wheelLF.fSpeedTarget = -(Vx - Vy - (rx + ry)*omega_z);//��Ϊ��ߵ��ֵĵ����ת�ǳ��ķ�����
	wheelRF.fSpeedTarget = Vx + Vy + (rx + ry)*omega_z;
	wheelLB.fSpeedTarget = -(Vx + Vy - (rx + ry)*omega_z);//��Ϊ��ߵ��ֵĵ����ת�ǳ��ķ�����
	wheelRB.fSpeedTarget = Vx - Vy + (rx + ry)*omega_z;
}

void Wheels2Speed(float* output)
{
	
	output[0]=(-wheelLB.fSpeedActual+wheelRB.fSpeedActual)/2;//Vx,ע�������������ֵ�ֵ��������һ�����ű仯������ٶȽ��㹫ʽ��һ��
	output[1]=(-wheelLB.fSpeedActual+wheelLF.fSpeedActual)/2;//Vy
	output[2]=(wheelRF.fSpeedActual+wheelLB.fSpeedActual)/(2*(rx+ry));//omega_z
	
}