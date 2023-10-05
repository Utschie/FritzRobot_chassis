#include "imu.h"
#include "filters.h"
#include <math.h>
extern imu_t imu;
int static_flag;//如果检测到传感器处于静止则为1，其他则为0

void StaticFilter_Init(void)
{
	int i;
	float sum=0.0;
	float var;
	int len;
	len =  STATIC_PERIOD/5;//每5毫秒读一次
	float buf[len];
	for (i=0;i<len;i++)
	{
		mpu_get_data();
		buf[i]=imu.wz;
		sum+=buf[i];
		MPU_DELAY(5);//每5毫秒读一次
	}
	imu.gyroz_mean=sum/len;
	for (int j = 0;j<len;j++)
	{
		var+=pow(buf[j]-imu.gyroz_mean,2)/len;
	}
	imu.gyroz_std=pow(var,0.5);
}

void StaticFilter(void)
{
	int len=STATIC_PERIOD/5;
	float mean_=imu.gyroz_mean;
	float var_ = pow(imu.gyroz_std,2);
	float var;
	imu.gyroz_mean=mean_+(imu.wz-mean_)/len;//update mean
	var = (len-1)/pow(len,2)*pow((imu.wz-mean_),2)+(len-1)/len*var_;
	imu.gyroz_std = pow(var,0.5);//update std
	if (imu.gyroz_std<STATIC_THRESHHOLD)
	{
		static_flag=1;
	}else
	{
		static_flag=0;
	}
}