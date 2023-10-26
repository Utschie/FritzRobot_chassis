#include "imu.h"
#include "filters.h"
#include <math.h>
extern imu_t imu;
int static_flag_x,static_flag_y,static_flag_z;//如果检测到传感器处于静止则为1，其他则为0

void StaticFilter_Init(void)
{
	static_flag_x=0;
	static_flag_y=0;
	static_flag_z=0;
	int i;
	float sum_z=0.0;
	float sum_x=0.0;
	float sum_y=0.0;
	float var_z;
	float var_x;
	float var_y;
	int len;
	len =  STATIC_PERIOD/5;//每5毫秒读一次
	float buf_z[len];
	float buf_x[len];
	float buf_y[len];
	for (i=0;i<len;i++)
	{
		mpu_get_data();
		buf_z[i]=imu.wz;
		buf_x[i]=imu.wx;
		buf_y[i]=imu.wy;
		sum_z+=buf_z[i];
		sum_x+=buf_x[i];
		sum_y+=buf_y[i];
		MPU_DELAY(5);//每5毫秒读一次
	}
	imu.gyroz_mean=sum_z/len;
	imu.gyrox_mean=sum_x/len;
	imu.gyroy_mean=sum_y/len;
	for (int j = 0;j<len;j++)
	{
		var_z+=pow(buf_z[j]-imu.gyroz_mean,2)/len;
		var_x+=pow(buf_x[j]-imu.gyrox_mean,2)/len;
		var_y+=pow(buf_y[j]-imu.gyroy_mean,2)/len;
	}
	imu.gyroz_std=pow(var_z,0.5);
	imu.gyrox_std=pow(var_x,0.5);
	imu.gyroy_std=pow(var_y,0.5);
}

void StaticFilter_z(void)
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
		static_flag_z=1;
	}else
	{
		static_flag_z=0;
	}
}
void StaticFilter_x(void)
{
	int len=STATIC_PERIOD/5;
	float mean_=imu.gyrox_mean;
	float var_ = pow(imu.gyrox_std,2);
	float var;
	imu.gyrox_mean=mean_+(imu.wx-mean_)/len;//update mean
	var = (len-1)/pow(len,2)*pow((imu.wx-mean_),2)+(len-1)/len*var_;
	imu.gyrox_std = pow(var,0.5);//update std
	if (imu.gyrox_std<STATIC_THRESHHOLD)
	{
		static_flag_x=1;
	}else
	{
		static_flag_x=0;
	}
}
void StaticFilter_y(void)
{
	int len=STATIC_PERIOD/5;
	float mean_=imu.gyroy_mean;
	float var_ = pow(imu.gyroy_std,2);
	float var;
	imu.gyroy_mean=mean_+(imu.wy-mean_)/len;//update mean
	var = (len-1)/pow(len,2)*pow((imu.wy-mean_),2)+(len-1)/len*var_;
	imu.gyroy_std = pow(var,0.5);//update std
	if (imu.gyroy_std<STATIC_THRESHHOLD)
	{
		static_flag_y=1;
	}else
	{
		static_flag_y=0;
	}
}