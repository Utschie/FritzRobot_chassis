#include "imu.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "mpu6500_reg.h"
#include "spi.h"
#include "filters.h"
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

//volatile float        q0 = 1.0f;
//volatile float        q1 = 0.0f;
//volatile float        q2 = 0.0f;
//volatile float        q3 = 0.0f;
//volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az;  
//volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
static uint8_t        tx, rx;
static uint8_t        tx_buff[14] = { 0xff };
uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
mpu_data_t            mpu_data;
imu_t                 imu={0};
extern int static_flag;
/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
	  tx = reg & 0x7F;//& 0x7F 是使reg的第一位为0，0x7F的二进制表示是01111111，第一位为0意味着是写入
	  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//MPU6500的spi通信模式是首先发送一个寄存器地址，如果寄存器地址第一位为1则为read，如果为0，则为write，这是mpu6500的寄存器地址格式
    tx = data;
	  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//在上一步告知要进行写入操作后，把data发送过去。由于spi是全双工，因此写入的同时rx也读到了数据，当然这里不需要返回
    MPU_NSS_HIGH;
	return 0;//因为是写入，所以rx内的值不需要被返回，所以丢弃
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
	  MPU_NSS_LOW;//选中mpu对应的从机，开始通讯
	  tx = reg | 0x80;//| 0x80的意思是使reg的第一位取1，因为0x80的二进制是10000000，|是按位取或，第一位是1意味着是读取
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//首先告诉mpu6500要读取了，这里的参数1意味着一个字节
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//然后开始接收数据到rx里，这里tx是什么都无所谓，也可以是0xff，即空字节，
    MPU_NSS_HIGH;
    return rx;//读到的数据返回
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)//C语言返回多个值都是通过待填充的数据指针放入函数参数的方式
{
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    tx_buff[0] = tx;
	  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//首先传入一个地址字节,所有寄存器的地址都是一个字节，在mpu_6500_reg.h里用16进制表示是4个字符
	  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);//这里的len就是表示我要读取多少个字节的数据
    MPU_NSS_HIGH;
    return 0;
}


/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{
	  mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);//从寄存器地址MPU6500_ACCEL_XOUT_H到MPU6500_GYRO_ZOUT_L，是刚好连续的14个字节。数据是按着地址连续排列的，告诉spi某个地址，其实就是在告诉它从哪个地址开始往下读

	  mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];//mpu_data.ax是一个16位的整型，mpu_buff是一个长度为14的8位整型数组，这个公式是将高八位和第八位合成一个16为，形式为“s =  (high << 8) | low;”。
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];

		mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
	

	  imu.ax   = mpu_data.ax / 8192.0; //因为量程是4g
    imu.ay   = mpu_data.ay / 8192.0; 
    imu.az   = mpu_data.az / 8192.0;
	
	  imu.wx   = mpu_data.gx / (131.068f*57.29578f); //量程设置为250度每秒，量程越小精度越高
    imu.wy   = mpu_data.gy / (131.068f*57.29578f); 
    imu.wz   = mpu_data.gz / (131.068f*57.29578f);
	  StaticFilter();
		if (static_flag==1)
		{
			imu.wz=0.0;
		}
	  
}


/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); //左移3位是因为MPU6500_ACCEL_CONFIG寄存器负责调量程的位是第4:3位，（寄存器的第0位一律从右边开始数）
}

uint8_t id;

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t mpu_device_init(void)
{
	MPU_DELAY(100);

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x02 },         /* 0x04的二进制第1位为1所以LPF 92Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x00 },    /* +-250dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x08 },   /* +-4G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  92hz */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}

	//mpu_set_gyro_fsr(0); //	这两个函数好像不太好使，所以直接在设置里设置了
	//mpu_set_accel_fsr(1);// 这两个函数好像不太好使

	mpu_offset_call();
	StaticFilter_Init();
	
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);//这里其实就是把静止情况下的设为offset

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gy_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}


