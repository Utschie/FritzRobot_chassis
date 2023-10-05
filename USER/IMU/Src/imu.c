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
	  tx = reg & 0x7F;//& 0x7F ��ʹreg�ĵ�һλΪ0��0x7F�Ķ����Ʊ�ʾ��01111111����һλΪ0��ζ����д��
	  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//MPU6500��spiͨ��ģʽ�����ȷ���һ���Ĵ�����ַ������Ĵ�����ַ��һλΪ1��Ϊread�����Ϊ0����Ϊwrite������mpu6500�ļĴ�����ַ��ʽ
    tx = data;
	  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//����һ����֪Ҫ����д������󣬰�data���͹�ȥ������spi��ȫ˫�������д���ͬʱrxҲ���������ݣ���Ȼ���ﲻ��Ҫ����
    MPU_NSS_HIGH;
	return 0;//��Ϊ��д�룬����rx�ڵ�ֵ����Ҫ�����أ����Զ���
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
	  MPU_NSS_LOW;//ѡ��mpu��Ӧ�Ĵӻ�����ʼͨѶ
	  tx = reg | 0x80;//| 0x80����˼��ʹreg�ĵ�һλȡ1����Ϊ0x80�Ķ�������10000000��|�ǰ�λȡ�򣬵�һλ��1��ζ���Ƕ�ȡ
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//���ȸ���mpu6500Ҫ��ȡ�ˣ�����Ĳ���1��ζ��һ���ֽ�
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//Ȼ��ʼ�������ݵ�rx�����tx��ʲô������ν��Ҳ������0xff�������ֽڣ�
    MPU_NSS_HIGH;
    return rx;//���������ݷ���
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)//C���Է��ض��ֵ����ͨ������������ָ����뺯�������ķ�ʽ
{
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    tx_buff[0] = tx;
	  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);//���ȴ���һ����ַ�ֽ�,���мĴ����ĵ�ַ����һ���ֽڣ���mpu_6500_reg.h����16���Ʊ�ʾ��4���ַ�
	  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);//�����len���Ǳ�ʾ��Ҫ��ȡ���ٸ��ֽڵ�����
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
	  mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);//�ӼĴ�����ַMPU6500_ACCEL_XOUT_H��MPU6500_GYRO_ZOUT_L���Ǹպ�������14���ֽڡ������ǰ��ŵ�ַ�������еģ�����spiĳ����ַ����ʵ�����ڸ��������ĸ���ַ��ʼ���¶�

	  mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];//mpu_data.ax��һ��16λ�����ͣ�mpu_buff��һ������Ϊ14��8λ�������飬�����ʽ�ǽ��߰�λ�͵ڰ�λ�ϳ�һ��16Ϊ����ʽΪ��s =  (high << 8) | low;����
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];

		mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
	

	  imu.ax   = mpu_data.ax / 8192.0; //��Ϊ������4g
    imu.ay   = mpu_data.ay / 8192.0; 
    imu.az   = mpu_data.az / 8192.0;
	
	  imu.wx   = mpu_data.gx / (131.068f*57.29578f); //��������Ϊ250��ÿ�룬����ԽС����Խ��
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
  * @param  fsr: range(0,��250dps;1,��500dps;2,��1000dps;3,��2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,��2g;1,��4g;2,��8g;3,��16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); //����3λ����ΪMPU6500_ACCEL_CONFIG�Ĵ�����������̵�λ�ǵ�4:3λ�����Ĵ����ĵ�0λһ�ɴ��ұ߿�ʼ����
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
																			{ MPU6500_CONFIG, 0x02 },         /* 0x04�Ķ����Ƶ�1λΪ1����LPF 92Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x00 },    /* +-250dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x08 },   /* +-4G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  92hz */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}

	//mpu_set_gyro_fsr(0); //	��������������̫��ʹ������ֱ����������������
	//mpu_set_accel_fsr(1);// ��������������̫��ʹ

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
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);//������ʵ���ǰѾ�ֹ����µ���Ϊoffset

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


