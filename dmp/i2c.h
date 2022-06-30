#include "STC15Fxxxx.H"
//mpu6050定义	 

#define SCL   P35			//6050IIC时钟引脚定义     P00
#define SDA   P34			//6050IIC数据引脚定义      P46
#define AD0   P32  //p32 定义mpu6050的数字io


#define PWR_MGMT_1 0x6b
#define SMPLRT_DIV 0x19
#define CONFIG 0x1a
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c
#define SlaveAddressW 0xd0
#define SlaveAddressR 0xd1
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48



extern bit ack;
extern unsigned char BUF[14];
void start_I2c(void );
void stop_I2c(void );
void send_bite(unsigned char c);
void delay_us(unsigned char us);
unsigned char rec_bit(void);
void Ack_I2c(void );
//void UnAck_I2c(void );
void Wake_I2c(void );
void send_I2c(unsigned char cadress,unsigned char date);
void  rec_I2c(unsigned char cadress,unsigned char *BUF);
void  recs_I2c(unsigned char addr,unsigned char length,unsigned char *Data); 
void  writeBit(unsigned char addr,unsigned char bitNum,unsigned char Data) ;
void writeBits(unsigned char addr,unsigned char bitStart,unsigned char length,unsigned char Data)  ;
void readBit(unsigned char addr,unsigned char bitNum,unsigned char *Data) ;
void readBits(unsigned char addr,unsigned char bitStart,unsigned char length,unsigned char *Data)  ;
void init_MPU6050(void);
void read_MPU6050();