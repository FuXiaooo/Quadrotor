
#include "i2c.h"
//mpu6050定义	 





//应答标志位
bit ack;
unsigned char recc,BUF[14];

//延时函数
//void delay_us(unsigned char us)
//{
//	int j1,j2;
//	for( j1=0;j1<us;j1++)
//	{
//	  for(j2=0;j2<30;j2++);
//	}
//}






/***************  启动总线 ******************/

void start_I2c()
{
	SDA=1;
	SCL=1;
  NOP30();
	NOP30();
  NOP30();
	NOP30();
	SDA=0;
	NOP30();
	NOP30();
  NOP30();
	NOP30();
	SCL=0;
	NOP30();
	NOP30();
  NOP30();
	NOP30();

}


/*****************结束I2c*******************/

void stop_I2c()
{
	SDA=0;
	SCL=1;
	NOP30();
	NOP30();
  NOP30();
	NOP30();
	SDA=1;
	NOP30();
	NOP30();
  NOP30();
	NOP30();

}


/**********字节发送函数**********/

void send_bite(unsigned char c)
{
	unsigned char  unbit;
	for(unbit=0;unbit<8;unbit++)
	{
		c <<= 1;
		SDA = CY; 
		SCL=1;
		NOP30();
	  NOP30();
    NOP10();
		SCL=0;
		NOP30();
	  NOP30();
    NOP10();
   }
}

/*****************字节接收函数***********/

unsigned char   rec_bite(void)
{
	unsigned char unbit,recc;
	recc=0;
	SDA=1;
	for(unbit=0;unbit<8;unbit++)
	{
		    recc <<= 1;
        SCL = 1;                //拉高时钟线
        NOP30();
	      NOP30();
        NOP10();
				recc |= SDA;             //读数据
        SCL = 0;                //拉低时钟线
				NOP30();
	      NOP30();
        NOP10();
   }
	// P12=0;
	 return recc;
}
/****************应答子函数***************/
void Ack_I2c(void )
{
	SDA=0;
	SCL=1;
	NOP30();
	NOP30();
	NOP10();
	SCL=0;
	NOP30();
	NOP30();
	NOP10();
}

///****************非应答子函数***************/
void UnAck_I2c(void )
{
	SDA=1;
	SCL=1;
	NOP30();
	NOP30();
	NOP10();
	SCL=0;
	NOP30();
	NOP30();
	NOP10();
}
///*************等待应答结果*****************/
void Wake_I2c(void )
{
	SDA=1;
	SCL=1;
  NOP30();
	NOP30();
	NOP10();
  ack=SDA;
	SCL=0;
	NOP30();
	NOP30();
	NOP10();

}
/**************多字节发送*******************/
void send_I2c(unsigned char cadress,unsigned char date)
{
	start_I2c();
	send_bite(SlaveAddressW);
 Wake_I2c();
	send_bite(cadress);
  Wake_I2c();
	send_bite(date);
  Wake_I2c();

				 
		stop_I2c();

  

}
/*************多字节接收*********************/
void rec_I2c(unsigned char cadress,unsigned char *BUF)
{
	unsigned char i;
	  start_I2c();
	  send_bite(SlaveAddressW);
	  Wake_I2c();
	  send_bite(cadress);
	Wake_I2c();
    start_I2c();
		send_bite(SlaveAddressR);
	Wake_I2c();
		for(i=0; i<13; i++)
	{
		BUF[i] = rec_bite();       //读出寄存器数据
    Ack_I2c();
	}
		BUF[i] = rec_bite();	//最后一个字节
		UnAck_I2c();
		stop_I2c();                    //停止信号
}
/*读取8位寄存器多个字节 
 * @参数 addr    I2C从器件内部地址 
 * @参数 length  写入数据长度     
 * @参数 Data    保存数据的地址       
 * @返回值 返回状态 (1=成功) 
 */  
void  recs_I2c(unsigned char addr,unsigned char length,unsigned char *Data)  
{  
        unsigned char i;  
        start_I2c();    //起始信号  
        send_bite(SlaveAddressW);    //设备地址+写信号  
        Wake_I2c();
        send_bite(addr);    //设备内部地址  
        Wake_I2c(); 
        start_I2c();    //起始信号  
        send_bite(SlaveAddressR);  //设备地址+读信号  
        Wake_I2c();  
        for(i=0;i<length-1;i++)  
        {  
            Data[i]= rec_bite();
             Ack_I2c();					//读取数据,发送ACK  
        }  
        Data[length] = rec_bite();//读取数据,发送NAK  
				UnAck_I2c();
        stop_I2c();    //停止信号  
}  
/**写入8位寄存器的一个位。 
 * @参数 addr    I2C从器件内部地址 
 * @参数 bitNum  写入的比特位(0-7)      
 * @参数 data    写入数据       
 * @返回值 返回状态 (1=成功) 
 */  
void  writeBit(unsigned char addr,unsigned char bitNum,unsigned char Data)  
{  
    unsigned char b;  
	  recs_I2c(addr,1,&b); 
    b = (Data != 0) ? (b | (1 << bitNum)):(b & ~(1 << bitNum));   
    send_I2c(addr,b);    //写入数据  
 
}  
/**写入8位寄存器的多个位。   
 * @参数 addr     I2C从器件内部地址 
 * @参数 bitStart 第一位的写入位置（0-7） 
 * @参数 length   写的比特数(不超过8) 
 * @参数 Data     写入数据 
 * @返回值 返回状态 (1=成功) 
 */  
void writeBits(unsigned char addr,unsigned char bitStart,unsigned char length,unsigned char Data)  
{   
    unsigned char bb,masks=0;  
        recs_I2c(addr,1,&bb);
        masks = (((1<<length) - 1) << (bitStart-length + 1));    //掩码  
        Data <<=(bitStart - length + 1);  //把写入的数据移动到位  
        Data &= masks;  
        bb &= ~(masks);  
        bb |= Data;  
        send_I2c(addr,bb);   //写入数据  
        
}  
/**读取一个位从8位器件的寄存器。     */  
void readBit(unsigned char addr,unsigned char bitNum,unsigned char *Data)  
{  
    unsigned char bbb;  
    recs_I2c(addr,1,&bbb) ;
    *Data = bbb & (1 << bitNum);  
}  
/**读取8位寄存器的多个位。  
* @参数 addr    I2C从器件内部地址  
* @参数 bitStart第一位的位置读取（0-7）  
* @参数 length  位读取@参数长度数（不超过8）  
* @参数 *data   数据存储地址（即'101'任何bitStart位置读取将等于0X05）  
* @返回值（1=成功）  
*/  
void readBits(unsigned char addr,unsigned char bitStart,unsigned char length,unsigned char *Data)  
{  
    unsigned char c,maskss=0;  
    recs_I2c(addr,1,&c)   ;  
    maskss = ((1 << length) - 1) << (bitStart - length + 1);  
    c &= maskss;  
    c >>= (bitStart - length + 1);  
    *Data = c;  
}  
  
/***********启动MPU6050******************/
void init_MPU6050(void)
{
	send_I2c(PWR_MGMT_1,0x00);
	send_I2c(SMPLRT_DIV,0x07);
	send_I2c(CONFIG,0x04);
	send_I2c(GYRO_CONFIG,0x08);             ////////0x18                       500
	send_I2c(ACCEL_CONFIG,0x08);              //////0x01                        
}
//void init_MPU6050(void)  
//{  
//    writeBits (0x6B,2,3,0x01);  //电源管理  
//    writeBits (0x1B,4,2,0x00);  //设置陀螺仪量程 250/s  
//    writeBits (0x1C,4,2,0x00);  //设置加速度量程 2G  
//    writeBit (0x6B,6,1);    //电源管理MUP进入睡眠模式  
//}  
/**********读取MPU6050数据****************/
void read_MPU6050(void)
{
	rec_I2c(ACCEL_XOUT_H,BUF);

 }