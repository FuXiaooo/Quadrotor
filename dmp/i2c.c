
#include "i2c.h"
//mpu6050����	 





//Ӧ���־λ
bit ack;
unsigned char recc,BUF[14];

//��ʱ����
//void delay_us(unsigned char us)
//{
//	int j1,j2;
//	for( j1=0;j1<us;j1++)
//	{
//	  for(j2=0;j2<30;j2++);
//	}
//}






/***************  �������� ******************/

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


/*****************����I2c*******************/

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


/**********�ֽڷ��ͺ���**********/

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

/*****************�ֽڽ��պ���***********/

unsigned char   rec_bite(void)
{
	unsigned char unbit,recc;
	recc=0;
	SDA=1;
	for(unbit=0;unbit<8;unbit++)
	{
		    recc <<= 1;
        SCL = 1;                //����ʱ����
        NOP30();
	      NOP30();
        NOP10();
				recc |= SDA;             //������
        SCL = 0;                //����ʱ����
				NOP30();
	      NOP30();
        NOP10();
   }
	// P12=0;
	 return recc;
}
/****************Ӧ���Ӻ���***************/
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

///****************��Ӧ���Ӻ���***************/
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
///*************�ȴ�Ӧ����*****************/
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
/**************���ֽڷ���*******************/
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
/*************���ֽڽ���*********************/
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
		BUF[i] = rec_bite();       //�����Ĵ�������
    Ack_I2c();
	}
		BUF[i] = rec_bite();	//���һ���ֽ�
		UnAck_I2c();
		stop_I2c();                    //ֹͣ�ź�
}
/*��ȡ8λ�Ĵ�������ֽ� 
 * @���� addr    I2C�������ڲ���ַ 
 * @���� length  д�����ݳ���     
 * @���� Data    �������ݵĵ�ַ       
 * @����ֵ ����״̬ (1=�ɹ�) 
 */  
void  recs_I2c(unsigned char addr,unsigned char length,unsigned char *Data)  
{  
        unsigned char i;  
        start_I2c();    //��ʼ�ź�  
        send_bite(SlaveAddressW);    //�豸��ַ+д�ź�  
        Wake_I2c();
        send_bite(addr);    //�豸�ڲ���ַ  
        Wake_I2c(); 
        start_I2c();    //��ʼ�ź�  
        send_bite(SlaveAddressR);  //�豸��ַ+���ź�  
        Wake_I2c();  
        for(i=0;i<length-1;i++)  
        {  
            Data[i]= rec_bite();
             Ack_I2c();					//��ȡ����,����ACK  
        }  
        Data[length] = rec_bite();//��ȡ����,����NAK  
				UnAck_I2c();
        stop_I2c();    //ֹͣ�ź�  
}  
/**д��8λ�Ĵ�����һ��λ�� 
 * @���� addr    I2C�������ڲ���ַ 
 * @���� bitNum  д��ı���λ(0-7)      
 * @���� data    д������       
 * @����ֵ ����״̬ (1=�ɹ�) 
 */  
void  writeBit(unsigned char addr,unsigned char bitNum,unsigned char Data)  
{  
    unsigned char b;  
	  recs_I2c(addr,1,&b); 
    b = (Data != 0) ? (b | (1 << bitNum)):(b & ~(1 << bitNum));   
    send_I2c(addr,b);    //д������  
 
}  
/**д��8λ�Ĵ����Ķ��λ��   
 * @���� addr     I2C�������ڲ���ַ 
 * @���� bitStart ��һλ��д��λ�ã�0-7�� 
 * @���� length   д�ı�����(������8) 
 * @���� Data     д������ 
 * @����ֵ ����״̬ (1=�ɹ�) 
 */  
void writeBits(unsigned char addr,unsigned char bitStart,unsigned char length,unsigned char Data)  
{   
    unsigned char bb,masks=0;  
        recs_I2c(addr,1,&bb);
        masks = (((1<<length) - 1) << (bitStart-length + 1));    //����  
        Data <<=(bitStart - length + 1);  //��д��������ƶ���λ  
        Data &= masks;  
        bb &= ~(masks);  
        bb |= Data;  
        send_I2c(addr,bb);   //д������  
        
}  
/**��ȡһ��λ��8λ�����ļĴ�����     */  
void readBit(unsigned char addr,unsigned char bitNum,unsigned char *Data)  
{  
    unsigned char bbb;  
    recs_I2c(addr,1,&bbb) ;
    *Data = bbb & (1 << bitNum);  
}  
/**��ȡ8λ�Ĵ����Ķ��λ��  
* @���� addr    I2C�������ڲ���ַ  
* @���� bitStart��һλ��λ�ö�ȡ��0-7��  
* @���� length  λ��ȡ@������������������8��  
* @���� *data   ���ݴ洢��ַ����'101'�κ�bitStartλ�ö�ȡ������0X05��  
* @����ֵ��1=�ɹ���  
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
  
/***********����MPU6050******************/
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
//    writeBits (0x6B,2,3,0x01);  //��Դ����  
//    writeBits (0x1B,4,2,0x00);  //�������������� 250/s  
//    writeBits (0x1C,4,2,0x00);  //���ü��ٶ����� 2G  
//    writeBit (0x6B,6,1);    //��Դ����MUP����˯��ģʽ  
//}  
/**********��ȡMPU6050����****************/
void read_MPU6050(void)
{
	rec_I2c(ACCEL_XOUT_H,BUF);

 }