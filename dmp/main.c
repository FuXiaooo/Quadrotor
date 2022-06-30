

#define MAIN_Fosc		30000000L	//������ʱ��
#define S2RI 0x1  //����2�Ľ��յ���־
#define S2TI 0x2  //����2�ķ������־

#define	EAXSFR()		P_SW2 |=  0x80	/* MOVX A,@DPTR/MOVX @DPTR,Aָ��Ĳ�������Ϊ��չSFR(XSFR) */
#define	UART_8bit_BRTx	(1<<6)	//8λ����,�ɱ䲨����
#define	UART1_SW_P30_P31	0
#define	BRT_Timer2	2




#include <stdio.h>
#include <math.h>
#include <intrins.h>
#include	"STC15Fxxxx.H"
#include <i2c.h>
#include <DataScope_DP.h> 
#include <IMU.h>


unsigned char Send_Count,ii; 
code float a[180]={0,0.017,0.035,0.052,0.070,0.087,0.105,0.122,0.139,0.156,0.174,0.191,0.208,0.225,0.242,0.259,
	                 0.276,0.292,0.309,0.326,0.342,0.358,0.375,0.391,0.407,0.423,0.438,0.454,0.469,0.485,0.5,
                   0.515,0.530,0.545,0.559,0.574,0.588,0.602,0.616,0.629,0.643,0.656,0.669,0.682,0.695,0.707,
                   0.719,0.731,0.743,0.755,0.766,0.777,0.788,0.799,0.809,0.819,0.829,0.839,0.848,0.857,0.866,
                   0.874,0.883,0.891,0.899,0.906,0.914,0.921,0.927,0.934,0.940,0.946,0.951,0.956,0.961,0.970,
                   0.970,0.974,0.978,0.981,0.985,0.988,0.990,0.993,0.995,0.996,0.998,0.999,0.999,1,1};





//u8	xdata	RxBuf[20]={0};
//u8 SW2_tmp,SSBU1,SSBU2,FLAG1,n,n1,n2,SSBU;
u8 up,ex_yaw,ex_pitch,ex_roll,key,keyt;//��Ф��������
bit begin;
//u8 yyhg,fggd,ZS1,ZS2,rc_pitch,rc_roll,rc_yaw,rc_gyro_x,rc_gyro_y,rc_gyro_z;	//���������������	
u8 TH0_TEMP,TL0_TEMP,temp;	
//u16 dj3,dj2,dj1,dj4;
u16 i,i1,i2,i1f,TTT3,TTT1,TTT2;
u32 TIM_Value,j1;

//float i1;
float ax,ay,az,gx,gy,gz;

void main()
{		begin=1;
	  pitch0=0;
		roll0=0;
		yaw0=0;
	  ax=0;
	  ay=0;
	  az=0;
	  gx=0;
	  gy=0;
	  gz=0;
	 for(i=0;i<2000;i++);
 		//����1��ʼ�� ������38400  T2��Ϊ�����ʷ�����
		j1 = (MAIN_Fosc / 4) /38400;	//��1T����
		j1= 65536UL - j1;
				AUXR &= ~(1<<4);	//Timer2 stop		
		AUXR &= ~(1<<3);	//Timer2 set As Timer
		AUXR |=  (1<<2);	//Timer2 set as 1T mode
		T2H = (u8)(j1>>8);
		T2L = (u8)j1;
		IE2  &= ~(1<<2);	//��ֹ�ж�
		AUXR &= ~(1<<3);	//��ʱ
		AUXR |=  (1<<4);	//Timer2 run enable
	
		SCON = (SCON & 0x3f) | UART_8bit_BRTx;
    AUXR |= 0x01;		//S1 BRT Use Timer2;	
    REN = 1;	//�������
    P_SW1 = (P_SW1 & 0x3f) | (UART1_SW_P30_P31 & 0xc0);	//�л�IO
   CLK_DIV &= ~(1<<4);//����1����������ʽ
	 
	 	//����2��ʼ�� ������38400
	// AUXR= 0X14;	//����2��ʼ�� ������38400Ҫ�ĳ�����
	 S2CON= 0X10;
	// T2L=(65536-(MAIN_Fosc/4/38400));//���ö�ʱ��2��
	 // T2H=(65536-(MAIN_Fosc/4/38400))>>8;
	 
	    SBUF=0x01;while(1){if(TI==1){TI=0;TI=TI;break;}}
		  SBUF=0x02;while(1){if(TI==1){TI=0;TI=TI;break;}}
		  SBUF=0xfA;while(1){if(TI==1){TI=0;TI=TI;break;}}
			

								
			//T0 ��ʱ����ʼ��
				TR0 = 0;		//ֹͣ����
				TMOD &= ~0x03;	//����ģʽ,0: 16λ�Զ���װ, 1: 16λ��ʱ/����, 2: 8λ�Զ���װ, 3: 16λ�Զ���װ, ���������ж�
        AUXR |=  0x80;	//1T
		    TMOD &= ~0x04;	//��ʱ
	
		    INT_CLKO &= ~0x01;	//T0�����ʱ�� T0��ʱ����ʱ
				TIM_Value= 65536UL -60000;		//��ֵ
		TH0_TEMP = (u8)(TIM_Value >> 8);
		TL0_TEMP = (u8)TIM_Value;
		TH0=TH0_TEMP;
		TL0=TL0_TEMP;
		TR0 = 1;	//��ʼ����	
		//ET0 = 1;	//�����ж�
		EA = 1;	//�����ж�
		ES=0;
	
		i=0;
//		i1=0;i1f=0;i1p=0; i2=0,i3=0;i4=0,i5=0,i6=0,i7=0,i8=0;
//		RxBuf[0]=0;	RxBuf[1]=0;	RxBuf[2]=0;	RxBuf[3]=0;
		
		
		
		//��ʼ��6050
		init_MPU6050();
//     dmpInitialize(); //���ز������˶���                
//    writeBit(0x6A,2,1); //��λ FIFO  
//    writeBit(0x6A,7,1); //ʹ��DMP  
 for(i=0;i<60000;i++);
 for(i=0;i<60000;i++);


    i2=0;
		
		
		 while(1)
    {     
             		if(TF0)
       	   	{ TF0=0;
			     	
   				  TTT1=TTT1+1;
					  TTT2=TTT2+1;
					  TTT3=TTT3+1;
				
       }
				
              switch (i2)
			      {  case 0:TTT1=0;i2=10;break;
			        case  10:if(TTT1>4)i2=30;							
							break;
			        case 30:  
							  smoothing1();
								dealdata();						  	
								IMUupdate(Angle_gx,Angle_gy,Angle_gz,Angle_ax,Angle_ay,Angle_az);
							  DataScope_Get_Channel_Data(pitch, 1 );
							  DataScope_Get_Channel_Data(roll, 2 );
						  	DataScope_Get_Channel_Data(yaw, 3 );
							  Send_Count = DataScope_Data_Generate(3);
							 
           //     if(begin){pitch0=pitch;roll0=roll;yaw0=yaw;begin=0;}							
							 for( ii = 0 ; ii < Send_Count; ii++) 
		               {
		 
	                      	SBUF = DataScope_OutPut_Buffer[ii]; 
			                  while(1){if(TI==1){TI=0;TI=TI;break;}}
	                  	}
							      i2=40;
							break;
							case 40:i2=0;break;
						}
		
											
//						switch(i1)//����1 ������MPU6050 ʵ����̬
//						{		
//							
//							 case 0:if(RI)
//				              {		SSBU=SBUF;RI=0;if (SSBU==0XFB)i1=15;else i1=0;break;}	break;								      
//								case 15:if(RI)//������pitch
//				               {	rc_pitch=SBUF;RI=0;i1=20;break;}	break;								      
//								case 20:if(RI)//���roll
//				               {rc_roll=SBUF;RI=0;i1=25;break;}break;
//								case 25:if(RI)//ƫ��yaw
//				               {rc_yaw=SBUF;RI=0;i1=30;break;}break;
//								case 30:if(RI)//��x����ٶ�
//				               {rc_gyro_x=SBUF;RI=0;i1=35;break;}break;
//							  case 35:if(RI)//��y����ٶ�
//				               {rc_gyro_y=SBUF;RI=0;i1=40;break;}break;
//					 		 case 40:if(RI)//��z����ٶ�
//				               {rc_gyro_z=SBUF;RI=0;i1=45;break;}break;		 
//								case 45:if(RI)
//				               {SSBU=SBUF;RI=0;if(SSBU==0XFE){i1=0;}else i1=0;break;}break;
//							
//										 }
	
						
//					 switch(i2)//����2 P1.0 P1.1  ��ң�ظ�����̬������
//	        {	
//	   	          case 0:if(S2CON & S2RI)
//				              {	S2CON&=~S2RI;	SSBU2=S2BUF;if (SSBU2==0XFB)i2=15;else i2=0;break;}	break;								      
//								case 15:if(S2CON & S2RI)//up����
//				               {S2CON&=~S2RI;	up=S2BUF;i2=20;break;}	break;								      
//								case 20:if(S2CON & S2RI)//ex_yaw ƫ��
//				               {S2CON&=~S2RI;ex_yaw=S2BUF;i2=25;break;}break;
//								case 25:if(S2CON & S2RI)//ex_pitch ǰ����
//				               {S2CON&=~S2RI;ex_pitch=S2BUF;i2=30;break;}break;
//								case 30:if(S2CON & S2RI)//ex_roll ���
//				               {S2CON&=~S2RI;ex_roll=S2BUF;i2=35;break;}break;
//							  case 35:if(S2CON & S2RI)//��
//				               {S2CON&=~S2RI;key=S2BUF;i2=40;break;}break;
//					 		 case 40:if(S2CON & S2RI)//��������
//				               {S2CON&=~S2RI;keyt=S2BUF;i2=45;break;}break;		 
//								case 45:if(S2CON & S2RI)
//				               {S2CON&=~S2RI;SSBU2=S2BUF;if(SSBU2==0XFE){i2=0;}else i2=0;break;}break;
//								
//					
//							}
					
         //      read_MPU6050();
					//			dealdata();
					//			IMUupdate(Angle_ax,Angle_ay,Angle_az,Angle_gx,Angle_gy,Angle_gz);
		           //   P12=1;
							 
//			

 
  




 //compute ();



		//	i1=i1+0.1;
//if(i1>3.14) i1=-3.14;
              //  if((BUF[8]*256+BUF[9])>=32768) aa=(float)((BUF[8]*256+BUF[9])-65536)/16.5;
            //    else  aa= (float)(BUF[8]*256+BUF[9])/16.5;
            //      DataScope_Get_Channel_Data(pitch, 1 );
//DataScope_Get_Channel_Data(100*tan(a), 2 );
//DataScope_Get_Channel_Data( 100*cos(a), 3 );
//DataScope_Get_Channel_Data( 100*a , 4 );
//DataScope_Get_Channel_Data(0, 5 );
//DataScope_Get_Channel_Data(0 , 6 );
//DataScope_Get_Channel_Data(0, 7 );
//DataScope_Get_Channel_Data( 0, 8 );
//DataScope_Get_Channel_Data(0, 9 );
//DataScope_Get_Channel_Data( 0 , 10);
//Send_Count = DataScope_Data_Generate(1);
//							 
//							 for( ii = 0 ; ii < Send_Count; ii++) 
//		{
//		 
//		SBUF = DataScope_OutPut_Buffer[ii]; 
//			while(1){if(TI==1){TI=0;TI=TI;break;}}
//		}
		//		for(i=0;i<3000;i++);	
	//	for(i=0;i<30000;i++);	
	
//char(i1f)//����1 ���� ��PC�����Ӷ�MPU6050������
//						{		
//            case 0:TTTf1=0;i1f=10;break;
//								case 10:if(TTTf1>20) {i1f=11;}break;
//								//��ʾ��Ф�ĸ������� ң������������
//								case 11:SBUF=0XFB;i1f=13;break;// 1
//							  
//							 case 13: if(TI){TI=0;i1f=15;}break;
//								
//								case 15:SBUF=up;i1f=17;break;// 2
//							  
//							 case 17: if(TI){TI=0;i1f=20;}else i1f=17;break;
//							 
//							 
//						  case 20:SBUF=ex_yaw;i1f=30;break;//3
//							 
//							 case 30:if(TI){TI=0;i1f=40;}else i1f=30;break;
//							 
//							 case 40:SBUF=ex_pitch;i1f=50;break;//4
//							  case 50: if(TI){TI=0;i1f=60;}break;
//									 case 60:SBUF=ex_roll;i1f=70;break;//5
//							  case 70: if(TI){TI=0;i1f=80;}break;
//							 
//							 case 80:SBUF=key;i1f=90;break;//6
//							  case 90: if(TI){TI=0;i1f=100;}break;
//									 case 100:SBUF=keyt;i1f=110;break;//7
//							  case 110: if(TI){TI=0;i1f=150;n=0;}break;
//								
//									//��ʾ������������� 6050���� rc_roll,rc_yaw,rc_gyro_x,rc_gyro_y,rc_gyro_z
//								case 150:SBUF=BUF[8];i1f=180;break;//8-1
//							  
//							 case 180: if(TI){TI=0;i1f=200;}break;
//								
//									case 200:SBUF=BUF[9];i1f=217;break;//8-1
//							  
//							 case 217: if(TI){TI=0;i1f=220;}break;
//						  
//							 case 220:SBUF=BUF[10];i1f=230;break;			//9-2			 
//							 case 230:if(TI){TI=0;i1f=240;}break;
//							 
//						 case 240:SBUF=BUF[11];i1f=250;break;//10-3
//						  case 250: if(TI){TI=0;i1f=260;}break;
//								 case 260:SBUF=BUF[12];i1f=270;break;//11-4
//							  case 270: if(TI){TI=0;i1f=280;}break;
//							 
//							 case 280:SBUF=BUF[13];i1f=290;break;//12-5
//							  case 290: if(TI){TI=0;i1f=357;}break;
//								//������								
//								case 357:SBUF=0xFE;i1f=360;break;
//							  case 360: if(TI){TI=0;i1f=0;n=0;}break;
//								
//									
//																			
//						}			
//							
							
	

}  //for wile
	
}  // for main

