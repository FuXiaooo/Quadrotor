#include <IMU.h>
#include <math.h>




#define pi 3.14159265f                           
#define Kp 10.0f                        /////0.8f
#define Ki 0.001f                         
#define halfT 0.004f           
float idata exInt=0,eyInt=0,ezInt=0; 
float idata accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z;
float idata Angle_ax,Angle_ay,Angle_az,Angle_gx,Angle_gy,Angle_gz;
float idata q0=1,q1=0,q2=0,q3=0;
float idata pitch,roll,yaw,pitch0,roll0,yaw0;

/**************取三个数中的中间的一个数值******************/
float medium(float i,float j,float k)
{
	float tmp;
	if(i>j)
	{
		tmp=i;i=j;j=tmp;
   }
	 if(k>j)
		 tmp=j;
	 else if (k>i)
		 tmp=k;
	 else 
		 tmp=i;
	 return tmp;
}
/**********转化正负*****////
float tranlate(unsigned char char1,unsigned char char2) 
{
	float aa;
	     if((char1*256+char2)>=32768) aa=(float)((char1*256+char2)-65536);
                else  aa= (float)(char1*256+char2);
	return aa;
}

/**********合成数据  有正负号********/
void dataadd()
{
	accel_x=tranlate(BUF[0],BUF[1]) ;
	accel_y=tranlate(BUF[2],BUF[3]) ;
	accel_z=tranlate(BUF[4],BUF[5]) ;
	gyro_x=tranlate(BUF[8],BUF[9]) ;
	gyro_y=tranlate(BUF[10],BUF[11]) ;
	gyro_z=tranlate(BUF[12],BUF[13]) ;
}
/****************滤波1**************/
void smoothing1()
{
	float idata axa,aya,aza,gxg,gyg,gzg,
	            xax,yay,zaz,xgx,ygy,zgz;   //取三组数据中的中间的数据
	read_MPU6050();
	dataadd();
	axa=accel_x;
	aya=accel_y;
	aza=accel_z;
	gxg=gyro_x;
	gyg=gyro_y;
	gzg=gyro_z;
	read_MPU6050();
	dataadd();
	xax=accel_x;
	yay=accel_y;
	zaz=accel_z;
	xgx=gyro_x;
	ygy=gyro_y;
	zgz=gyro_z;
	read_MPU6050();
	dataadd();
	accel_x=medium(axa,xax,accel_x);
	accel_y=medium(aya,yay,accel_y);
	accel_z=medium(aza,zaz,accel_z);
	gyro_x=medium(gxg,xgx,gyro_x);
	gyro_y=medium(gyg,ygy,gyro_y);
	gyro_z=medium(gzg,zgz,gyro_z);
}


/*******************处理原始数据*******************/
void dealdata()
{

	
	Angle_ax=accel_x/8192;
	Angle_ay=accel_y/8192;
	Angle_az=accel_z/8192;
	
	Angle_gx=gyro_x/65.5;
	Angle_gy=gyro_y/65.5;
	Angle_gz=gyro_z/65.5;
	
	
	Angle_ax=Angle_ax*pi/180.0;
	Angle_ay=Angle_ay*pi/180.0;
	Angle_az=Angle_az*pi/180.0;
	Angle_gx=Angle_gx*pi/180.0;
	Angle_gy=Angle_gy*pi/180.0;
	Angle_gz=Angle_gz*pi/180.0;
}





/********************计算角度*****************/
	void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float idata norm;
  float idata vx, vy, vz;
  float idata ex, ey, ez;

  float idata q0q0 = q0*q0;
  float idata q0q1 = q0*q1;
  float idata q0q2 = q0*q2;
  float idata q0q3 = q0*q3;
  float idata q1q1 = q1*q1;
  float idata q1q2 = q1*q2;
  float idata q1q3 = q1*q3;
  float idata q2q2 = q2*q2;
  float idata q2q3 = q2*q3;
  float idata q3q3 = q3*q3;

  norm = sqrt(ax*ax + ay*ay + az*az);      
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
           
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  pitch=asin(2*(q0*q2-q1*q3 ))* 57.2957795f; // 俯仰
  roll=-asin(2*(q0*q1+q2*q3 ))* 57.2957795f; // 横滚
	yaw=-asin(2*(q1*q2+q0*q3))*57.2957795f;  //航向
//	pitch=pitch-pitch0;
  //roll=roll-roll0;
//	yaw=yaw-yaw0;
}

//void database(void)
//{
//	smoothing1();
//	
//	
//	
//}