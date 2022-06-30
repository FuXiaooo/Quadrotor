#include <i2c.h>


extern float idata pitch,roll,yaw,pitch0,roll0,yaw0;
extern float idata Angle_ax,Angle_ay,Angle_az,Angle_gx,Angle_gy,Angle_gz;
extern float idata accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z;



void dataadd(void);
void dealdata(void);
void smoothing1(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);