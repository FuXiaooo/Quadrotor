#include <i2c.h>




extern unsigned char dmpdatas[42],qq[4],q[4],pitch,roll,yaw; //DMP����  







 
void loadfirmware(void);
void loadcfgupd(void);
void xdmpUpdates(unsigned char datacounts);
unsigned int getFIFOCount()  ;
void readdmp(unsigned char *Data);
void dmpInitialize(void)  ;
void compute (void);