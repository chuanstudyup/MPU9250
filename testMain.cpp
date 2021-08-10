#include <iostream>
#include "MPU9250.h"

MPU9250 mpu;

int main()
{
	
    bcm2835_init();
    mpu.verbose(true);
    if(!mpu.setup(0x68))
    {
	while(1){
		printf("MPU connection failed.\n");
		delay(5000);
	}
    }else
	printf("MPU setup successfully!\n");
    
    /**精校准系数**/
    mpu.Ta12 = 0.0002;mpu.Ta13=-0.0013;mpu.Ta23=-0.0066;
    mpu.Ka11 = 0.9980;mpu.Ka22=0.9985;mpu.Ka33=0.9797;
    mpu.Bax = -0.0695;mpu.Bay=0.0073;mpu.Baz=0.0171;
    
    mpu.Tg12 = -0.0085;mpu.Tg13 = -0.0021;
    mpu.Tg21 = -0.0075;mpu.Tg23 = 0.0014;
    mpu.Tg31 = -0.0038;mpu.Tg32 = -0.0085;
    mpu.Kg11 = 1.0010;mpu.Kg22=1.0000;mpu.Kg33=0.9968;
    mpu.Bgx = 0.0518;mpu.Bgy=-0.0848;mpu.Bgz=-0.0281;
    
    mpu.setMagBias(366,859,-443,false);
    mpu.setMagScale(0.9384,1.1889,0.9141,false);
    
    
    mpu.print_calibration();
    //mpu.coarseAlignment(); //
    mpu.selectFilter(QuatFilterSel::EKF);
    delay(5000);
    mpu.verbose(false);
    
    while(1)
    {
	if(mpu.update()){
		uint32_t now = millis();
		printf("now = %d ",now);
		printf("Yaw,Pitch,Roll: %f %f %f\n",mpu.getYaw(),mpu.getPitch(),mpu.getRoll());
		delay(8);
	}else{
		printf("mpu.update failed\n");
	}
    }
    return 0;
}
// g++ -o testMain testMain.cpp ArduTime.cpp -l bcm2835
