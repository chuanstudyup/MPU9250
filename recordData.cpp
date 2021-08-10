#include "MPU9250.h"
#include <fstream>
using namespace std;

int main()
{
	MPU9250 mpu;
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
	mpu.initRecordCalData();  //记录用来校准的原始数据时，先把校准参数都设置为0或1
	delay(5000);
	mpu.calibrateAccelGyro();  //粗校准：静置，自动校准acc和gyro的偏移量，然后写入到器件中
	mpu.print_calibration();
	printf("MagBias: %f, %f, %f\n",mpu.getMagBiasX(),mpu.getMagBiasY(),mpu.getMagBiasZ());
	printf("MagScale: %f, %f, %f\n",mpu.getMagScaleX(),mpu.getMagScaleY(),mpu.getMagScaleZ());
	
	delay(3000);
	//以下开始记录精校准用的原始数据，		
	ofstream magFile;
	magFile.open("rawData.csv",ios::app|ios::out);
	float startTime = millis()/1000.f;
	while(1)
	{
		if(mpu.update()){
			float now = millis()/1000.f-startTime;
			float accx = mpu.getAccX();
			float accy = mpu.getAccY();
			float accz = mpu.getAccZ();
			float gyrox = mpu.getGyroX();
			float gyroy = mpu.getGyroY();
			float gyroz = mpu.getGyroZ(); 
			float magx = mpu.getMagX();
			float magy = mpu.getMagY();
			float magz = mpu.getMagZ();
			printf("time=%f, accx=%f, accy=%f, accz=%f, gyrox=%f, gyroy=%f, gyroz=%f, magx=%f, magy=%f, magz=%f\n",
					now,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz);
			magFile <<now<<","<<accx<<","<<accy<<","<<accz<<","
					<<gyrox<<","<<gyroy<<","<<gyroz<<","
					<<magx<<","<<magy<<","<<magz<<endl;
			delay(8);
		}else{
			printf("mpu.update failed\n");
		}
	}
	magFile.close();
	return 0;
}
// g++ -o recordData recordData.cpp ArduTime.cpp -l bcm2835
