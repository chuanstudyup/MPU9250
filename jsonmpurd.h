#ifndef JSONMPURD_H
#define JSONMPURD_H

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <string>

# define CalibrationFileName "./CalibrationParam.json"

using namespace std;
using json = nlohmann::json;

/*
void saveAccelBias(float accelBias[3]);
void saveGyroBias(float gyroBias[3]);
void saveMagBias(float magBias[3]);
void saveMagScale(float magScale[3]);
void saveCalibrationParam(float accelBias[3], float gyroBias[3], float magBias[3], float magScale[3]);
void getCalibrationParam(float accelBias[3], float gyroBias[3], float magBias[3], float magScale[3]);
*/

void saveAccelBias(float accelBias[3])
{
	fstream jsonFile(CalibrationFileName, ios::in);
	json rootObject;
	jsonFile >> rootObject;
	jsonFile.close();
	
	rootObject["accelBiasX"] = accelBias[0];
	rootObject["accelBiasY"] = accelBias[1];
	rootObject["accelBiasZ"] = accelBias[2];
	
	jsonFile.open(CalibrationFileName,ios::out);								   
	jsonFile << rootObject;
	jsonFile.close();
}

void saveGyroBias(float gyroBias[3])
{
	fstream jsonFile(CalibrationFileName, ios::in);
	json rootObject;
	jsonFile >> rootObject;
	jsonFile.close();
	
	rootObject["gyroBiasX"] = gyroBias[0];
	rootObject["gyroBiasY"] = gyroBias[1];
	rootObject["gyroBiasZ"] = gyroBias[2];
	
	jsonFile.open(CalibrationFileName,ios::out);								   
	jsonFile << rootObject;
	jsonFile.close();
}

void saveMagBias(float magBias[3])
{
	fstream jsonFile(CalibrationFileName, ios::in);
	json rootObject;
	jsonFile >> rootObject;
	jsonFile.close();
	
	rootObject["magBiasX"] = magBias[0];
	rootObject["magBiasY"] = magBias[1];
	rootObject["magBiasZ"] = magBias[2];
	
	jsonFile.open(CalibrationFileName,ios::out);								   
	jsonFile << rootObject;
	jsonFile.close();
}

void saveMagScale(float magScale[3])
{
	fstream jsonFile(CalibrationFileName, ios::in);
	json rootObject;
	jsonFile >> rootObject;
	jsonFile.close();
	
	rootObject["magScaleX"] = magScale[0];
	rootObject["magScaleY"] = magScale[1];
	rootObject["magScaleZ"] = magScale[2];
	
	jsonFile.open(CalibrationFileName,ios::out);								   
	jsonFile << rootObject;
	jsonFile.close();
}

void saveCalibrationParam(float accelBias[3], float gyroBias[3], float magBias[3], float magScale[3])
{
	fstream jsonFile(CalibrationFileName, ios::in);
	json rootObject;
	jsonFile >> rootObject;
	jsonFile.close();
	
	rootObject["accelBiasX"] = accelBias[0];
	rootObject["accelBiasY"] = accelBias[1];
	rootObject["accelBiasZ"] = accelBias[2];
	rootObject["gyroBiasX"] = gyroBias[0];
	rootObject["gyroBiasY"] = gyroBias[1];
	rootObject["gyroBiasZ"] = gyroBias[2];
	rootObject["magBiasX"] = magBias[0];
	rootObject["magBiasY"] = magBias[1];
	rootObject["magBiasZ"] = magBias[2];
	rootObject["magScaleX"] = magScale[0];
	rootObject["magScaleY"] = magScale[1];
	rootObject["magScaleZ"] = magScale[2];
		
	jsonFile.open(CalibrationFileName,ios::out);								   
	jsonFile << rootObject;
	jsonFile.close();
}

void getCalibrationParam(float accelBias[3], float gyroBias[3], float magBias[3], float magScale[3])
{
	ifstream jsonFile(CalibrationFileName);
	json rootObject;
	jsonFile >> rootObject;
	jsonFile.close();
	
	accelBias[0] = rootObject["accelBiasX"];
	accelBias[1] = rootObject["accelBiasY"];
	accelBias[2] = rootObject["accelBiasZ"];
	gyroBias[0] = rootObject["gyroBiasX"];
	gyroBias[1] = rootObject["gyroBiasY"];
	gyroBias[2] = rootObject["gyroBiasZ"];
	magBias[0] = rootObject["magBiasX"];
	magBias[1] = rootObject["magBiasY"];
	magBias[2] = rootObject["magBiasZ"];
	magScale[0] = rootObject["magScaleX"];
	magScale[1] = rootObject["magScaleY"];
	magScale[2] = rootObject["magScaleZ"];
}
#endif
