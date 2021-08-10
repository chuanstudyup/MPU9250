#include "ArduTime.h"

/**
 * @brief: Return the computer running to the current time of microseconds.
 * @return: Microseconds us
 **/
uint64_t micros(){
	timespec now;
	clock_gettime(CLOCK_MONOTONIC,&now);
	uint64_t mic = now.tv_sec*1000000 + now.tv_nsec/1000;
	return mic;
}

/**
 * @brief: Return the computer running to the current time of milliseconds.
 * @return: Milliseconds ms
 **/
uint32_t millis(){
	timespec now;
	clock_gettime(CLOCK_MONOTONIC,&now);
	uint32_t mil = now.tv_sec*1000 + now.tv_nsec/1000000;
	return mil;
}

/**
 * @brief: Get string form of today date YYYYMMDD
 * @param: date
 **/
void getTodayDate(char *date)
{
	time_t seconds;
	time(&seconds);
	tm nowtm;
	localtime_r(&seconds,&nowtm);
	sprintf(date,"%04d%02d%02d",nowtm.tm_year+1900,nowtm.tm_mon+1,nowtm.tm_mday);
}

/**
 * @brief: Get string form of now time HH:MM:SS.ms
 * @param: sTime
 **/
void getNowTime(char *sTime)
{
	timespec now;
	clock_gettime(CLOCK_REALTIME,&now);
	tm nowtm;
	localtime_r(&(now.tv_sec),&nowtm);
	sprintf(sTime,"%02d:%02d:%02d.%03d",nowtm.tm_hour,nowtm.tm_min,nowtm.tm_sec,static_cast<int>(now.tv_nsec/1000000));
}
