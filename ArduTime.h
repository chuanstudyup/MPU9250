#ifndef ARDUTIME_H
#define ARDUTIME_H

#include <time.h>
#include <cstdint>
#include <cstdio>

uint64_t micros();
uint32_t millis();
void getTodayDate(char *date);
void getNowTime(char *sTime);

#endif
