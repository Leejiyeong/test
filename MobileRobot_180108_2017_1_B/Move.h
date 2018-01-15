#ifndef __MOVE_H
#define __MOVE_H

#include "Motor.h"

void StartMotion(void);
void StopMotion(uint16_t mode);

void MOTOR_CTR(long MT0,long MT1,long MT2);


#endif      // __MOVE_H
