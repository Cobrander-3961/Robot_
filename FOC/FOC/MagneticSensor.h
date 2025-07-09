#ifndef MAGNETICSENSOR_H
#define MAGNETICSENSOR_H
#include "main.h"

/******************************************************************************/
extern long  cpr;
extern float full_rotation_offset;
extern long angle_data, angle_data_prev;
extern unsigned long velocity_calc_timestamp;
extern float angle_prev;
/******************************************************************************/
void MagneticSensor_Init(void);
float getAngle(void);
// float getAngle_2(void);
// float getVelocity(void);
// // unsigned short I2C_getRawCount();
// // uint16_t AS5600_ReadTwoByte(uint16_t readAddr);
// void InitAllSensors();
/******************************************************************************/

#endif
