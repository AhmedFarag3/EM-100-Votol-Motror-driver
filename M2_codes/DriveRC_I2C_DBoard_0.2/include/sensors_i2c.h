#ifndef _SENSORS_I2C_H_
#define _SENSORS_I2C_H_

int Garmin1 = 0;
bool Garmin1_Read = LOW;

int Garmin2 = 0;
bool Garmin2_Read = LOW;

int Garmin3 = 0;
bool Garmin3_Read = LOW;

int Garmin4 = 0;
bool Garmin4_Read = LOW;

int Radar1 = 0;
bool Radar1_Read = LOW;

int Radar2 = 0;
bool Radar2_Read = LOW;

#define i2c_garmin1_flag 1500
#define i2c_garmin2_flag 1510
#define i2c_garmin3_flag 1520
#define i2c_garmin4_flag 1530

#define i2c_spot1_flag 610
#define i2c_spot2_flag 620

int sensors_data_received[4];
byte x;
int y;
int j;

#endif //_SENSORS_I2C_H_
