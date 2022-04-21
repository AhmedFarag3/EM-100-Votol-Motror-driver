#ifndef _KELLY_H
#define _KELLY_H

#include <Arduino.h>

// #### FR (1) Wheel ####
#define FWD_SW_FR 33 // (white 12)////
#define REV_SW_FR 20 // (orange 14) ///////
#define BRK_SW_FR 29 // (yellowish 25)///////
#define THR_AN_FR 15 // analog output to control the speed (green 3)
#define BRK_AN_FR 1  // analog output to control the brake (Brown 2)

// #### FL (2) Wheel ####
#define FWD_SW_FL 17 // (white 12)////
#define REV_SW_FL 16 // (orange 14) ///////
#define BRK_SW_FL 30 // (yellowish 25)///////
#define THR_AN_FL 2  // analog output to control the speed (green 3)
#define BRK_AN_FL 3  // analog output to control the brake (Brown 2)

// #### BR (3) Wheel ####
#define FWD_SW_BR 12 // (white 12)////
#define REV_SW_BR 26 // (orange 14) ///////
#define BRK_SW_BR 30 // (yellowish 25)///////
#define THR_AN_BR 6  // analog output to control the speed (green 3)
#define BRK_AN_BR 7  // analog output to control the brake (Brown 2)
// #define footsw_bl     4 // define foot switch pin (gray 15)
// #define speedmeter_bl  // measure the motor speed (gray 8)

// #### BL (4) Wheel ####
#define FWD_SW_BL 32 // (white 12)////
#define REV_SW_BL 31 // (orange 14) ///////
#define BRK_SW_BL 22 // (yellowish 25)///////
#define THR_AN_BL 14 // analog output to control the speed (green 3)
#define BRK_AN_BL 13 // analog output to control the brake (Brown 2)

const unsigned int maxfwd = 200;
const unsigned int minfwd = 60;
const unsigned int maxrev = -200;
const unsigned int minrev = -60;
const unsigned int maxbrake = 250;
const unsigned int minbrake = 20;

void kellyInit();

void kellyBrake(unsigned int BRK_SW_PIN, unsigned int throttle_PWM_PIN, unsigned int brake_PWM_PIN, unsigned int BRK_MAX_PWM);

void kellyForward(unsigned int FWD_SW_PIN, unsigned int REV_SW_PIN, unsigned int BRK_SW_PIN, unsigned int throttle_PWM_PIN, unsigned int throttle_percentage, unsigned int FWD_MIN_PWM, unsigned int FWD_MAX_PWM);

void kellyBackward(unsigned int FWD_SW_PIN, unsigned int REV_SW_PIN, unsigned int BRK_SW_PIN, unsigned int throttle_PWM_PIN, int throttle_percentage, unsigned int REV_MIN_PWM, unsigned int REV_MAX_PWM);

#endif //_KELLY_H