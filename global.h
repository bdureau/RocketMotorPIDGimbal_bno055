#ifndef _GLOBAL_H
#define _GLOBAL_H
#define INTERRUPT_PIN PB12
#define LED_PIN PC13 //pin 13 for the arduino Uno and PC13 for the stm32 
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>
#include <Servo.h> //servo library
#include <I2Cdev.h>
#include <PID_v1.h> // Arduino PID library
#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <BMP085_stm32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
BMP085 bmp;
bool blinkState = true;
bool telemetryEnable = false;
bool mainLoopEnable = true;

Servo ServoX;   // X axis Servo
Servo ServoY;   // Y axis Servo

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
float correct;

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define PITCH   2//1     // X defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  0     // Y defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW  1// 2     // Z defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// PID stuff
//Define Variables we'll be connecting to
double SetpointX=0, InputX, OutputX;
double SetpointY=-90, InputY, OutputY;

// Those initial tuning parameters need to be tuned
// this is a bit like when you tune your copter appart from the fact that the rocket motor last only few seconds
// please help !!!!!!!
//double KpX = 2, KiX = 5, KdX = 1;
//double KpX = 3.55, KiX = 0.005, KdX = 2.05;
double KpX =0.09, KiX = 0.06, KdX = 0.0275;

//Specify the links and initial tuning parameters
//double KpY = 2, KiY = 5, KdY = 1;
//double KpY = 3.55, KiY = 0.005, KdY = 2.05;
double KpY =0.09, KiY = 0.06, KdY = 0.0275;
//SetpointX = 0.0;
//SetpointY = -90.O;
PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

long last_telem_time=0;
#endif
