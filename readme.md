# RocketMotorPIDGimbal BNO055
This is another rocket Gimbal that is using a bno055 sensor instead of the MPU6050
It looks a lot more stable and a lot easier to use. No interupt, no buffer overflow etc..
I am redesining a board

I am using the Arduino PID library an STM32F103C board and a 3D printer to print the 24mm motor gimbal
It uses a bluetooth module (HC-05, HC-06 etc ...)to communicate with an Android device

This is the bno055 version of 
https://github.com/bdureau/RocketMotorPIDGimbal

Use it with my Gimbal console application
https://github.com/bdureau/MotorGimbalConsole
