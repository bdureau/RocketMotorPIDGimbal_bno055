/*

   Description: Model Rocket motor gimbal using 2 servo. This is yet again another attempt to fly a model rocket without fins using long burn motors.
                It is using the Arduino PID controler to move a rocket motor.
                Angle changes can be monitored using a USB cable or a bluetooth interface
                Inspired by various camera gimbal projects
   Author: Boris du Reau
   Date: June 2018-2020
   Sensor used is an BNO085 board

   You can use an Arduino Uno/Nano or stm32F103C board

   Servo Connection
   BROWN - gnd
   red - 5v
   yellow - d10 (pwm for Sero X) or PA1 for stm32
          - d11 (servo Y) or PA2 for stm32

   GY-BNO055 board Connection
   VIN - 5v
   GND - GND
   SCL - A5  or pin SCL on the stm32
   SDA - A4  or pin SDA on the stm32
   

  This can be configured using the Gimbale Android application
  which is also hosted on Github
Version 1.0: 
Can log the data from all sensors on the eeprom and comunicate with an Android device 
via the serial port or a bluetooth module.  
Version 1.1
Configure the accelero and gyro range
Change the code so that it uses the bno055 instead of the MPU6050
*/


#include "config.h"
#include "global.h"
#include "utils.h"
#include "kalman.h"
#include "logger_i2c_eeprom.h"
logger_I2C_eeprom logger(0x50) ;
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start adress for the flights. Anything before that is the flight index
long currentMemaddress = 200;
boolean liftOff = false;
boolean landed = true;
//ground level altitude
long initialAltitude;
long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
bool canRecord;
bool recording = false;
bool rec = false;
unsigned long initialTime = 0;
unsigned long prevTime = 0;
unsigned long diffTime;
unsigned long currentTime = 0;
/*
 * ReadAltitude()
 * Read altitude and filter any nose with a Kalman filter
 */
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}


/*
   Initial setup
    

*/
void setup()
{
  pinMode(pinSpeaker, OUTPUT);
  digitalWrite(pinSpeaker, LOW);
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);
  
  ServoX.attach(PA1);  // attaches the X servo on PA1 for stm32 or D10 for the Arduino Uno
  ServoY.attach(PA2);  // attaches the Y servo on PA2 for stm32 or D11 for the Arduino Uno

  // set both servo's to 90 degree
  ServoX.write(90);
  ServoY.write(90);
  delay(500);
  Serial1.begin(38400);
  while (!Serial1);      // wait for Leonardo enumeration, others continue immediately // Do we need to????
  
  Wire.begin();
  // clock speed is important this is the only way I could get the BNO055 sensor to work with the stm32F103 board
  Wire.setClock(400000 );
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial1.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  
  // Read altimeter softcoded configuration
  boolean softConfigValid = false;
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    writeConfigStruc();
  }
  

  bmp.begin( config.altimeterResolution);

  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude(); //bmp.readAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  
  
  // Get flight
  int v_ret;
  v_ret = logger.readFlightList();
  //int epromsize = logger.determineSize();
  //Serial1.println(epromsize);
  long lastFlightNbr = logger.getLastFlightNbr();

  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  canRecord = logger.CanRecord();
  //canRecord = true;
}



/*

   MAIN PROGRAM LOOP

*/
void loop(void)
{
  MainMenu();
}

void Mainloop(void)
{
  long startTime = millis();
  /* Get a new sensor event */
  sensors_event_t linearAccelData;
  //bno.getEvent(&event);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  /* The WebSerial 3D Model Viewer also expects data as roll, pitch, heading */
  imu::Quaternion quat = bno.getQuat();
  
  //read current altitude
  currAltitude = (ReadAltitude() - initialAltitude);
  bool lift = false;
  if (config.liftOffDetect == 0) { //use baro detection
    if ( currAltitude > liftoffAltitude)
      lift = true;
  }
  else { // use accelero
   // if (mpu.getAccelerationY() > 30000)
   //   lift = true;
  }

  if ((lift && !liftOff) || (recording && !liftOff))
  {
    liftOff = true;
    if (recording)
      rec = true;
    // save the time
    initialTime = millis();
    prevTime = 0;
    if (canRecord)
    {
      long lastFlightNbr = logger.getLastFlightNbr();

      if (lastFlightNbr < 0)
      {
        currentFileNbr = 0;
        currentMemaddress = 201;
      }
      else
      {
        currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
        currentFileNbr = lastFlightNbr + 1;
      }
      //Save start address
      logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
    }
    //Serial1.println("We have a liftoff");
  }
  if (canRecord && liftOff)
  {
    currentTime = millis() - initialTime;
    diffTime = currentTime - prevTime;
    prevTime = currentTime;
    logger.setFlightTimeData( diffTime);
    logger.setFlightAltitudeData(currAltitude);
    logger.setFlightTemperatureData((long) bmp.readTemperature());
    logger.setFlightPressureData((long) bmp.readPressure());

    float w = (float)quat.w();//q.w;
    float x = (float)quat.x(); //q.x;
    float y = (float)quat.y(); //q.y;
    float z = (float)quat.z(); //q.z;

    logger.setFlightRocketPos((long) (w * 1000), (long) (x * 1000), (long) (y * 1000), (long) (z * 1000));
    logger.setFlightCorrection( (long) OutputX, (long)OutputY);
    logger.setAcceleration(linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z);
    //logger.setAcceleration(0,0,0);
    if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
      //flight is full let save it
      //save end address
      logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
      canRecord = false;
    } else {
      currentMemaddress = logger.writeFastFlight(currentMemaddress);
      currentMemaddress++;
    }
  }

  if (((canRecord && currAltitude < 10) && liftOff && !recording && !rec) || (!recording && rec))
  {
    liftOff = false;
    rec = false;
    //end loging
    //store start and end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
   
  }

 
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  //if using PID do those
  //ServoX.write(-OutputX + 90);
  //ServoY.write(OutputY + 90);

  // if you do not want to use the PID
  /* ServoX.write(-mpuPitch + 90);
   ServoY.write(mpuRoll + 90);*/
   int SX = -mpuPitch + 180;
   if (SX < 80) 
    SX = 80;
   if (SX > 100)
    SX = 100;
   int SY = mpuYaw + 90;

    if (SY < 80) 
    SY = 80;
   if (SY > 100)
    SY = 100;
    
   ServoX.write(SX);
   ServoY.write(SY);
   /*delay(10);*/
  

  float q1[4];
  //mpu.dmpGetQuaternion(&q, fifoBuffer);
  
  q1[0] = quat.w();//q.w;
  q1[1] = (float)quat.x();//q.x;
  q1[2] = (float)quat.y(); //q.y;
  q1[3] = (float)quat.z(); //q.z;
 
  //serialPrintFloatArr(q1, 4);
  SendTelemetry(q1, 200);
  checkBatVoltage(BAT_MIN_VOLTAGE);

  if (!liftOff) // && !canRecord)
    delay(10);

}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[1000]="";


  while ( readVal != ';') {
    if(mainLoopEnable)
      Mainloop();
    while (Serial1.available())
    {
      readVal = Serial1.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }

  interpretCommandBuffer(commandbuffer);
for (int i=0; i< sizeof(commandbuffer); i++)
  commandbuffer[i]='\0';
}
/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   e  erase all saved flights
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   w  Start or stop recording
   n  Return the number of recorded flights in the EEprom
   l  list all flights
   c  Different from other altimeters. Here it will calibrate the IMU
   a  get all flight data
   b  get altimeter config
   s  write altimeter config
   d  reset alti config
   h  hello. Does not do much
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
*/
void interpretCommandBuffer(char *commandbuffer) {
  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
#ifdef SERIAL_DEBUG
    Serial1.println(F("Erase\n"));
#endif
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      Serial1.print(F("$start;\n"));
      logger.printFlightData(atoi(temp));
      Serial1.print(F("$end;\n"));
    }
    else
      Serial1.println(F("not a valid flight"));
  }
  //start or stop recording
  else if (commandbuffer[0] == 'w')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Start Recording\n"));
#endif
      recording = true;
    }
    else {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Stop Recording\n"));
#endif
      recording = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    Serial1.print(F("$start;\n"));
    Serial1.print(F("$nbrOfFlight,"));
    logger.readFlightList();
    Serial1.print(logger.getLastFlightNbr() + 1);
    Serial1.print(";\n");
    Serial1.print(F("$end;\n"));
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    Serial1.println(F("Flight List: \n"));
    logger.printFlightList();
  }
  // calibrate the IMU
  else if (commandbuffer[0] == 'c')
  {
    //not used
    Serial1.print(F("$OK;\n"));
  }
  //get all flight data
  else if (commandbuffer[0] == 'a')
  {
    Serial1.print(F("$start;\n"));
    //getFlightList()
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }

    Serial1.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    Serial1.print(F("$start;\n"));

    SendAltiConfig();

    Serial1.print(F("$end;\n"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer))
      Serial1.print(F("$OK;\n"));
    else
      Serial1.print(F("$KO;\n"));
      commandbuffer="";
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    Serial1.print(F("$OK;\n"));
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Telemetry enabled\n"));
#endif
      telemetryEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Telemetry disabled\n"));
#endif
      telemetryEnable = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      Serial1.print(F("main Loop enabled\n"));
#endif
      mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      Serial1.print(F("main loop disabled\n"));
#endif
      mainLoopEnable = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  else
  {
    Serial1.println(F("Unknown command" ));
    Serial1.println(commandbuffer[0]);
    Serial1.println(commandbuffer);
  }
}

/*

   Send telemetry to the Android device

*/
void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  int pressure;
  //imu::Quaternion quat = bno.getQuat();
  /* Get a new sensor event */
  sensors_event_t event;
  //bno.getEvent(&event);
  sensors_event_t orientationData , linearAccelData,angVelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //float batVoltage;
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude() - initialAltitude;
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      Serial1.print(F("$telemetry,"));
      Serial1.print("RocketMotorGimbal_bno055");
      Serial1.print(F(","));
      //tab 1
      //GyroX
      Serial1.print(angVelData.gyro.x);
      Serial1.print(F(","));
      //GyroY
      Serial1.print(angVelData.gyro.y);
      Serial1.print(F(","));
      //GyroZ
      Serial1.print(angVelData.gyro.z);
      Serial1.print(F(","));
      //AccelX
      Serial1.print(linearAccelData.acceleration.x);
      Serial1.print(F(","));
      //AccelY
      Serial1.print(linearAccelData.acceleration.y);
      Serial1.print(F(","));
      //AccelZ
      Serial1.print(linearAccelData.acceleration.z);
      Serial1.print(F(","));
      mpuYaw=orientationData.orientation.x;
      //OrientX
      Serial1.print(mpuYaw);
      Serial1.print(F(","));
      //OrientY
      mpuRoll =orientationData.orientation.y;
      Serial1.print(mpuRoll); //mpuPitch
      Serial1.print(F(","));
      //OrientZ
      mpuPitch=orientationData.orientation.z;
      Serial1.print(mpuPitch);//mpuRoll
      Serial1.print(F(","));

      //tab 2
      //Altitude
      Serial1.print(currAltitude);
      Serial1.print(F(","));
      //temperature
      Serial1.print(temperature);
      Serial1.print(F(","));
      //Pressure
      Serial1.print(pressure);
      Serial1.print(F(","));
      //Batt voltage
      pinMode(PB1, INPUT_ANALOG);
      int batVoltage = analogRead(PB1);
      // Serial1.print(batVoltage);
      float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
      Serial1.print(bat);
      Serial1.print(F(","));
      //tab3
      serialPrintFloatArr(arr, 4);
      //Serial1.print(F(","));
      Serial1.print((int)(100*((float)logger.getLastFlightEndAddress()/endAddress))); 
      Serial1.print(F(","));
      Serial1.print((int)correct);
      Serial1.print(F(","));
      Serial1.print(-mpuPitch + 180); // ServoX
      Serial1.print(F(","));
      //Serial1.print(mpuRoll + 90); //ServoY
      Serial1.print(mpuYaw + 90); //ServoY
      Serial1.println(F(";"));
    }
}

/*

   Send the Gimbal configuration to the Android device

*/
void SendAltiConfig() {
  bool ret = readAltiConfig();

  Serial1.print(F("$alticonfig"));
  Serial1.print(F(","));
  //AltimeterName
  Serial1.print("RocketMotorGimbal_bno055");
  Serial1.print(F(","));
  Serial1.print(config.ax_offset);
  Serial1.print(F(","));
  Serial1.print(config.ay_offset);
  Serial1.print(F(","));
  Serial1.print(config.az_offset);
  Serial1.print(F(","));
  Serial1.print(config.gx_offset);
  Serial1.print(F(","));
  Serial1.print(config.gy_offset);
  Serial1.print(F(","));
  Serial1.print(config.gz_offset);
  Serial1.print(F(","));
  Serial1.print(config.KpX);
  Serial1.print(F(","));
  Serial1.print(config.KiX);
  Serial1.print(F(","));
  Serial1.print(config.KdX);
  Serial1.print(F(","));
  Serial1.print(config.KpY);
  Serial1.print(F(","));
  Serial1.print(config.KiY);
  Serial1.print(F(","));
  Serial1.print(config.KdY);
  Serial1.print(F(","));
  Serial1.print(config.ServoXMin);
  Serial1.print(F(","));
  Serial1.print(config.ServoXMax);
  Serial1.print(F(","));
  Serial1.print(config.ServoYMin);
  Serial1.print(F(","));
  Serial1.print(config.ServoYMax);
  Serial1.print(F(","));
  Serial1.print(config.connectionSpeed);
  Serial1.print(F(","));
  Serial1.print(config.altimeterResolution);
  Serial1.print(F(","));
  Serial1.print(config.eepromSize);
  Serial1.print(F(","));
  //alti major version
  Serial1.print(MAJOR_VERSION);
  //alti minor version
  Serial1.print(F(","));
  Serial1.print(MINOR_VERSION);
  Serial1.print(F(","));
  Serial1.print(config.unit);
  Serial1.print(F(","));
  Serial1.print(config.endRecordAltitude);
  Serial1.print(F(","));
  Serial1.print(config.beepingFrequency);
  Serial1.print(F(","));
  Serial1.print(config.liftOffDetect);
  Serial1.print(F(","));
  Serial1.print(config.gyroRange);
  Serial1.print(F(","));
  Serial1.print(config.acceleroRange);
  Serial1.print(F(";\n"));
  
}



/*

   Check if the battery voltage is OK.
   If not warn the user so that the battery does not get
   damaged by over discharging
*/
void checkBatVoltage(float minVolt) {

  pinMode(PB1, INPUT_ANALOG);
  int batVoltage = analogRead(PB1);
 
  float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
  
 /* if (bat < minVolt) {
    for (int i = 0; i < 10; i++)
    {
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }*/
}