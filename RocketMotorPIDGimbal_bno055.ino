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
  Adding checksum
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
int SX, SY;

/* Get a new sensor event */
sensors_event_t orientationData , linearAccelData, angVelData;

/*
   ReadAltitude()
   Read altitude and filter any nose with a Kalman filter
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

  ServoX.attach(PA3);  // attaches the X servo on PA1 for stm32 or D10 for the Arduino Uno
  ServoY.attach(PA7);  // attaches the Y servo on PA2 for stm32 or D11 for the Arduino Uno

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

  //PID
  SetpointX = 0.0;
  SetpointY = -90.0;
  myPIDX.SetOutputLimits(-5, 5);
  myPIDY.SetOutputLimits(-95, -85);
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
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


  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
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

    float w = (float)quat.w();
    float x = (float)quat.x(); 
    float y = (float)quat.y(); 
    float z = (float)quat.z(); 

    logger.setFlightRocketPos((long) (w * 1000), (long) (x * 1000), (long) (y * 1000), (long) (z * 1000));
    logger.setFlightCorrection( (long) OutputX, (long)OutputY);
    logger.setAcceleration(linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z);

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

  /*mpuYaw = orientationData.orientation.x;
  mpuPitch = orientationData.orientation.y;
  mpuRoll = orientationData.orientation.z;*/

  InputX = orientationData.orientation.y;
  myPIDX.Compute();
  InputY = orientationData.orientation.z;
  myPIDY.Compute();

  /*Serial1.print(OutputX);
    Serial1.print("   ");
    Serial1.println(orientationData.orientation.y);
    Serial1.print(OutputY);
    Serial1.print("   ");
    Serial1.println(orientationData.orientation.z);*/
  // if you do not want to use the PID

  SX = orientationData.orientation.y + 90;
  //SX = OutputX +90;
  if (SX < 80)
    SX = 80;
  if (SX > 100)
    SX = 100;

  SY = -orientationData.orientation.z;
  //SY = -OutputY;
  if (SY < 80)
    SY = 80;
  if (SY > 100)
    SY = 100;

  ServoX.write(SX);
  ServoY.write(SY);



  float q1[4];

  q1[0] = quat.w();
  q1[1] = (float)quat.x();
  q1[2] = (float)quat.y();
  q1[3] = (float)quat.z();

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

  char commandbuffer[300] = "";


  while ( readVal != ';') {
    if (mainLoopEnable)
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
  for (int i = 0; i < sizeof(commandbuffer); i++)
    commandbuffer[i] = '\0';
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
    commandbuffer = "";
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
/*void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  int pressure;

  String myTelemetry="";

  //float batVoltage;
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude() - initialAltitude;
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      //Serial1.print(F("$telemetry,"));
      //Serial1.print("RocketMotorGimbal_bno055");
      //Serial1.print(F(","));
      myTelemetry = myTelemetry + "telemetry,RocketMotorGimbal_bno055,";
      //tab 1
      //GyroX
      //Serial1.print(angVelData.gyro.x);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry + angVelData.gyro.x+ ",";
      //GyroY
      //Serial1.print(angVelData.gyro.y);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +angVelData.gyro.y + ",";
      //GyroZ
      //Serial1.print(angVelData.gyro.z);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +angVelData.gyro.z + ",";
      //AccelX
      //Serial1.print(linearAccelData.acceleration.x);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +linearAccelData.acceleration.x + ",";
      //AccelY
      //Serial1.print(linearAccelData.acceleration.y);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +linearAccelData.acceleration.y + ",";
      //AccelZ
      //Serial1.print(linearAccelData.acceleration.z);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +linearAccelData.acceleration.z + ",";
      //OrientX
      mpuYaw = orientationData.orientation.x;
      //Serial1.print(mpuYaw);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +mpuYaw + ",";
      //OrientY
      mpuRoll = orientationData.orientation.z;
      //Serial1.print(mpuRoll); //mpuPitch
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +mpuRoll + ",";
      //OrientZ
      mpuPitch = orientationData.orientation.y;
      //Serial1.print(mpuPitch);//mpuRoll
      //Serial1.print(F(","));
      myTelemetry = myTelemetry + mpuPitch+ ",";
      //tab 2
      //Altitude
      //Serial1.print(currAltitude);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +currAltitude + ",";
      //temperature
      //Serial1.print(temperature);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +temperature + ",";
      //Pressure
      //Serial1.print(pressure);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry + pressure+ ",";
      //Batt voltage
      pinMode(PB1, INPUT_ANALOG);
      int batVoltage = analogRead(PB1);
      // Serial1.print(batVoltage);
      float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
      //Serial1.print(bat);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +bat + ",";
      //tab3
      //serialPrintFloatArr(arr, 4);
      char temp [9]="";
      floatToByte(arr[0], temp);
      myTelemetry = myTelemetry + temp + ",";
      floatToByte(arr[1], temp);
      myTelemetry = myTelemetry + temp + ",";
      floatToByte(arr[2], temp);
      myTelemetry = myTelemetry + temp + ",";
      floatToByte(arr[3], temp);
      myTelemetry = myTelemetry + temp + ",";
      //Serial1.print(F(","));
      //Serial1.print((int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)));
      //Serial1.print(F(","));
      myTelemetry = myTelemetry + (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)) + ",";
      //Serial1.print((int)correct);
      //Serial1.print(F(","));
      myTelemetry = myTelemetry + (int)correct + ",";
      //Serial1.print(SX); // ServoX
      //Serial1.print(F(","));
      myTelemetry = myTelemetry +SX + ",";
      //Serial1.print(SY); //ServoY
      //Serial1.println(F(";"));
      myTelemetry = myTelemetry +SY +",";//+ ";";

      String checkCal = myTelemetry;
      checkCal.replace(",","");
      char msg[checkCal.length() + 1];
      checkCal.toCharArray(msg, checkCal.length() + 1);
      unsigned int chk;
      chk = msgChk(msg, sizeof(msg));
      myTelemetry = myTelemetry + chk + ";";
      Serial1.println("$"+myTelemetry);
    }
  }
*/
void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  int pressure;

  char myTelemetry[300] = "";

  //float batVoltage;
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude() - initialAltitude;
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      strcat( myTelemetry , "telemetry,RocketMotorGimbal_bno055,");
      //tab 1
      //GyroX
      char temp[10];
      sprintf(temp, "%f", angVelData.gyro.x);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //GyroY
      sprintf(temp, "%f", angVelData.gyro.y);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //GyroZ
      sprintf(temp, "%f", angVelData.gyro.z);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //AccelX
      sprintf(temp, "%f", linearAccelData.acceleration.x);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //AccelY
      sprintf(temp, "%f", linearAccelData.acceleration.y);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //AccelZ
      sprintf(temp, "%f", linearAccelData.acceleration.z);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //OrientX
      sprintf(temp, "%f", orientationData.orientation.x);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //OrientY
      sprintf(temp, "%f", orientationData.orientation.z);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //OrientZ
      sprintf(temp, "%f", orientationData.orientation.y);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //tab 2
      //Altitude
      sprintf(temp, "%i", currAltitude);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //temperature
      sprintf(temp, "%i", temperature);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //Pressure
      sprintf(temp, "%i", pressure);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //Batt voltage
      pinMode(PB1, INPUT_ANALOG);
      int batVoltage = analogRead(PB1);
      float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
      sprintf(temp, "%f", bat);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      //tab3
      floatToByte(arr[0], temp);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      floatToByte(arr[1], temp);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      floatToByte(arr[2], temp);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");
      floatToByte(arr[3], temp);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");

      sprintf(temp, "%i", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)));
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");

      sprintf(temp, "%i", (int)correct);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");

      sprintf(temp, "%i", SX);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");

      sprintf(temp, "%i", SY);
      strcat( myTelemetry , temp);
      strcat( myTelemetry, ",");

      unsigned int chk;
      chk = msgChk(myTelemetry, sizeof(myTelemetry));
      sprintf(temp, "%i", chk);
      strcat(myTelemetry, temp);
      strcat(myTelemetry, ";");
      Serial1.print("$");
      Serial1.println(myTelemetry);
    }
}
/*

   Send the Gimbal configuration to the Android device

*/
/*void SendAltiConfig() {
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

  }*/
/*void SendAltiConfig() {
  bool ret = readAltiConfig();
  String myconfig= "";
  // Serial1.print(F("$alticonfig"));
  // Serial1.print(F(","));
  myconfig = myconfig + "$alticonfig,";
  //AltimeterName
  // Serial1.print("RocketMotorGimbal_bno055");
  // Serial1.print(F(","));
  myconfig = myconfig + "RocketMotorGimbal_bno055,";
  // Serial1.print(config.ax_offset);
  //Serial1.print(F(","));
  myconfig = myconfig +config.ax_offset +",";
  //Serial1.print(config.ay_offset);
  //Serial1.print(F(","));
  myconfig = myconfig +config.ay_offset +",";
  // Serial1.print(config.az_offset);
  //Serial1.print(F(","));
  myconfig = myconfig +config.az_offset +",";
  //Serial1.print(config.gx_offset);
  //Serial1.print(F(","));
  myconfig = myconfig + config.gx_offset+",";
  //Serial1.print(config.gy_offset);
  //Serial1.print(F(","));
  myconfig = myconfig +config.gy_offset +",";
  //Serial1.print(config.gz_offset);
  //Serial1.print(F(","));
  myconfig = myconfig +config.gz_offset +",";
  //Serial1.print(config.KpX);
  //Serial1.print(F(","));
  myconfig = myconfig +config.KpX +",";
  //Serial1.print(config.KiX);
  //Serial1.print(F(","));
  myconfig = myconfig + config.KiX+",";
  //Serial1.print(config.KdX);
  //Serial1.print(F(","));
  myconfig = myconfig +config.KdX +",";
  //Serial1.print(config.KpY);
  //Serial1.print(F(","));
  myconfig = myconfig +config.KpY +",";
  //Serial1.print(config.KiY);
  //Serial1.print(F(","));
  myconfig = myconfig + config.KiY+",";
  //Serial1.print(config.KdY);
  //Serial1.print(F(","));
  myconfig = myconfig + config.KdY+",";
  //Serial1.print(config.ServoXMin);
  //Serial1.print(F(","));
  myconfig = myconfig +config.ServoXMin +",";
  //Serial1.print(config.ServoXMax);
  //Serial1.print(F(","));
  myconfig = myconfig +config.ServoXMax +",";
  //Serial1.print(config.ServoYMin);
  //Serial1.print(F(","));
  myconfig = myconfig +config.ServoYMin +",";
  //Serial1.print(config.ServoYMax);
  //Serial1.print(F(","));
  myconfig = myconfig +config.ServoYMax +",";
  //Serial1.print(config.connectionSpeed);
  //Serial1.print(F(","));
  myconfig = myconfig + config.connectionSpeed +",";
  //Serial1.print(config.altimeterResolution);
  //Serial1.print(F(","));
  myconfig = myconfig + config.altimeterResolution +",";
  //Serial1.print(config.eepromSize);
  //Serial1.print(F(","));
  myconfig = myconfig +config.eepromSize +",";
  //alti major version
  //Serial1.print(MAJOR_VERSION);
  //alti minor version
  // Serial1.print(F(","));
  myconfig = myconfig + MAJOR_VERSION+",";
  //Serial1.print(MINOR_VERSION);
  //Serial1.print(F(","));
  myconfig = myconfig +MINOR_VERSION +",";
  // Serial1.print(config.unit);
  //Serial1.print(F(","));
  myconfig = myconfig +config.unit +",";
  //Serial1.print(config.endRecordAltitude);
  //Serial1.print(F(","));
  myconfig = myconfig + config.endRecordAltitude +",";
  //Serial1.print(config.beepingFrequency);
  //Serial1.print(F(","));
  myconfig = myconfig + config.beepingFrequency +",";
  //Serial1.print(config.liftOffDetect);
  //Serial1.print(F(","));
  myconfig = myconfig + config.liftOffDetect +",";
  //Serial1.print(config.gyroRange);
  //Serial1.print(F(","));
  myconfig = myconfig + config.gyroRange +",";
  //Serial1.print(config.acceleroRange);
  //Serial1.print(F(";\n"));
  myconfig = myconfig + config.acceleroRange + ";\n";
  Serial1.print(myconfig);
  }*/

void SendAltiConfig() {
  bool ret = readAltiConfig();
  char myconfig [300] = "";

  strcat(myconfig , "alticonfig,");
  //AltimeterName
  strcat(myconfig , "RocketMotorGimbal_bno055,");
  char temp [10];
  sprintf(temp, "%i", config.ax_offset);
  strcat( myconfig , temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ay_offset);
  strcat( myconfig , temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.az_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gx_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gy_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gz_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KpX);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KiX);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KdX);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KpY);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KiY);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KdY);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoXMin);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoXMax);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoYMin);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoYMax);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.connectionSpeed);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.altimeterResolution);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.eepromSize);
  strcat( myconfig, temp);
  strcat( myconfig, ",");
  //alti major version
  sprintf(temp, "%i", MAJOR_VERSION);
  strcat( myconfig, temp);
  strcat( myconfig, ",");
  //alti minor version
  sprintf(temp, "%i", MINOR_VERSION);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.unit);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.endRecordAltitude);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.beepingFrequency);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.liftOffDetect);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gyroRange);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.acceleroRange);
  strcat( myconfig, temp);
  strcat( myconfig, ",");
  unsigned int chk = msgChk(myconfig, sizeof(myconfig));
  sprintf(temp, "%i", chk);
  strcat(myconfig, temp);
  strcat(myconfig, ";\n");
  Serial1.print("$");
  Serial1.print(myconfig);
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

  if (bat < minVolt) {
    for (int i = 0; i < 10; i++)
    {
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }
}
