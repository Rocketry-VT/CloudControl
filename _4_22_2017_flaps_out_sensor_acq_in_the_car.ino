#include <SPI.h> //SPI library
#include <SD.h> //SD card library for datalogger
#include <RTClib.h> //Real Time Clock library for datalogger
#include <Wire.h> //Wire library
#include <SFE_BMP180.h> //Accellerometer library
#include <SFE_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#include <Adafruit_MotorShield.h> //Motor shield library
#include "utility/Adafruit_MS_PWMServoDriver.h" //Motor shield utility file

#define FS 10 //this is the loop frequency in Hz

#define ECHO_TO_SERIAL   1 // echo data to serial port; keep 1 to make the data you write to the SD card to the serial
#define WAIT_TO_START    0 // Wait for serial input in setup(); keep this 0 on the final version

const double G = 9.8436;

const double dt = 1 / FS;
const int launchArmPin = 2; //the pin to be set as an interrupt pin for Pad Arming (phase 2)
const int insideSwitch = 11;//pin for switch to check if fully retracted
const int outsideSwitch = 12;//pin for switch to check if fully deployed
const int stepSlow = 1;// step# per

int insideState = 0;
int outsideState = 0;
bool motorCal = false;
int calPhase = 1;
int stepCount = 0;
int deployCountLimit = 0;


int launchTime = 0;

bool deploying = true;

volatile int phase = 1;
/* Phase = 1 --> Waiting on ground; standby for moving on pad
   Phase = 2 --> On pad; begins datalogging; waiting for launch
   Phase = 3 --> In burn phase
   Phase = 4 --> Burn out; deploy flaps
   Phase = 5 --> Descent; stop logging
*/

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
RTC_DS1307 RTC; // define the Real Time Clock object
SFE_BMP180 pressure; //define the BMP180 barometer object (Address = 0x1D)
MMA8452Q accel; // Default MMA8452Q object create. (Address = 0x1D)
Adafruit_StepperMotor *stepper = AFMS.getStepper(200, 2);

File logfile; // the logging file

const int chipSelect = 10; // for the data logging shield, we use digital pin 10 for the SD cs line
double baseline; // baseline pressure, used to set the zero for our relative altitude measurement

int currentTime  = 0; //used for setting
int lastReading = 0;  //the frequency at which we fast loop
int motorReading = 0; //the frequency at which we go slow motor loop

double barAlt = 0; //variable to store the barometer's latest altitude measurement
double tempC = 0; //

double accelAccX = 0; //variable to store the accelerometer's latest x-acceleration
double accelAccY = 0; //variable to store the accelerometer's latest y-acceleration
double accelAccZ = 0; //variable to store the accelerometers' latest z-acceleration <-- most important

void setPhaseOnPad() {
  if (phase == 1) {
    phase = 2;
  }
  else if (phase == 2) {
    phase = 1;
  }
  
}

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  while (1);
}

void setup() {
  Serial.begin(9600);
  accel.init(SCALE_8G, ODR_400);  // Initialized at scale of +/-8g and 400 Hz ODR

  /* Get the baseline pressure:*/
  baseline = getPressure();


  pinMode(insideSwitch, INPUT_PULLUP);
  pinMode(outsideSwitch, INPUT_PULLUP);
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  stepper->setSpeed(20);
  stepper->release();

  pinMode(10, OUTPUT);// make sure that the default chip select pin is set to output, even if you don't use it
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }

  Serial.println("card initialized.");
  // create a new file
  char filename[] = "LOGGER00.txt";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  Wire.begin();

  logfile.println("Time (ms), Temp (C), Altitude (m), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Phase, ");
  /*attaching interrupt for launch detect*/
  pinMode(launchArmPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(launchArmPin), setPhaseOnPad, CHANGE);

 
}

void loop() {
  currentTime = millis();
  if (currentTime - lastReading >= (1000 / FS)) { //this runs at a certain frequency equal to FS in Hz
    /* getting barometer reading */
    lastReading = currentTime;
    double a, P;
    // Get a new pressure reading:
    P = getPressure();
    // Show the relative altitude difference between
    // the new reading and the baseline reading:
    a = pressure.altitude(P, baseline);
    barAlt = a;//stores altitude in m
    if (accel.available())
    {
      /* getting accelerometer reading */
      accel.read();
      accelAccX = 9.80665 * ((double) accel.cx);
      accelAccY = 9.80665 * ((double) accel.cy);
      accelAccZ = 9.80665 * ((double) accel.cz) - G;//correct for gravity
    }
    /*the control diagram
      Phase = 1 --> Waiting on ground; standby for moving on pad
      Phase = 2 --> On pad; begins datalogging; waiting for launch
      Phase = 3 --> In burn phase
      Phase = 4 --> Burn out; deploy flaps
      Phase = 5 --> Descent; stop logging
    */
    switch (phase) {
      case 1:
        /*Waiting on ground; standby for moving on pad*/
        break;
      case 2:
        /*On pad; begins datalogging; waiting for launch*/
        /* printing time, altitude, and acceleration out */
        logfile.print(lastReading);//in ms
        logfile.print(",");
        logfile.print(barAlt);//in m AGL
        logfile.print(",");
        logfile.print(tempC);//in degrees C
        logfile.print(",");
        logfile.print(accelAccX);//in m/s^2
        logfile.print(",");
        logfile.print(accelAccY);//in m/s^2
        logfile.print(",");
        logfile.print(accelAccZ);//in m/s^2
        logfile.print(",");
        logfile.print(phase);
        logfile.print(",");
        logfile.println(stepCount);//in steps
        if (accelAccZ >= 50.0) {
          phase = 3;
          launchTime = millis();
        }
        break;
      case 3:
        /*In burn phase*/
        /* printing time, altitude, and acceleration out */
        logfile.print(lastReading);//in ms
        logfile.print(",");
        logfile.print(barAlt);//in m AGL
        logfile.print(",");
        logfile.print(tempC);//in degrees C
        logfile.print(",");
        logfile.print(accelAccX);//in m/s^2
        logfile.print(",");
        logfile.print(accelAccY);//in m/s^2
        logfile.print(",");
        logfile.print(accelAccZ);//in m/s^2
        logfile.print(",");
        logfile.print(phase);
        logfile.print(",");
        logfile.println(stepCount);//in steps
        /*if altitude exceeds 1500 m, we're in burnout*/
        if (barAlt >= 1500.0) {
          phase = 4;
          motorReading = millis();
        }
        break;
      case 4:
        stepper->setSpeed(20);
        /*Burn out; deploy flaps*/
        /* printing time, altitude, and acceleration out */
        if (currentTime - motorReading >= 500) {
          motorReading = currentTime;
          insideState = !(digitalRead(insideSwitch));
          outsideState = !(digitalRead(outsideSwitch));
          if (insideState == HIGH)
          {
            deploying = true;
          }
          else if (outsideState == HIGH)
          {
            deploying = false;
          }
          if (deploying == true)
          {
            for (int thisPin = 1; thisPin <= 5; thisPin++)
            {
              stepper->step(2, FORWARD, SINGLE);
              delay(5);
            }
            stepCount += 10;
          }
          else if (deploying == false)
          {
            for (int thatPin = 1; thatPin <= 5; thatPin++)
            {
              stepper->step(2, BACKWARD, SINGLE);
              delay(5);
            }
            stepCount -= 10;
          }
          stepper->release();

          logfile.print(lastReading);//in ms
          logfile.print(",");
          logfile.print(barAlt);//in m AGL
          logfile.print(",");
          logfile.print(tempC);//in degrees C
          logfile.print(",");
          logfile.print(accelAccX);//in m/s^2
          logfile.print(",");
          logfile.print(accelAccY);//in m/s^2
          logfile.print(",");
          logfile.print(accelAccZ);//in m/s^2
          logfile.print(",");
          logfile.print(phase);
          logfile.print(",");
          logfile.println(stepCount);//in steps

          if ((millis() - launchTime) >= 60000) {
            phase = 5;
            logfile.close();
          }
        }
        break;
      case 5:
        stepper->setSpeed(20);
        insideState = !(digitalRead(insideSwitch));
        if (insideState == LOW) {
          stepper->step(2, BACKWARD, SINGLE);
          insideState = !(digitalRead(insideSwitch));
        }
        if (insideState == HIGH) {
          stepCount = 0;
          stepper->release();
        }
        delay(5);
        break;
    }
  }
}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(0);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        tempC = T;
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}


