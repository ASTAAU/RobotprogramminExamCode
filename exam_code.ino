#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#define NUM_SENSORS 5

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonA buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Buzzer buzz;
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;

uint16_t lineSensorValues[NUM_SENSORS];
bool useEmitters = true;

bool lineFound = false;
int threshHold = 500;
int sideThreshHold = 600;
int motorspeed = 100;
int motorspeedFast = 250;
int turnSpeed = 150;

struct LineSensorsWhite
{ // True if White, False if Black
  bool L;
  bool LC;
  bool C;
  bool RC;
  bool R;
};

LineSensorsWhite sensorsState = {0, 0, 0, 0, 0};

bool detectObject = false;
int brightnessLevel[6] = {1, 2, 3, 4, 5, 6};
int sensorthreshHold = 3;

//------------Functions------------//
//This function drives the robot until it finds a line, and turns the robot to align with the line
void lineFind() {

  //this boolean is used to break the while loop later
  lineFound = false;

  //set motors to drive forward at speed 100
  motors.setSpeeds(100, 100);

  //in this loop, the robot drives forward until it detects a white line
  while (lineFound == false) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    Serial.println(lineSensorValues[2]);

    //the robot checks the middle sensor to see if it registers white and stops if it does
    if (lineSensorValues[2] < threshHold) {
      //Serial.println(1);
      lineFound = true;
      motors.setSpeeds(0, 0);
    } else {
      //Serial.println(0);
    }
  }

  //resets the encoders and the counts variable
  int counts = 0;
  encoders.getCountsAndResetRight();

  //drive forward a small distance to center the robot over the line
  motors.setSpeeds(80, 80);
  while (counts < 450) {
    //900 cpr
    counts = encoders.getCountsRight();
    delay(5);
  }

  //reset the encoders and lineFound bool, and sets the motors to turn right
  encoders.getCountsAndResetRight();
  lineFound = false;
  motors.setSpeeds(100, -100);

  //turn the robot until the middle sensor detects the line
  while (lineFound == false) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    Serial.println(lineSensorValues[2]);
    if (lineSensorValues[2] < threshHold) {
      //Serial.println(1);
      lineFound = true;
      motors.setSpeeds(0, 0);
    }
  }
}

//This function reads the line sensors, and adds them to an array
void readSensors(LineSensorsWhite &state) {
  //reads the sensor values and store them in the array lineSensorValues
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
}

//This function follows the line to get to the belt sensor reader
void followLine(int a) {

  bool b = true; //this is for breaking out of the while loop
  int forwardvalue = 55; //the value for how fast the zumo drives
  int turnvalue = 50; // the value for how fast the zumo turns

  //while b is true, the struct is opdateded and keeps the zumo on the line
  while (b == true)
  {

    readSensors(sensorsState);
    // print the bool values of the struct
    lineSensors.read(lineSensorValues);

    //checks if the leftmost linesensor is under or over the threshold
    if (lineSensorValues[0] < threshHold)
    {
      sensorsState.L = true;
      lcd.gotoXY(0, 0);
      lcd.write('1');
    }
    else
    {
      sensorsState.L = false;
      lcd.gotoXY(0, 0);
      lcd.write('0');
    }
    //checks if the left linesensor is under or over the threshold
    if (lineSensorValues[1] < threshHold)
    {
      sensorsState.LC = true;
      lcd.gotoXY(1, 0);
      lcd.write('1');
    }
    else
    {
      sensorsState.LC = false;
      lcd.gotoXY(1, 0);
      lcd.write('0');
    }
    //checks if the centor linesensor is under or over the threshold
    if (lineSensorValues[2] < threshHold)
    {
      sensorsState.C = true;
      lcd.gotoXY(2, 0);
      lcd.write('1');
    }
    else
    {
      sensorsState.C = false;
      lcd.gotoXY(2, 0);
      lcd.write('0');
    }
    //checks if the right linesensor is under or over the threshold
    if (lineSensorValues[3] < threshHold)
    {
      sensorsState.RC = true;
      lcd.gotoXY(3, 0);
      lcd.write('1');
    }
    else
    {
      sensorsState.RC = false;
      lcd.gotoXY(3, 0);
      lcd.write('0');
    }
    //checks if the rightmost linesensor is under or over the threshold
    if (lineSensorValues[4] < threshHold)
    {
      sensorsState.R = true;
      lcd.gotoXY(4, 0);
      lcd.write('1');
    }
    else
    {
      sensorsState.R = false;
      lcd.gotoXY(4, 0);
      lcd.write('0');
    }

    /*
      if (sensorsState.C)
      {
       motors.setSpeeds(forwardvalue,forwardvalue);
       Serial.println("first forward");
      }*/

    //if the rightmost and leftmost linesensor ses white, then the zumo stops and breaks the while loop
    if (sensorsState.L && sensorsState.R)
    {
      motors.setSpeeds(0, 0);
      Serial.println("stop");
      b = false;
    }
    //makes the zumo turn left
    else if ((sensorsState.LC && sensorsState.C && !sensorsState.RC) ||
             (sensorsState.LC && !sensorsState.C && !sensorsState.RC))
    {
      motors.setSpeeds(0, turnvalue);
      Serial.println("left");
    }
    //makes the zumo turn right
    else if ((!sensorsState.LC && sensorsState.C && sensorsState.RC) ||
             (!sensorsState.LC && !sensorsState.C && sensorsState.RC))
    {
      motors.setSpeeds(turnvalue, 0);
      Serial.println("right");
    }

    //if the centor linesensor ses white then the zumo goes forward
    else if (/*sensorsState.LC && */ sensorsState.C /* && sensorsState.RC*/)
    {
      motors.setSpeeds(forwardvalue, forwardvalue);
      Serial.println("first forward");
    }
    
    //if the rightmost and leftmost linesensor ses white, then the zumo stops and breaks the while loop
    else if (sensorsState.L && sensorsState.R)
    {
      motors.setSpeeds(0, 0);
      Serial.println("stop");
      b = false;
    }
    //if non of the above is not true then the zumo just goes forward
    else
    {
      motors.setSpeeds(forwardvalue, forwardvalue);
      Serial.println("second forward");
    }
  }
}

//This function looks for an obstacle in fornt of the robot, and uses the distance to determine can size
char detectType() {

  //initialize variables for the prox sensor, and set brightness levels of the sensors
  uint8_t leftValue;
  uint8_t rightValue;
  proxSensors.setBrightnessLevels(brightnessLevel, 6);

  //looks for objects in front of the sensors
  detectObject = false;
  while (detectObject == false)  {
    lineSensors.emittersOn();
    delay(500);
    proxSensors.read();
    leftValue = proxSensors.countsFrontWithLeftLeds();
    rightValue = proxSensors.countsFrontWithRightLeds();
    Serial.println("proxLeft: " + String(leftValue) + " // " + "proxRight: " + String(rightValue));

    if (leftValue >= sensorthreshHold) {
      detectObject = true;
    }
    delay(500);
  }

  //turn off the linesensor emitters to turn off the belt
  lineSensors.emittersOff();
  delay(200);

  //if the prox sensor recieves all pulses back, return 'L' for Large
  if (leftValue >= 6 ) {
    Serial.println("funktion 1");
    buzz.playFrequency(220,250,11);
    return 'L';
  }

  //if the prox sensor recieves only some pulses back, return 'S' for Small
  if (leftValue <= 5 ) {
    Serial.println("funktion 2");
    buzz.playFrequency(880,250,11);
    return 'S';
  }
}

//This function is run when the robot registers a small can, and pushes the can off the side of the conveyor
void pushSmall() {
  //drive forward for half a second to avoid a false reading when passing over the belt
  motors.setSpeeds(motorspeed, motorspeed);
  delay(500);

  //drive forward until the robot detects a white line
  lineFound = false;
  while (lineFound == false) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    if (lineSensorValues[2] < threshHold) {
      motors.setSpeeds(0, 0);
      lineFound = true;
    }
    else {
      motors.setSpeeds(motorspeed, motorspeed);
    }
  }

  lineFound = false;

  //drive an small distance and reverse for half a second to avoid false reading when passing over the belt
  delay(200);
  motors.setSpeeds(-motorspeed, -motorspeed);
  delay(500);

  //drive backwards until the robot detects a white line again, meaning it has returned to the starting position
  while (lineFound == false) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    if (lineSensorValues[1] < threshHold && lineSensorValues[3] < threshHold) {
      motors.setSpeeds(0, 0);
      lineFound = true;
    }
    else {
      motors.setSpeeds(-motorspeed, -motorspeed);
    }

  }
}

//This function is run when the robot registers a large can, and pushes the can off the end of the conveyor
void pushLarge() {
  turnRight(90);
  //reset the counters;
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  
  /*In this do while loop, the Zumo will drive forward, as long as the condition is true. 
    The condition is that the encoders should be less than 1676 counts.*/
  do {                                
    motors.setSpeeds(100, 100);
    delay(100);
  }  while (encoders.getCountsLeft() < 1676 && encoders.getCountsRight() < 1676);

  /*The Zumo is now out of the do while loop and continues with the rest of the code.
    First the Zumo stops and then turn 90 degrees left.*/
  motors.setSpeeds(0, 0);
  delay(200);

  turnLeft(90);
  //reset the counters;
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  
  /*In this do while loop, the Zumo will drive forward, as long as the condition is true. 
   The condition is that the encoders should be less than 1209 counts.*/
  do {
    motors.setSpeeds(100, 100);
  } while (encoders.getCountsLeft() < 1209 && encoders.getCountsRight() < 1209);

  //nu er den ude ad "do-while" og fortsætter med resten af koden

  motors.setSpeeds(0, 0);
  delay(200);

  turnLeft(90);
  //reset the counters;
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  do {
    motors.setSpeeds(200, 200);
  } while (encoders.getCountsLeft() < 2350 && encoders.getCountsRight() < 2350);

  /*The Zumo is now out of the do while loop and continues with the rest of the code.
    First the Zumo stops.*/
  motors.setSpeeds(0, 0);
  delay(1000);

  /* Now, the rest of this part of the code, is for the Zumo 
     to drive forward until it finds a white line.*/
  lineFound = false;
  /* lineFound is equl false. If the line sensors read lineSensorValues[2] to be 
     lower than the treshols, the zumo will stop and lineFound will be equal true, 
     and therefore exit the loop. Else the Zumo will drive forward.
     The data the lineSensors read will be put into the array lineSensorValues,
     and here we want to look a the middel sensor, lineSensorValues[2]*/
  while (lineFound == false) {
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);            
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[2] < threshHold)  
    { motors.setSpeeds(0, 0);
      lineFound = true;                  
    } else {
      motors.setSpeeds(100, 100);       
    }
  }
  delay(500);
}

//This function turns left a number of degrees
void turnLeft(int degrees) {
  //reset the gyro and set the motors to turn
  turnSensorReset();
  motors.setSpeeds(-turnSpeed, turnSpeed);
  int angle = 0;

  //run this loop while the angle on the gyro is less than the desired angle
  do {
    delay(1);                                           //small delay
    turnSensorUpdate();                                 //update the gyro
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;   //calculate the gyro
    lcd.gotoXY(0, 0);                                   //write the angle to the LCD
    lcd.print(angle);
    lcd.print(" ");
  } while (angle < degrees);

  // set the motor speeds to 0
  motors.setSpeeds(0, 0);
}

//This function turns right a number of degrees
void turnRight(int degrees) {
  //reset the gyro and set the motors to turn
  turnSensorReset();
  motors.setSpeeds(turnSpeed, -turnSpeed);
  int angle = 0;

  //run this loop while the angle on the gyro is less than the desired angle
  do {
    delay(1);                                           //small delay
    turnSensorUpdate();                                 //update the gyro
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;   //calculate the gyro
    lcd.gotoXY(0, 0);                                   //write the angle to the LCD
    lcd.print(angle);
    lcd.print(" ");
  } while (angle > -degrees);

  // set the motor speeds to 0
  motors.setSpeeds(0, 0);
}

//This function returns the robot to the starting line
void driveBack(){
  encoders.getCountsAndResetLeft ();
  
  //while(encoders.getCountsLeft()<3000){ // 500 mm/ 0.138133 mm pr tic = 3000
    motors.setSpeeds (-100, -100);    //setSpeeds (int16_t leftSpeed, int16_t rightSpeed)
    delay(2500);
  //}
  motors.setSpeeds (0, 0); // Stopper while loop
  delay(500);
  
                      // step 2 turn 90 deg counter clockwise
  turnLeft(90);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();               
  
                      // step 3 Driveforward
  encoders.getCountsAndResetLeft ();
  // put your main code here, to run repeatedly:
  while(encoders.getCountsLeft()<1448){ // 200 mm /0.138133 mm pr tic = 1447.87
    motors.setSpeeds (100, 100);    //setSpeeds (int16_t leftSpeed, int16_t rightSpeed)
  }
  motors.setSpeeds (0, 0); // Stopper while loop
  delay(1000);             
  
                      // step 4 turn 90 deg clockwise
  turnRight(90);  
  /*encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();                 
  
                      // step 5 Driveforward
  encoders.getCountsAndResetLeft ();
  // put your main code here, to run repeatedly:
  while(encoders.getCountsLeft()<1448){ // 200 mm / 0.138133 mm pr tic = 1447.8799418
    motors.setSpeeds (100, 100);    //setSpeeds (int16_t leftSpeed, int16_t rightSpeed)
  }
  motors.setSpeeds (0, 0); // Stopper while loop
  delay(1000);*/
}

//--------------Setup--------------//
void setup() {
  //turn on line emitters and initialize line and prox sensors
  lineSensors.emittersOn ();
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  //begin serial for debugging
  Serial.begin(9600);

  //run the turnSensorSetup from turnSensor.cpp
  //this calibrates the gyro before the program begins
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  lcd.clear();

  //wait until the button is pressed before running the code
  buttonA.waitForPress();
  delay(500);

  //run the lineFind and followLine functions once before entering the loop
  //this aligns the robot with the belt before beginning the regular program
  lineFind();
  followLine(threshHold);
}

//--------------Loop---------------//
void loop() {
  //initialize the variable "type", and set it to 'N'
  char type;
  type = 'N';

  //use the detectType function to look for cans, and add it to "type"
  type = detectType();

  //if the can that is found is large, then run the relevant function for pushing a large can
  if (type == 'L'){
    pushLarge();              //push the can off the end of the belt
    driveBack();              //drive back towards the line
    lineFind();               //find the line and align the robot
    followLine(threshHold);   //follow the line to the belt controller
  }
  lcd.print(type);

  //if the can that is found is small, then run pushSmall
  if (type == 'S'){
    pushSmall();              //drive the can off the side of the belt and return
  }
  lcd.clear();
  delay(200);
}
