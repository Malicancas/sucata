//ROS2 CONTROL COMPATIBLE FIRMWARE FOR SEEEDSTUDIO HERCULES 4WD CONTROLLER
//Based on MECHANICAPE.NL firmware by Rein Velt / http://mechanicape.nl        
//Modified for ROS2 Control compatibility
//=====================================================================
// The device is controlled by serial commands compatible with ros2_control.
// The commands, descriptions and responses are:
//  CMD     DESCRIPTION                     RESPONSE
//  M0      Motor OFF                      -returns M0,speedL,speedR\n
//  M1      Motor Forward                  -returns M1,speedL,speedR\n
//  M2      Motor Reverse                  -returns M2,speedL,speedR\n
//  M3      Motor Turn Left (ccw)          -returns M3,speedL,speedR\n
//  M4      Motor Turn Right (cw)          -returns M4,speedL,speedR\n
//  MT      Motor Test (test motors)       -returns MT,speedL,speedR\n
//  S0..S9  Speed (0=stop,.., 9=fast)      -returns S*,speedL,speedR\n
//  B?      Query battery voltage          -returns B?,millivolts\n
//  
//  ROS2 Control specific commands:
//  r       Read encoder values            -returns encoder_left,encoder_right\n
//  e       Reset encoder values           -returns ok\n
//  m<L>,<R> Set motor RPM values          -returns ok\n
//  p<P>,<D>,<I>,<O> Set PID values        -returns ok\n
//
//  the comma is used to separate commands when more are following
//  the newline is used to terminate one or more commands
//  Examples:
//      M1\n 
//      r\n
//      m50,-30\n
//      p2.0,1.0,0.5,25.0\n
//====================================================================

#include <Arduino.h>
#include <avr/wdt.h>

// pin ctrl
#define PINRX           0           // receive serial data
#define PINTX           1           // send serial data
#define PININT0         2           // interrupt 0
#define PININT1         3           // interrupt 1
#define PINCS           6           // all motors cs
#define PINM1F          4           // motor 1 forward
#define PINM1R          5           // motor 1 reverse
#define PINM2F          7           // motor 2 forward
#define PINM2R          8           // motor 2 reverse
#define PINPWMA         9           // PWM channel A (motor A speed)
#define PINPWMB         10          // PWM channel B (motor B speed)
#define PINVS           A6          // voltage/battery sensor

void leftInterruptHandler();
void rightInterruptHandler();
void resetFeedback();
void resetEncoders();
void motorSetSpeed(int power);
void motorStop();
void motorForward();
void motorReverse();
void motorTurnLeft();
void motorTurnRight();
void processAllSerial();
int getBatteryVoltage();
void motorTest();
void setMotorRPM(int leftRPM, int rightRPM);
void setPIDValues(float p, float d, float i, float o);
void sendEncoderValues();

//global vars
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
volatile int leftFeedback = 0;
volatile int rightFeedback = 0;
int motorSpeed=0;
int leftMotorRPM = 0;
int rightMotorRPM = 0;

// Motor direction tracking (simple approach)
volatile bool leftMotorForward = true;
volatile bool rightMotorForward = true;

// PID variables
float pid_p = 2.0;
float pid_d = 1.0;
float pid_i = 0.5;
float pid_o = 25.0;

// Motor control variables
unsigned long lastMotorUpdate = 0;
const unsigned long motorUpdateInterval = 50; // 50ms update interval


void setup()
{
  //initialize pins (make them output) 
  pinMode(PINCS, OUTPUT);
  pinMode(PINM1F, OUTPUT);
  pinMode(PINM1R, OUTPUT);
  pinMode(PINM2F, OUTPUT);
  pinMode(PINM2R, OUTPUT);
  pinMode(PINVS,  INPUT);

  //initialize motors
  motorStop();

  //initialize interrupts
  attachInterrupt(0, leftInterruptHandler, CHANGE);
  attachInterrupt(1, rightInterruptHandler, CHANGE);

  //initialize serial port
  Serial.begin(57600); // Increased baud rate for ROS2 compatibility
  Serial.println("//ROS2 CONTROL HERCULES 4WD ROBOTFIRMWARE V2.0");
  Serial.print("//Battery=");
  Serial.print(getBatteryVoltage());
  Serial.print("mV");
  Serial.println();
  Serial.println("OK");

  //initialize watch dog timer and set it to 2 seconds
  wdt_enable(WDTO_2S);
}


void loop()
{
  wdt_reset();  //reset watch dog timer
  
  // Process all serial communication in one place
  processAllSerial();
  
  resetFeedback();  //reset speed sensors
  
  // Update motor control at regular intervals
  if (millis() - lastMotorUpdate >= motorUpdateInterval) {
    // Here you could implement PID control for precise RPM control
    lastMotorUpdate = millis();
  }
}


//**** INTERRUPT HANDLER ******************************************************************************


void leftInterruptHandler()
{
  leftFeedback++;
  // Increment or decrement based on motor direction
  if (leftMotorForward) {
    leftEncoder++;
  } else {
    leftEncoder--;
  }
}


void rightInterruptHandler()
{
  rightFeedback++;
  // Increment or decrement based on motor direction
  if (rightMotorForward) {
    rightEncoder++;
  } else {
    rightEncoder--;
  }
}


void resetFeedback()
{
  leftFeedback=0;
  rightFeedback=0;
}

void resetEncoders()
{
  leftEncoder = 0;
  rightEncoder = 0;
}


//**** MOTOR CONTROLLER *******************************************************************************


void motorSetSpeed(int power)
{
  motorSpeed=power;
  analogWrite(PINPWMA,motorSpeed);
  analogWrite(PINPWMB,motorSpeed);
}


void motorStop()
{
  digitalWrite(PINCS,LOW);
  digitalWrite(PINM1R,LOW);
  digitalWrite(PINM1F,LOW);
  digitalWrite(PINM2F,LOW);
  digitalWrite(PINM2R,LOW); 
}


void motorForward()
{
  digitalWrite(PINM1R,HIGH);
  digitalWrite(PINM1F,LOW);
  digitalWrite(PINM2F,HIGH);
  digitalWrite(PINM2R,LOW);
  digitalWrite(PINCS,HIGH);
  // Update direction tracking
  leftMotorForward = true;
  rightMotorForward = true;
}

void motorReverse()
{
  digitalWrite(PINM1R,LOW);
  digitalWrite(PINM1F,HIGH);
  digitalWrite(PINM2F,LOW);
  digitalWrite(PINM2R,HIGH);
  digitalWrite(PINCS,HIGH);
  // Update direction tracking
  leftMotorForward = false;
  rightMotorForward = false;
}


void motorTurnLeft()
{
  digitalWrite(PINM1R,LOW);
  digitalWrite(PINM1F,HIGH);
  digitalWrite(PINM2F,HIGH);
  digitalWrite(PINM2R,LOW);
  digitalWrite(PINCS,HIGH);
  // Update direction tracking
  leftMotorForward = false;  // Left motor reverse
  rightMotorForward = true;  // Right motor forward
}


void motorTurnRight()
{
  digitalWrite(PINM1R,HIGH);
  digitalWrite(PINM1F,LOW);
  digitalWrite(PINM2F,LOW);
  digitalWrite(PINM2R,HIGH);
  digitalWrite(PINCS,HIGH);
  // Update direction tracking
  leftMotorForward = true;   // Left motor forward
  rightMotorForward = false; // Right motor reverse
}


//**** MESSAGE HANDLING ******************************************************************************


void processAllSerial()
{
  static unsigned long lastCommandTime = 0;
  
  if (Serial.available() > 0)
  {
    lastCommandTime = millis();
    
    // Try to read ROS2 style commands first (line-based)
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      // Handle ROS2 commands
      if (command.startsWith("r")) {
        // Read encoder values
        sendEncoderValues();
        return;
      }
      else if (command.startsWith("e")) {
        // Reset encoder values
        resetEncoders();
        Serial.println("ok");
        return;
      }
      else if (command.startsWith("m")) {
        // Set motor RPM values: m<left_rpm>,<right_rpm>
        int commaIndex = command.indexOf(',');
        if (commaIndex > 1) {
          int leftRPM = command.substring(1, commaIndex).toInt();
          int rightRPM = command.substring(commaIndex + 1).toInt();
          setMotorRPM(leftRPM, rightRPM);
          Serial.println("ok");
          return;
        }
      }
      else if (command.startsWith("p")) {
        // Set PID values: p<P>,<D>,<I>,<O>
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);
        
        if (firstComma > 1 && secondComma > firstComma && thirdComma > secondComma) {
          float p = command.substring(1, firstComma).toFloat();
          float d = command.substring(firstComma + 1, secondComma).toFloat();
          float i = command.substring(secondComma + 1, thirdComma).toFloat();
          float o = command.substring(thirdComma + 1).toFloat();
          setPIDValues(p, d, i, o);
          Serial.println("ok");
          return;
        }
      }
    }
  }
  
  // Handle legacy 3-byte commands if no ROS2 command was processed
  if (Serial.available() >= 3)
  {
    lastCommandTime = millis();
    int commandByte = Serial.read();
    int valueByte = Serial.read();
    int endofline = Serial.read();
    
    // Handle motor commands (M0, M1, M2, M3, M4, MT)
    if (commandByte == 77 && (endofline == 10 || endofline == 44)) // M
    {
      motorSetSpeed(motorSpeed);      
      switch (valueByte)
      {
        case 48+0: motorStop(); break;        // M0
        case 48+1: motorForward(); break;     // M1
        case 48+2: motorReverse(); break;     // M2
        case 48+3: motorTurnLeft(); break;    // M3
        case 48+4: motorTurnRight(); break;   // M4
        case 84: motorTest(); break;          // MT
        default: motorStop(); break;          // fail safe
      }

      // Confirm the serial request by sending a response
      Serial.print(char(commandByte));
      Serial.print(char(valueByte));
      Serial.print(char(44));
      Serial.print(leftFeedback);
      Serial.print(char(44));
      Serial.print(rightFeedback);
      Serial.println();
    } 
    
    // Handle speed commands (S0-S9)
    else if (commandByte == 83 && (endofline == 10 || endofline == 44)) // S
    {
      if (valueByte > 47 && valueByte < 59)
      { 
        int motorNewSpeed = (valueByte - 48) * 26;
        motorSetSpeed(motorNewSpeed);
      }
      Serial.print(char(commandByte));
      Serial.println(char(valueByte));
    } 

    // Handle battery voltage request (B?)
    else if (commandByte == 66 && (endofline == 10 || endofline == 44)) // B
    {
      int millivolt = getBatteryVoltage();
      Serial.print(char(commandByte));
      Serial.print(char(valueByte));
      Serial.print(char(44));
      Serial.print(millivolt);
      Serial.println();
    }
  }
  else if (millis() - lastCommandTime > 1000) // 1 second timeout
  {
    // Timeout handling - don't stop motors for ROS2 compatibility
  }
}



//***** SENSOR HANDLING ******************************************************************************

int getBatteryVoltage()
{

  int value=analogRead(PINVS);
  int millivolt=round((float)(value)/0.037479); //guess
  return millivolt;
}
//***** SYSTEM TEST **********************************************************************************

void motorTest()
{
  for (int i=0;i<255;i++)
  {
    wdt_reset();
    motorSetSpeed(i);
    motorTurnRight();
    delay(50) ;
  }
  for (int i=255;i>-1;i--)
  {
    wdt_reset();
    motorSetSpeed(i);
    motorTurnRight();
    delay(50) ;
  }
  motorStop();
}

//***** ROS2 CONTROL SPECIFIC FUNCTIONS *************************************************************

void setMotorRPM(int leftRPM, int rightRPM)
{
  leftMotorRPM = leftRPM;
  rightMotorRPM = rightRPM;
  
  // Update direction tracking based on RPM sign
  leftMotorForward = (leftRPM >= 0);
  rightMotorForward = (rightRPM >= 0);
  
  // Convert RPM to PWM values (this is a simplified conversion)
  // You may need to adjust this based on your motor characteristics
  int leftPWM = abs(leftRPM) * 255 / 20;  // Assuming max 20 RPM
  int rightPWM = abs(rightRPM) * 255 / 20;
  
  // Limit PWM values
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);
  
  // Set motor directions and speeds
  if (leftRPM >= 0) {
    digitalWrite(PINM1R, HIGH);
    digitalWrite(PINM1F, LOW);
  } else {
    digitalWrite(PINM1R, LOW);
    digitalWrite(PINM1F, HIGH);
  }
  
  if (rightRPM >= 0) {
    digitalWrite(PINM2F, HIGH);
    digitalWrite(PINM2R, LOW);
  } else {
    digitalWrite(PINM2F, LOW);
    digitalWrite(PINM2R, HIGH);
  }
  
  // Enable motors if any RPM is non-zero
  if (leftRPM != 0 || rightRPM != 0) {
    digitalWrite(PINCS, HIGH);
    analogWrite(PINPWMA, leftPWM);
    analogWrite(PINPWMB, rightPWM);
  } else {
    motorStop();
  }
}

void setPIDValues(float p, float d, float i, float o)
{
  pid_p = p;
  pid_d = d;
  pid_i = i;
  pid_o = o;
}

void sendEncoderValues()
{
  Serial.print(leftEncoder);
  Serial.print(",");
  Serial.print(rightEncoder);
  Serial.println();
}

//*****************************************************************************************************
// * END FILE
// *****************************************************************************************************




