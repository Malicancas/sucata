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



//global vars
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
volatile int leftFeedback = 0;
volatile int rightFeedback = 0;
int motorSpeed=0;

// PID variables
float pidP = 1.0, pidI = 0.1, pidD = 0.05, pidO = 0.0;  // More conservative values
int targetRpmLeft = 0, targetRpmRight = 0;
int currentRpmLeft = 0, currentRpmRight = 0;
float leftError = 0, rightError = 0;
float leftErrorSum = 0, rightErrorSum = 0;
float leftLastError = 0, rightLastError = 0;
int leftPwm = 0, rightPwm = 0;
unsigned long lastTime = 0;
const int ENCODER_PULSES_PER_REV = 20;  // Adjust based on your encoder


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
  Serial.begin(19200);
  Serial.println("HERCULES 4WD ROBOTFIRMWARE V1.1");
  Serial.print("//Bateria=");
  Serial.print(getBatteryVoltage());
  Serial.print("mV");
  Serial.println();
  Serial.println("OK");

  //initialize watch dog timer and set it to 2 seconds
  wdt_enable(WDTO_2S);
  
  // Initialize PID timer
  lastTime = millis();
}


void loop()
{
  wdt_reset();  //reset watch dog timer
  processSerialData(); //read data from serial and send it to the motors
  updatePID(); //update PID control
  resetFeedback();  //reset speed sensors
}


//**** INTERRUPT HANDLER ******************************************************************************


void leftInterruptHandler()
{
  leftFeedback++;
  leftEncoder++;
}


void rightInterruptHandler()
{
  rightFeedback++;
  rightEncoder++;
}


void resetFeedback()
{
  leftFeedback=0;
  rightFeedback=0;
}


//**** MOTOR CONTROLLER *******************************************************************************

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  // Limit PWM values
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Set direction and speed for left motor
  if (leftSpeed >= 0) {
    digitalWrite(PINM1R, HIGH);
    digitalWrite(PINM1F, LOW);
    analogWrite(PINPWMA, leftSpeed);
  } else {
    digitalWrite(PINM1R, LOW);
    digitalWrite(PINM1F, HIGH);
    analogWrite(PINPWMA, -leftSpeed);
  }
  
  // Set direction and speed for right motor
  if (rightSpeed >= 0) {
    digitalWrite(PINM2F, HIGH);
    digitalWrite(PINM2R, LOW);
    analogWrite(PINPWMB, rightSpeed);
  } else {
    digitalWrite(PINM2F, LOW);
    digitalWrite(PINM2R, HIGH);
    analogWrite(PINPWMB, -rightSpeed);
  }
  
  digitalWrite(PINCS, HIGH);
}

void setMotorRpm(int leftRpm, int rightRpm)
{
  targetRpmLeft = leftRpm;
  targetRpmRight = rightRpm;
}

void updatePID()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;
  
  if (deltaTime >= 100) { // Update every 100ms
    // Calculate current RPM based on encoder feedback
    // RPM = (pulses_per_period * 60000) / (pulses_per_rev * period_ms)
    currentRpmLeft = (leftFeedback * 60000) / (ENCODER_PULSES_PER_REV * deltaTime);
    currentRpmRight = (rightFeedback * 60000) / (ENCODER_PULSES_PER_REV * deltaTime);
    
    // Only run PID if we have target RPM set
    if (targetRpmLeft != 0 || targetRpmRight != 0) {
      // Calculate errors
      leftError = targetRpmLeft - currentRpmLeft;
      rightError = targetRpmRight - currentRpmRight;
      
      // Calculate integral
      leftErrorSum += leftError * (deltaTime / 100.0);  // Scale by time
      rightErrorSum += rightError * (deltaTime / 100.0);
      
      // Limit integral windup
      leftErrorSum = constrain(leftErrorSum, -1000, 1000);
      rightErrorSum = constrain(rightErrorSum, -1000, 1000);
      
      // Calculate derivative
      float leftErrorDelta = (leftError - leftLastError) / (deltaTime / 100.0);
      float rightErrorDelta = (rightError - rightLastError) / (deltaTime / 100.0);
      
      // Calculate PID output
      leftPwm = (pidP * leftError) + (pidI * leftErrorSum) + (pidD * leftErrorDelta) + pidO;
      rightPwm = (pidP * rightError) + (pidI * rightErrorSum) + (pidD * rightErrorDelta) + pidO;
      
      // Apply motor speeds
      setMotorSpeed(leftPwm, rightPwm);
      
      // Store for next iteration
      leftLastError = leftError;
      rightLastError = rightError;
    }
    
    lastTime = currentTime;
  }
}


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
}

void motorReverse()
{
  digitalWrite(PINM1R,LOW);
  digitalWrite(PINM1F,HIGH);
  digitalWrite(PINM2F,LOW);
  digitalWrite(PINM2R,HIGH);
  digitalWrite(PINCS,HIGH);
}


void motorTurnLeft()
{
  digitalWrite(PINM1R,LOW);
  digitalWrite(PINM1F,HIGH);
  digitalWrite(PINM2F,HIGH);
  digitalWrite(PINM2R,LOW);
  digitalWrite(PINCS,HIGH);
}


void motorTurnRight()
{
  digitalWrite(PINM1R,HIGH);
  digitalWrite(PINM1F,LOW);
  digitalWrite(PINM2F,LOW);
  digitalWrite(PINM2R,HIGH);
  digitalWrite(PINCS,HIGH);

}


//**** MESSAGE HANDLING ******************************************************************************


void processSerialData()
{
  // Handle string commands (for RPM control and encoder reading)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      // RPM command: "R120,-80" (left RPM, right RPM)
      if (command.startsWith("R")) {
        int commaIndex = command.indexOf(',');
        if (commaIndex > 0) {
          int leftRpm = command.substring(1, commaIndex).toInt();
          int rightRpm = command.substring(commaIndex + 1).toInt();
          setMotorRpm(leftRpm, rightRpm);
          Serial.print("R,");
          Serial.print(leftRpm);
          Serial.print(",");
          Serial.println(rightRpm);
        }
        return;
      }
      
      // Encoder read command: "E"
      if (command == "E") {
        Serial.print("E,");
        Serial.print(leftEncoder);
        Serial.print(",");
        Serial.println(rightEncoder);
        return;
      }
      
      // PID settings command: "P1.0,0.1,0.05,0.0" (P,I,D,O)
      if (command.startsWith("P")) {
        int comma1 = command.indexOf(',');
        int comma2 = command.indexOf(',', comma1 + 1);
        int comma3 = command.indexOf(',', comma2 + 1);
        
        if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
          pidP = command.substring(1, comma1).toFloat();
          pidI = command.substring(comma1 + 1, comma2).toFloat();
          pidD = command.substring(comma2 + 1, comma3).toFloat();
          pidO = command.substring(comma3 + 1).toFloat();
          
          // Reset PID state when parameters change
          leftErrorSum = 0;
          rightErrorSum = 0;
          leftLastError = 0;
          rightLastError = 0;
          
          Serial.print("P,");
          Serial.print(pidP);
          Serial.print(",");
          Serial.print(pidI);
          Serial.print(",");
          Serial.print(pidD);
          Serial.print(",");
          Serial.println(pidO);
        }
        return;
      }
      
      // Status command: "S" - returns current status
      if (command == "S") {
        Serial.print("S,");
        Serial.print(currentRpmLeft);
        Serial.print(",");
        Serial.print(currentRpmRight);
        Serial.print(",");
        Serial.print(targetRpmLeft);
        Serial.print(",");
        Serial.println(targetRpmRight);
        return;
      }
    }
  }
  
  // Original 3-byte command handling (legacy support)
  if (Serial.available() >= 3)
  {
    int commandByte= Serial.read();
    int valueByte  = Serial.read();
    int endofline  = Serial.read();
    
    //handle the motor request
    if (commandByte==77 && (endofline==10 || endofline==44)) //M
    {
      // Disable PID when using legacy commands
      targetRpmLeft = 0;
      targetRpmRight = 0;
      
      motorSetSpeed(motorSpeed);      
      switch (valueByte)
      {
      case 48+0: 
        motorStop(); 
        break;       //M0\n
      case 48+1: 
        motorForward(); 
        break;      //M1\n
      case 48+2: 
        motorReverse(); 
        break;      //M2\n
      case 48+3: 
        motorTurnLeft(); 
        break;     //M3\n
      case 48+4: 
        motorTurnRight(); 
        break;     //M4\n
      case 84:
        motorTest(); //MT\n
        break;
      default: 
        motorStop();  
        break;    //fail safe
      }

      Serial.print(char(commandByte));
      Serial.print(char(valueByte));
      Serial.print(char(44));
      Serial.print(leftFeedback);
      Serial.print(char(44));
      Serial.print(rightFeedback);
      Serial.println();
    } 
  
    //handle the motorspeed request
    if (commandByte==83 && (endofline==10 || endofline==44)) //S
    {
      if (valueByte>47 && valueByte<59)
      { 
        int motorNewSpeed=(valueByte-48)*26;
        motorSetSpeed(motorNewSpeed);
      }
      Serial.print(char(commandByte));
      Serial.println(char(valueByte));
    } 

    //handle the battery voltage request
    if (commandByte==66 && (endofline==10 || endofline==44)) //B
    {
      int millivolt=getBatteryVoltage();
      Serial.print(char(commandByte));
      Serial.print(char(valueByte));
      Serial.print(char(44));
      Serial.print(millivolt);
      Serial.println();
    } 
  }
  else
  {
    //no data - don't stop motors if using RPM control
    if (targetRpmLeft == 0 && targetRpmRight == 0) {
      motorStop();  //fail safe only if not in RPM mode
    }
  }
  delay(10); // Reduced delay for better PID performance
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
//*****************************************************************************************************
// * END FILE
// *****************************************************************************************************




