#include <Wire.h>
#include <math.h>


//////Pin mappings for sensors////////////
//////////////////////////////////////////
//input 9 is right side facing outwards
//input 10 is right 45
//input 11 is right forward
//
//
//input 8 is left side facing outward.
//input 7 is the left 45
//input 6 left front facing
//////////////////////////////////////////
int right90Pin = 9;
int right45Pin = 10;
int rightForwardPin = 11;

int left90Pin = 8;
int left45Pin = 7;
int leftForwardPin = 6;

//array to hold sensor voltages  
int sensorVoltage[6];

//array to hold sensor distances
double sensorDistances[6];

///// Right Wheel ///////
// Pin PWMA goes to pin 16
// Pin AIN2 goes to pin 17
// Pin AIN1 goes to pin 18
int pwmRightMotor  = 16;
int ain2RightMotor = 17;
int ain1RightMotor = 18;

//Global counter for encoders
//These are declared as volatile because they are modified inside
//an interrupt function
volatile unsigned int globalCounter = 0;
volatile unsigned int globalCounter2 = 0;

//Variable that determines the state of the encoder counters
volatile unsigned int state;

///// Left Wheel ////////
// Pin STBY goes to pin 30
// Pin BIN1 goes to pin 28
// Pin BIN2 goes to pin 29
// Pin PWMB goes to pin 27
int stbyHBridge   = 30;
int bin1LeftMotor = 28;
int bin2LeftMotor = 29;
int pwmLeftMotor  = 27;

//create a variable that will be make a counter
//where the difference between the last time and the current time
//will give a time measurement.  Use unsigned long to make sure we never
//get a negative lastTime.
unsigned long lastTime = 0;
unsigned long currentTime;
double timeChange;

//Variables for PID
double lastError = 0;
double change = 0;

//These will need to change based on experimentation...
double kp = 200.0;
double kd = 0.0;

//Set the base speed (May have to be changed...)
int baseSpeed = 20000;

//Set rightSpeed and leftSpeed equal to baseSpeed initially
int rightSpeed = baseSpeed;
int leftSpeed  = baseSpeed;

//set the sample time to 5ms due to response time of sensors
int sampleTime = 5;

//Variables to keep track of ticks while turning
int leftTurnTicks = 0;
int rightTurnTicks = 0;
boolean rightFlag = 0;
boolean leftFlag = 0;

//For driving by encoders 
int rightEncoderDriver = 0;


//Set up everything
void setup() {

  //Special note: for pins that are to be used as PWM pins, the declaration must be:
  //pinMode(PIN_#, PWM); where Pin# is a pwm pin.  See below for details
  //http://leaflabs.com/docs/lang/api/analogwrite.html
  
  //Right wheel setup
  pinMode(pwmRightMotor, PWM);
  pinMode(ain1RightMotor, OUTPUT);
  pinMode(ain2RightMotor, OUTPUT);

  //Left wheel setup
  pinMode(pwmLeftMotor, PWM);
  pinMode(bin1LeftMotor, OUTPUT);
  pinMode(bin2LeftMotor, OUTPUT);
  pinMode(bin2LeftMotor, OUTPUT);
  
  //Set standby pin
  pinMode(stbyHBridge, OUTPUT);

  //Interrupts for the LEFT wheel
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  attachInterrupt(19, encoderCounter2, CHANGE);
  attachInterrupt(20, encoderCounter2, CHANGE);

  //Interrupts for the RIGHT wheel
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  attachInterrupt(21, encoderCounter, CHANGE);
  attachInterrupt(22, encoderCounter, CHANGE);

  //enable interrupts
  interrupts();
  
  //Setup sensor pins for right sensors
  pinMode(right90Pin, INPUT_ANALOG);
  pinMode(right45Pin, INPUT_ANALOG);
  pinMode(rightForwardPin, INPUT_ANALOG);
  
  //Setup sensor pins for left sensors
  pinMode(right90Pin, INPUT_ANALOG);
  pinMode(right45Pin, INPUT_ANALOG);
  pinMode(leftForwardPin, INPUT_ANALOG);
  
    //Serial1.begin(9600);
 
}

void loop(){
  //create a timer to check how long it has been.  Better than using delay which
  //stops the whole operation of the main loop
  currentTime = millis();
      
  //compute how long it has been since last we checked
  timeChange = (currentTime - lastTime);

  //Sample time is for when the sensors are added, but the motor code will go inside
  //this if statement for now
  if (timeChange >= sampleTime){

    //Set the lastTime to the currentTime so that we can reset the timer
    lastTime = currentTime;
    
    //Get the voltage values from the right sensors
    sensorVoltage[0] = analogRead(right90Pin);
    sensorVoltage[1] = analogRead(right45Pin);
    sensorVoltage[2] = analogRead(rightForwardPin);
    
    //Get the voltage values from the left sensors
    sensorVoltage[3] = analogRead(left90Pin);
    sensorVoltage[4] = analogRead(left45Pin);
    sensorVoltage[5] = analogRead(leftForwardPin);
  
    //Convert the voltages to distances
    sensorDistances[0] = right90Dist(sensorVoltage[0]);
    sensorDistances[3] = left90Dist(sensorVoltage[3]);
    
    //Compute the PID
    //pass arguements by refernece 
    computePID(&leftSpeed, &rightSpeed, &lastError, &change, baseSpeed, kp, kd); 
  
    //Drive the motors based on the righspeed and leftspeed obtained above
    digitalWrite(bin1LeftMotor, LOW);
    digitalWrite(bin2LeftMotor, HIGH);
    pwmWrite(pwmLeftMotor, leftSpeed);
        SerialUSB.print(leftSpeed);
       	SerialUSB.print("\t"); 
        SerialUSB.println(rightSpeed);  
     
     //pwmWrite(pwmLeftMotor, 600);
    
    digitalWrite(ain1RightMotor, LOW);
    digitalWrite(ain2RightMotor, HIGH);
    pwmWrite(pwmRightMotor, rightSpeed);
    //pwmWrite(pwmRightMotor, 600);
    
    digitalWrite(stbyHBridge, HIGH);
    
  }
}

//Keeps track of encoder ticks for the left motor
void encoderCounter2(){
   switch (state)
  {
    case 0:
      globalCounter2++;
    break; 
    case 1:
      leftTurnTicks++;
   break;
  }
 
  return;
}

//Keeps track of encoder ticks for the right motor
void encoderCounter(){
   switch (state)
  {
    case 0:
      globalCounter++;
    break; 
    case 1:
      rightTurnTicks++;
   break;
  }
  return;
}


double left90Dist(int x) {
  double y;

  
  if(x <= 2957){
        y = 1.301142e-8*x*x*x - 8.262500e-5*x*x + 1.815567e-1*x -1.361187e2;
  }
  
  if(x > 2957){
        y = 2.410786e-7*x*x*x - 1.654148e-3*x*x + 3.496635*x - 2.093753e3;
  }
  
  
  return y;
}

double right90Dist(int x) {
  double y;
  
  if(x <= 2736){
        y = 1.841298e-8*x*x*x - 1.013821e-4*x*x + 1.914072e-1*x -1.212412e2;
  }
  
  if(x > 2736){
        y = 3.466274e-7*x*x*x - 1.281139e-3*x*x - 7.847033e-1*x + 4.660555e3;
  }
  
  
  return y;
}




