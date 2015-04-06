#include <Wire.h>
#include <math.h>

/////  Sensor Values ///////
double arrayOfVoltageConvert[6];
int arrayOfVoltage[6];
	

///// Right Wheel ///////
// Pin PWMA goes to pin 16
// Pin AIN2 goes to pin 17
// Pin AIN1 goes to pin 18
int pwmRightMotor = 16;
int ain2RightMotor = 17;
int ain1RightMotor = 18;

//For Testing motors
int pwmCounter = 20000;

//Global counter for encoders
volatile int globalCounter1 = 0;
volatile int globalCounter2 = 0;


///// Left Wheel ////////
// Pin STBY goes to pin 30
// Pin BIN1 goes to pin 28
// Pin BIN2 goes to pin 29
// Pin PWMB goes to pin 27
int stbyHBridge = 30;
int bin1LeftMotor = 28;
int bin2LeftMotor = 29;
int pwmLeftMotor = 27;

//// Sensor Pin Mapping /////
int analogInput0 = 11;
int analogInput1 = 10;
int analogInput2 = 9;
int analogInput3 = 8;
int analogInput4 = 7;
int analogInput5 = 6;


//Create a 16x16 array to keep track of where we have visited initialized to zero
//Let 0 be unvisited
//Let 1 be visited
//Let 2 be a dead end
//int[16][16] expMaze = {};
/*int[16][16] distMaze = {};
int[16][16] maze = {};
int visited[16][16] = {};
*/

//PID values
double kp = 80.0;
double kd = 30.00;

//Set the base speed
int baseSpeed = 1200;

//create a variable that will be make a counter
//where the difference between the last time and the current time
//will give a time measurement.  Use unsigned long to make sure we never
//get a negative lastTime.
unsigned long lastTime = 0;
unsigned long currentTime;
double timeChange;
double lastError = 0;
double backDifference = 0;
double totalDifference = 0;
double change = 0;

//set the sample time to 5ms due to response time of sensors
int sampleTime = 5;

void setup() {
  pinMode(analogInput0, INPUT_ANALOG);
  pinMode(analogInput1, INPUT_ANALOG);
  pinMode(analogInput2, INPUT_ANALOG);
  pinMode(analogInput3, INPUT_ANALOG);
  pinMode(analogInput4, INPUT_ANALOG);
  pinMode(analogInput5, INPUT_ANALOG);


  //Right wheel setup
  //Special note: for pins that are to be used as PWM pins, the declaration must be:
  //pinMode(PIN_#, PWM); where Pin# is a pwm pin.  See below for details
  //http://leaflabs.com/docs/lang/api/analogwrite.html
  pinMode(pwmRightMotor, PWM);
  pinMode(ain1RightMotor, OUTPUT);
  pinMode(ain2RightMotor, OUTPUT);

  //Left wheel setup
  pinMode(pwmLeftMotor, PWM);
  pinMode(bin1LeftMotor, OUTPUT);
  pinMode(bin2LeftMotor, OUTPUT);
  pinMode(bin2LeftMotor, OUTPUT);
  pinMode(stbyHBridge, OUTPUT);
  pinMode(BOARD_LED_PIN, OUTPUT);      // sets the digital pin as output

  //Interrupts for the LEFT wheel
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  attachInterrupt(8, encoderCounter2, CHANGE);
  attachInterrupt(9, encoderCounter2, CHANGE);

  //Interrupts for the RIGHT wheel
  
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  attachInterrupt(20, encoderCounter, CHANGE);
  attachInterrupt(21, encoderCounter, CHANGE);
  Serial1.begin(9600);

  //enable interrupts
  interrupts();
  delay(100);

}

void loop(){
  //create a timer to check how long it has been.  Better than using delay which
  //stops the whole operation of the main loop
  currentTime = millis();
    
    /*
    digitalWrite(ain1RightMotor, HIGH);
    digitalWrite(ain2RightMotor, LOW);
    digitalWrite(bin1LeftMotor, HIGH);
    digitalWrite(bin2LeftMotor, LOW);

     //Set standby to HIGH so 
    digitalWrite(stbyHBridge, HIGH);
    
    
    pwmWrite(pwmLeftMotor, pwmCounter);
    pwmWrite(pwmRightMotor, pwmCounter);
    */
  
  
  //compute how long it has been since last we checked
  timeChange = (currentTime - lastTime);

    //If we have reached our sample time, implement PID
  //if (timeChange >= sampleTime){

    //Set the lastTime to the currentTime so that we can reset the timer
    lastTime = currentTime;

    //Drive the two motors forward
    digitalWrite(ain1RightMotor, HIGH);
    digitalWrite(ain2RightMotor, LOW);
    digitalWrite(bin1LeftMotor, HIGH);
    digitalWrite(bin2LeftMotor, LOW);

     //Set standby to HIGH so 
    digitalWrite(stbyHBridge, HIGH);
    
    //Rotate the shaft exactly 10 times
    //if(globalCounter1 < (512*10)){
      //Drive the motor with a pwm signal
      pwmWrite(pwmLeftMotor, 20000);
      pwmWrite(pwmRightMotor, 20000);
    //}else{
      
      //Stop the spinning if it has already rotated 10 times
      //At a PWM of 600 or less the shaft stops spinning
     // pwmWrite(pwmLeftMotor, 600);
     // pwmWrite(pwmRightMotor, 600); 
    //}


  //Get the voltage values
  //arrayOfVoltage[0] = analogRead(analogInput0);
  //arrayOfVoltage[1] = analogRead(analogInput1);
  //arrayOfVoltage[2] = analogRead(analogInput2);
  //arrayOfVoltage[3] = analogRead(analogInput3);
  //arrayOfVoltage[4] = analogRead(analogInput4);
  //arrayOfVoltage[5] = analogRead(analogInput5);
/*
  if( analogInput4 < 1415 ) {
    */

    /*
  }
  else {
    pwmWrite(pwmLeftMotor, 0);
    pwmWrite(pwmRightMotor, 0);
  }
  */
  //convert each sensor voltage to distance
  /*arrayOfVoltageConvert[0] = distanceFrontRight(arrayOfVoltage[0]);
  arrayOfVoltageConvert[1] = distanceFrontRight(arrayOfVoltage[1]);
  arrayOfVoltageConvert[2] = distanceBackRight(arrayOfVoltage[2]);
  arrayOfVoltageConvert[3] = distanceBackLeft(arrayOfVoltage[3]);
  arrayOfVoltageConvert[4] = distanceFrontLeft(arrayOfVoltage[4]);
  arrayOfVoltageConvert[5] = distanceFrontRight(arrayOfVoltage[5]);
  */

  //Use encoders for when there is not a wall present on either side.
    //computePID(&rightSpeed, &leftSpeed, &lastError, &change, baseSpeed, kp, kd);

    //Drive the left motor forward
    //driveMotor(leftSpeed, leftPWM, HIGH, LOW, leftIn1, leftIn2);

    //Drive the right wheel forward
   // driveMotor(rightSpeed, rightPWM, LOW, HIGH, rightIn1, rightIn2);


  //}
}

void encoderCounter2(){
  globalCounter2++;
  return;
}
void encoderCounter(){
  globalCounter1++;
  return;
}

