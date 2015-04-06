



//////////////////////////////////////////////////////////////
//Computes PID
//rightSpeed: pwm to drive right wheel
//leftSpeed:  pwm to drive left wheel
//lastError:  keeps track of error from last function call 
/////////////////////////////////////////////////////////////
void computePID(int *leftSpeed, int *rightSpeed, double *lastError, double *change, int baseSpeed, double kp, double kd){
   
  double frontSensorDifference;
        
  //turnVoltageCheck = analogRead(A3);                 
  //compute the front sensor difference and send the values to computePID
  //if(sensorDistances[0] <= 9){
  //  frontSensorDifference = sensorDistances[0] - 6.31;
  //}else{
    frontSensorDifference = 10.52 - sensorDistances[3];
  //}
  //SerialUSB.println(frontSensorDifference);
  *change = frontSensorDifference * kp; //+ (frontSensorDifference - *lastError) * kd;
  //*change = 0;
  *rightSpeed = baseSpeed - *change;
  *leftSpeed = baseSpeed + *change;
     

  //reset the lastError to frontDifference for the next function call
  *lastError = frontSensorDifference;
}

//////////////////////////////////////////////////////
//Function driveMotors 
/////////////////////////////////////////////////////
void driveMotor(int PWMSpeed, int PWMpin, boolean In1, boolean In2, int In1Pin, int In2Pin){
  
  //Rotate the wheel
  digitalWrite(In1Pin, In1);
  digitalWrite(In2Pin, In2);
  pwmWrite(PWMpin, PWMSpeed);
}

//Simple function that stops the left wheel
void stopLeftWheel(){
    digitalWrite(bin1LeftMotor, HIGH);
    digitalWrite(bin2LeftMotor, HIGH);
    pwmWrite(pwmLeftMotor, 600);

}

//Simple function that stops the right wheel
void stopRightWheel(){
    digitalWrite(ain2RightMotor, HIGH);
    digitalWrite(ain1RightMotor, HIGH);
    pwmWrite(pwmRightMotor, 600);
}

void turn90Clk(){
     //This will have to be found experimentally
     int numberOfTicks90 = 53;
  
    //Stop the right and left wheels 
    stopRightWheel();
    stopLeftWheel();
  
    //Set state to turn
    state = 1;
  
    //Wait a second
    delay(100);  
    leftTurnTicks = 0;
    rightTurnTicks = 0;

    //Rotate both wheels until we have made the whole 90 degree turn
     while(leftTurnTicks < numberOfTicks90){
      if(globalCounter <= (1) && !rightFlag){ 
        //Rotate the right wheel forward
        digitalWrite(ain1RightMotor, HIGH);
        digitalWrite(ain2RightMotor, LOW);
        pwmWrite(pwmRightMotor, 1200);
      }else{
        //Stop the right wheel and wait for the left
        stopRightWheel();
        rightFlag = 1;
        globalCounter = 0;
     }
     if(globalCounter2 <= (1) && !leftFlag){
        //Rotate left wheel backwards
        digitalWrite(bin1LeftMotor, LOW);
        digitalWrite(bin2LeftMotor, HIGH);
        pwmWrite(pwmLeftMotor, 1200);
     }else{
        //Stop the left wheel and wait for the right
        stopLeftWheel();
        globalCounter2 = 0;
        leftFlag = 1;
     }
      //Reset flags 
      if(leftFlag && rightFlag){
        leftFlag = 0;
        rightFlag = 0;
      }  
     }
     
    //Stop the right and left wheels 
    stopRightWheel();
    stopLeftWheel();
    delay(100);
}







