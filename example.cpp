
void turn(double theta, bool isPossessingItem){

   double turn_speed = 0;
   double inaccuracyRange = 0.1;
   double kp = 1.2; //proportional constant
   double ki = 0.00005; //integration constant
   double kd = 0.9; //derivative constant


  if(isPossessing){ //center of mass impacted by certain game elements, retuning constants accordingly
    kp = 1.0;
    ki = 0.00002;
    kd = 0.9;
  } 

   int loop_count = 0;
   double error = 0;
   double previouserror = 0;
   double derivative = 0;
   double pwr = 0;
   double area = 0;
   double dT = 15;
   double integral = 0;
   double setpoint = theta;
   int count = 0;
  
  /* Do the following if using inertial
  
  imu.resetRotation();
  imu.resetHeading();
  replace robotPos.heading with imu.heading(rotationUnits::deg) * scalingFactor if using inertial
  can also create a field-centric system and not reset 
  
  */
  
  //assuming GPS/odom:
  
  autonomous_func::position_info robotPos = odom::getRobotPosition(); 

  while(fabs(robotPos.heading - setpoint) > inaccuracyRange) { 
    loop_count++;


    error = setpoint - robotPos.heading;
  

    pwr = error*kp;
    turn_speed = pwr;
    area = error * dT;
    integral = integral + area;
    if (fabs(error) <= 0.5){
      integral = 0;
    }
    
    if (fabs(error) >= 100){
      integral = 0;
    }
    derivative = error - previouserror;
    previouserror = error;
    pwr = error*kp + integral*ki + derivative*kd;
    turn_speed = (pwr*12)/100; 
    if(pwr<0) {
      turn_speed = fabs(turn_speed);
      leftDrive.spin(directionType::fwd, turn_speed, voltageUnits::volt);
      rightDrive.spin(directionType::reverse, turn_speed, voltageUnits::volt);
      
    }
    else {
      turn_speed = fabs(turn_speed);
      rightDrive.spin(directionType::fwd, turn_speed, voltageUnits::volt);
      leftDrive.spin(directionType::reverse, turn_speed, voltageUnits::volt);

    }

    if(fabs(derivative) <= 0.01){
      count+=1;
    }
    else if(fabs(derivative) > 0.01){
      count = 0;
    }

    if(loop_count>10 && count>=10){
      break;
    }
    vex::task::sleep(dT);
    robotPos = odom::getRobotPosition(); //updates robotPos with current position; only necessary if using odom
  }


    leftDrive.stop();
    rightDrive.stop();
  


  }

}
