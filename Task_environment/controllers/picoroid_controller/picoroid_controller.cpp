// File:          picoroid_controller.cpp
// Date:
// Description:
// Author: pico-roid
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
//#include <cmath>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;
using namespace std;

void setMotorSpeed(double left_motor_speed,double right_motor_speed);
double PIDCalc(double setPoint,double sensorReading);
double PIDCalcLine(double setPoint,double sensorReading);
double PIDCalcStripLine(double setPoint,double sensorReading);
void readIRSensors();
bool floorChangeDetected();


Robot *robot = new Robot();
Camera *camera = robot->getCamera("camera");
Motor *leftMotor = robot->getMotor("leftmotor");
Motor *rightMotor = robot->getMotor("rightmotor");
DistanceSensor *frontsensor = robot->getDistanceSensor("frontsensor");
DistanceSensor *leftsensor = robot->getDistanceSensor("leftsensor");
DistanceSensor *rightsensor = robot->getDistanceSensor("rightsensor");
DistanceSensor *backsensor = robot->getDistanceSensor("backsensor");
DistanceSensor *irSensorArray[8];

int irSensorData[8];
//double error = 0;
double backsensorReading = 0;
double frontsensorReading = 0;
double leftsensorReading = 0;
double rightsensorReading = 0;
double kp = 0.18;
double ki = 0.0009;
double kd = 2;
double sumError = 0;
double prevError = 0 ;
double avgSpeed = 2.5;
double IntegralMaxLimit = 0.015;
double PIDMaxLimit = 1.5;
double PIDMaxLimitLine = 2;


bool pass = false; 
bool turningLeft = false;
bool turning180 = false;
int rightsensordetected20 = 0; 


int lineError = 0;
int scase = 0;
double kpLine = 0.006;
double kiLine = 0.04;
double kdLine = 0;

double kpstripLine = 0.008;
double kistripLine = 0.02;
double kdstripLine = 0;
double avgSpeedStripped = 2;


int switchcase = 0;

int main(int argc, char **argv) {


  for (int i=0; i<8;i++){
    string irSensorName = "sensor" + to_string(i);
    irSensorArray[i] = robot->getDistanceSensor(irSensorName);
    irSensorArray[i]->enable(TIME_STEP);
  }
  
  frontsensor->enable(TIME_STEP);
  leftsensor->enable(TIME_STEP);
  rightsensor->enable(TIME_STEP);
  backsensor->enable(TIME_STEP);
   
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);  
  
  cout.precision(4);
 
  double pidResult = 0;
  // Main loop:------------------------------------------------------------

  while (robot->step(TIME_STEP) != -1) {

    switch (switchcase) {
    // -------------- Line Following   -------------------------------------
  case 0:{
     

    readIRSensors();
    cout <<  "LINE ERROR = " << lineError << endl;

    lineError = (irSensorData[0]*-240+irSensorData[1]*-120 + irSensorData[2]*-60 +irSensorData[3]*-40 +
    irSensorData[4]*40 + irSensorData[5]*60 +irSensorData[6]*120 + +irSensorData[7]*240);

    pidResult = PIDCalcLine(0,lineError);
    cout << "PID output : " << pidResult <<endl;
    setMotorSpeed(avgSpeed+pidResult, avgSpeed-pidResult); 
  
    if (floorChangeDetected()) switchcase += 1;
    cout << switchcase << endl;

  }
  break;
    // ---- Wall Following ------------------
    case 1: {
    backsensorReading = backsensor->getValue();
    frontsensorReading = frontsensor->getValue();
    leftsensorReading = leftsensor->getValue();
    rightsensorReading = rightsensor->getValue();

    cout << "Back Distance : " << backsensorReading << "   | Left Distance : "
     << leftsensorReading << "  | Front Distance : " << frontsensorReading <<  "  | Right Distance : " 
     << rightsensorReading << endl;
     
    pidResult = PIDCalc(10,backsensorReading);

    if (frontsensorReading >= 20 && leftsensorReading <= 15 ){
      cout << " -------------- LEFT WALL FOLLOw --------------" << endl;
      setMotorSpeed(avgSpeed-pidResult, avgSpeed+pidResult); 
    }
     
    else if (frontsensorReading <= 20 && leftsensorReading <= 15  && rightsensorReading > 30){
      while(robot->step(TIME_STEP) != -1 && (!pass || leftsensorReading > 15)){
            cout << " -------------- TURN RIGHT --------------" << endl;
            leftsensorReading = leftsensor->getValue();
            cout << "Left Distance : " <<  leftsensorReading << endl;
            cout << !pass << endl;
            setMotorSpeed(avgSpeed + pidResult, 0.25);
            if ( leftsensorReading > 40 ){
              pass = true;
               cout << "---------- " << endl;
             }     
      }
      pass = false;
    }
    else if(frontsensorReading <= 20 && leftsensorReading >= 15){

      turningLeft = true;
  
      while(robot->step(TIME_STEP) != -1 && (!pass || leftsensorReading > 15)){
            cout << " -------------- TURN LEFT --------------" << endl;
            leftsensorReading = leftsensor->getValue();
            cout << "Left Distance : " <<  leftsensorReading << endl;
            cout << !pass << endl;
            setMotorSpeed(0.25, avgSpeed);
            if ( leftsensorReading > 40 ){
              pass = true;
               cout << "---------- " << endl;
             }     
      }
      pass = false;

    }
    else if(frontsensorReading <= 15 && leftsensorReading <= 15 && rightsensorReading <=17){
      while(robot->step(TIME_STEP) != -1 && (!pass || rightsensorReading > 30)){
            cout << " -------------- TURN 180 Degrees --------------" << endl;
            rightsensorReading = rightsensor->getValue();
            cout << "Right Distance : " <<  rightsensorReading << endl;
            cout << !pass << endl;
            setMotorSpeed(1.5 , -1.5);
            if ( rightsensorReading > 40 ){
              pass = true;
              cout << "---------- " << endl;
             }     
      }
    pass = false;
    }
    else {
        cout << " ------------DEFAULT--------------" << endl;
        setMotorSpeed(avgSpeed-pidResult, avgSpeed+pidResult); 
        cout << "Left Motor Speed = " <<  avgSpeed-pidResult << "  | Right Motor Speed = " <<  avgSpeed+pidResult << endl;
    }


    cout << "PID output : " << pidResult <<endl;
  }
  break;

     // -------------- Stripped Line Following   -------------------------------------
  case 2:{
     
     for (int i=0; i<8;i++){
      if (irSensorArray[i]->getValue()>500){
        irSensorData[i] = 0;
      }
      else{
        irSensorData[i] = 1;
      }
    }
    scase = 0;
    for (int i=0; i<8;i++){
      scase+=irSensorData[i];
      cout << irSensorData[i] << "  ";
    }
    cout <<  "ERROR = " << lineError << endl;

    lineError = (irSensorData[0]*-240+irSensorData[1]*-120 + irSensorData[2]*-60 +irSensorData[3]*-40 +
    irSensorData[4]*40 + irSensorData[5]*60 +irSensorData[6]*120 + +irSensorData[7]*240);

    cout << lineError << endl;
    pidResult = PIDCalcStripLine(0,lineError);
    cout << "PID output : " << pidResult <<endl;
    setMotorSpeed(avgSpeed+pidResult, avgSpeed-pidResult); 
     if (lineError>200){
        
        while(robot->step(TIME_STEP) != -1 && (lineError > -20)){
          cout << " TURN RIGHT : " <<endl;
          cout <<  "ERROR = " << lineError << endl;
          for (int i=0; i<8;i++){
            if (irSensorArray[i]->getValue()>500){
              irSensorData[i] = 0;
            }
            else{
              irSensorData[i] = 1;
            }
          }
          lineError = (irSensorData[0]*-240+irSensorData[1]*-120 + irSensorData[2]*-60 +irSensorData[3]*-40 +
                        irSensorData[4]*40 + irSensorData[5]*60 +irSensorData[6]*120 + +irSensorData[7]*240);
          setMotorSpeed(avgSpeed,0.2);
      }
     }
     else if (lineError<-200){
        
        while(robot->step(TIME_STEP) != -1 && (lineError < 20)){
          cout << " TURN LEFT : " <<endl;
          cout <<  "ERROR = " << lineError << endl;
          for (int i=0; i<8;i++){
            if (irSensorArray[i]->getValue()>500){
              irSensorData[i] = 0;
            }
            else{
              irSensorData[i] = 1;
            }
          }
          lineError = (irSensorData[0]*-240+irSensorData[1]*-120 + irSensorData[2]*-60 +irSensorData[3]*-40 +
                        irSensorData[4]*40 + irSensorData[5]*60 +irSensorData[6]*120 + +irSensorData[7]*240);
          setMotorSpeed(0.2,avgSpeed);
      }
     }

  }
  break;
  }
  };

  delete robot;
  return 0;
}



void setMotorSpeed(double left_motor_speed,double right_motor_speed){
    leftMotor->setVelocity(left_motor_speed);
    rightMotor->setVelocity(right_motor_speed);
}

double PIDCalc(double setPoint,double sensorReading){

  double error = sensorReading - setPoint;
  double porpotional = kp*error;
  sumError += error;
  double integral = ki*sumError;
  cout << "porportional = " << porpotional << "  | Sum Error = " << sumError <<"  |  ";
  if (integral <= -IntegralMaxLimit || integral >= IntegralMaxLimit){
      integral = 0.01;
  }
  
  double derivative = kd* (error - prevError);
  prevError = error;
  cout << "derivative = " << derivative << endl; 
  double PIDResult =  porpotional + integral + derivative;

  if ( -PIDMaxLimit >= PIDResult || PIDResult >= PIDMaxLimit){
    PIDResult = PIDMaxLimit;
  }
  return PIDResult;
}

double PIDCalcLine(double setPoint,double sensorReading){

  double error = sensorReading - setPoint;
  double porpotional = kpLine*error;
  sumError += error;
  double integral = kiLine*sumError;
  
  if (abs(integral) > IntegralMaxLimit){
       if (integral > 0){integral = 0.2;}
       else{integral = -0.2;}
   }
  
  double derivative = kdLine* (error - prevError);
  prevError = error;
  cout << "porportional = " << porpotional << "  | Sum Error = " << sumError <<"  |  "
   << "derivative = " << derivative << endl; 
  double PIDResult =  porpotional + integral + derivative;

  if ( -PIDMaxLimitLine >= PIDResult || PIDResult >= PIDMaxLimit){
    PIDResult = PIDMaxLimitLine;
  }
  return PIDResult;
}

double PIDCalcStripLine(double setPoint,double sensorReading){

  double error = sensorReading - setPoint;
  double porpotional = kpstripLine*error;
  sumError += error;
  double integral = kistripLine*sumError;
  cout << "porportional = " << porpotional << "  | Sum Error = " << sumError <<"  |  ";
  if (abs(integral) > IntegralMaxLimit){
       if (integral > 0){integral = 0.2;}
       else{integral = -0.2;}
   }
  
  double derivative = kdstripLine* (error - prevError);
  prevError = error;
  cout << "derivative = " << derivative << endl; 
  double PIDResult =  porpotional + integral + derivative;

  if ( -PIDMaxLimitLine >= PIDResult || PIDResult >= PIDMaxLimit){
    PIDResult = PIDMaxLimitLine;
  }
  return PIDResult;
}

void readIRSensors(){
   for (int i=0; i<8;i++){
      if (irSensorArray[i]->getValue()>500){
        irSensorData[i] = 0;
      }
      else{
        irSensorData[i] = 1;
      }
    }
    cout <<  "\n\nIR SENSOR READINGS = [ ";
    for (int i=0; i<8;i++){
      cout << irSensorData[i] << " ";
    }
    cout << "]" << endl;
}

bool floorChangeDetected(){
  int x = 0;
  for (int i=0; i<8;i++){
      x +=irSensorData[i];
      cout << irSensorData[i] << "  ";
  }
  if (x==0) return true;
  else return false;
}


