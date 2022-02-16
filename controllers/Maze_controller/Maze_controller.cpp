// XtremeV_controller
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <string>
#include <vector>
#include <webots/PositionSensor.hpp>    
#include "MainHeader.h"
#include "motion.cpp"
//constants
#define TIME_STEP 32
#define NORMAL_SPEED 5
#define MAX_SPEED 7
#define IR_THRESHOLD 250
//macros
#define print(x)     cout << #x": " << x << "\n";

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

//names
char motorNames[2][15] = { "left_motor", "right_motor" };
char dsNames[3][10] = { "left_ds", "right_ds","front_ds"};
char irNames[8][4] = { "ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7"};
char linearNames[3][15] = { "left_linear", "right_linear","ROD_Linear"  };
char servoName[15] = "Gripper_folder";
char psNames[2][15] = { "left_encoder", "right_encoder" };

Motor *motors[2];
Motor *linear[3];
Motor *servo;
DistanceSensor* dSensors[3];
DistanceSensor* irPanel[8];
PositionSensor *pSensors[2];
Robot *robot = new Robot();
Gyro* gyro;
//function init
void forward();
void stop();
bool is_wall(char x);
void unit();

int main(int argc, char **argv) {

  
  // init Motors
  for (int i = 0; i < 2; i++)
  {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }
  
  //Hand Motors
  for (int i = 0; i < 3; i++)
  {
    linear[i] = robot->getMotor(linearNames[i]);
    linear[i]->setPosition(0.0);
  }
  
  servo =  robot->getMotor(servoName);
  servo->setPosition(0.0);
  
  // init Distance Sensors
  for (int i = 0; i < 3; i++)
  {
    dSensors[i] = robot->getDistanceSensor(dsNames[i]);
    dSensors[i]->enable(TIME_STEP);
  }
  
   // init IR Panel
  for (int i = 0; i < 8; i++)
  {
    irPanel[i] = robot->getDistanceSensor(irNames[i]);
    irPanel[i]->enable(TIME_STEP);
  }

  for (int i = 0; i < 2; i++)
  {
    pSensors[i] = robot->getPositionSensor(psNames[i]);
    pSensors[i]->enable(TIME_STEP);
  }
  
  gyro = robot->getGyro("gyro");
  gyro->enable(TIME_STEP);

  //variables
  // double ir_val;  // used for printing IR values
  string IR_Values;


  while (robot->step(TIME_STEP) != -1)
  {
    unit();
  };

  delete robot;
  return 0;
}


void forward(int distance)
{
  double lambda = distance/35;
  double ang=pSensors[0] -> getValue();
  double ang1=ang;
  cout<<ang1<<endl;
  while (robot->step(TIME_STEP) != -1)
  {
    //cout<<ang1<<endl;
    ang1=pSensors[0] -> getValue();
    if (ang1-ang>lambda)////-//////////////////////[12.5 change kranw]
    {
      for(int k=0;k<2;k++)
      {
        motors[k]->setVelocity(0);
      }
      return;
    }
      for(int k=0;k<2;k++)
      {
        motors[k]->setVelocity(MAX_SPEED);
      }
  }
}

void stop()
{
  for(int k=0;k<2;k++)
  {
    motors[k]->setVelocity(0);
  }
}

bool is_wall(char x)
{
  if (x=='l')
  {
    if (dSensors[0]->getValue()<2000)
    {
      return true;
    }
    return false;
  }
  else if (x=='f')
  {
    if (dSensors[2]->getValue()<2000)
    {
      return true;
    }
    return false;
  }
  else
  {
    if (dSensors[1]->getValue()<2000)
    {
      return true;
    }
    return false;
  }
}

void turning(double angle){
  std::cout << dSensors[0]->getValue() << " , " << dSensors[1]->getValue() << " , " << dSensors[2]->getValue() << std::endl;
  std::cout << "Position1: " << pSensors[0]->getValue() << " , " << pSensors[1]->getValue() << std::endl;
  motion::init_turn(motors, angle);
	while(robot->step(TIME_STEP) != -1){
		if (motion::turn_flag){
			motion::turn(motors, gyro);
		}
		else{
			break;
		}
	}

  if (std::abs(angle) == 90 * 3.14/180){
    //forward(50);
    std::cout << "Position: " << pSensors[0]->getValue() << " , " << pSensors[1]->getValue() << std::endl;
  }

  std::cout << "turning complete" << std::endl;
}

void unit()
{
  forward(370);
  if (!(is_wall('l')))
  {
    turning(90 * 3.14/180);
    return;
  }
  else if (!(is_wall('f')))
  {
    return;
  }
  else if (!(is_wall('r')))
  {
    turning(-90 * 3.14/180);
    return;
  } 
  else
  {
    turning(180 * 3.14/180);
    return;
  } 
}