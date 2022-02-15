// XtremeV_controller
#include "MainHeader.h"
#include "motion.cpp"
#include "lineFollow.cpp"
#include "hand.cpp"

// names
char motorNames[2][15] = { "left_motor", "right_motor" };
char dsNames[3][10] = { "left_ds", "right_ds","front_ds"};
char irNames[8][4] = { "ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7"};
char linearNames[3][15] = { "left_linear", "right_linear","ROD_Linear"  };
char servoName[15] = "Gripper_folder";
char psNames[2][15] = { "left_encoder", "right_encoder" };

// objects
Motor *motors[2];
Motor *linear[3];
Motor *servo;
DistanceSensor* dSensors[3];
DistanceSensor* irPanel[8];
PositionSensor* pSensors[2];
Gyro* gyro;

//PID variables
double prevError = 0;
double integral = 0;

//Gripper Lifter
int target_lift=36;

//state variable
int overall_state = 1;

//function init
void forward(Robot* robot);
void stop(Motor** motors);
bool is_wall(DistanceSensor** dSensors, char x);
void turning(Robot* robot, Gyro* gyro);
void unit(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, Gyro* gyro);

//Main
int main(int argc, char **argv) 
{

	Robot *robot = new Robot();
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

	// init position sensors
	for (int i=0; i<2; i++){
		pSensors[i] = robot->getPositionSensor(psNames[i]);
		pSensors[i]->enable(TIME_STEP);
	}

	// init gyroscope
	gyro = robot->getGyro("gyro");
	gyro->enable(TIME_STEP);

	//variables
	string IR_Values;
	int counter = 0;
	int state=1;
	int current_hand_position = 0; //0--50
	int target_hand_position = 20;

	while (robot->step(TIME_STEP) != -1)
	{   
		switch (overall_state)
		{
		case 0:
			//Line Following Block
			IR_Values = lineFollow::getIRValues(irPanel);
			cout << IR_Values << endl;
			lineFollow::PID(motors, IR_Values, &integral, &prevError);
			break;
		
		case 1:
			//Maze navigation
			motion::unit(robot, motors, pSensors, dSensors, gyro);
			break;
		
		case 2:
			//Horizontal Move Control of the hand
			if (current_hand_position<target_hand_position)
			{
				hand::Gripper(linear,1, current_hand_position);
				current_hand_position+=1;
			}
			else if (current_hand_position>target_hand_position)
			{
				hand::Gripper(linear,-1, current_hand_position);
				current_hand_position-=1;    
			}
				
			//Gripper Lift up Control
			if (counter<target_lift)
			{
				hand::Gripper_Lifter(servo,1,counter,target_lift);
				counter++;
			}

			//Solenoid Control
			if(state==1)
			{
				hand::Solenoid(linear,state);
				state=0;  
			}		
		}
	} 
	// cleanup
	delete robot;
	return 0;	
}

void forward(Robot* robot, Motor** motors, PositionSensor** pSensors, int distance)
{
  double lambda = distance/35;
  double ang= pSensors[0] -> getValue();
  double ang1=ang;
  cout<<ang1<<endl;
  while (robot->step(TIME_STEP) != -1)
  {
    cout<<ang1<<endl;
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

void stop(Motor** motors)
{
  for(int k=0;k<2;k++)
  {
    motors[k]->setVelocity(0);
  }
}

bool is_wall(DistanceSensor** dSensors, char x)
{
  if (x=='l')
  {
    if (dSensors[0]->getValue()<4000)
    {
      return true;
    }
    return false;
  }
  else if (x=='f')
  {
    if (dSensors[2]->getValue()<4000)
    {
      return true;
    }
    return false;
  }
  else
  {
    if (dSensors[1]->getValue()<4000)
    {
      return true;
    }
    return false;
  }
}

void turning(Robot* robot, Motor** motors, Gyro* gyro, double angle){
	motion::init_turn(motors, angle);
	while(robot->step(TIME_STEP) != -1){
		if (motion::turn_flag){
			motion::turn(motors, gyro);
		}
		else{
			break;
		}
	}
}

void unit(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, Gyro* gyro)
{
  forward(robot, motors, pSensors, 370);
  if (!(is_wall(dSensors, 'l')))
  {
    turning(robot, motors, gyro, 90 * 3.14/180);
    return;
  }
  else if (!(is_wall(dSensors, 'f')))
  {
    return;
  }
  else if (!(is_wall(dSensors, 'r')))
  {
    turning(robot, motors, gyro, -90 * 3.14/180);
    return;
  } 
  else
  {
    turning(robot, motors, gyro, 180 * 3.14/180);
    return;
  } 
}