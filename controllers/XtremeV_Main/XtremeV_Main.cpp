// XtremeV_controller
#include "MainHeader.h"

// names
char motorNames[2][15] = { "left_motor", "right_motor" };
char dsNames[3][10] = { "left_ds", "right_ds","front_ds"};
char irNames[8][4] = { "ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7"};
char linearNames[3][15] = { "left_linear", "right_linear","ROD_Linear"  };
char servoName[15] = "Gripper_folder";

// objects
Motor *motors[2];
Motor *linear[3];
Motor *servo;
DistanceSensor* dSensors[3];
DistanceSensor* irPanel[8];


//PID variables
double prevError = 0;
double integral = 0;

//Gripper Lifter
int target_lift=36;

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

	//variables
	string IR_Values;
	int counter = 0;
	int state=1;
	int current_hand_position = 0; //0--50
	int target_hand_position = 20;

	while (robot->step(TIME_STEP) != -1)
	{   
		//Line Following Block
		IR_Values = lineFollow::getIRValues(irPanel);
		cout << IR_Values << endl;
		lineFollow::PID(motors, IR_Values, &integral, &prevError);

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
	// cleanup
	delete robot;
	return 0;	
}
