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
// int target_lift=36;
int target_lift=100;


//state variable
int overall_state = 2;

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

			if (IR_Values == "00000000" && dSensors[0]->getValue() < 4000 && dSensors[1]->getValue() < 4000){
				std::cout << dSensors[0]->getValue() << " & " << dSensors[1]->getValue() << " : " << "Line following sequence is finished" << std::endl;
				overall_state = 1;
				motion::lineFollowerForward(robot, motors, pSensors, dSensors, 185);
				break;
			}

			break;
		
		case 1:
			//Maze navigation
			motion::unit(robot, motors, pSensors, dSensors, gyro);
			break;
		
		case 2:
			hand::SetGripperPosition(robot, linear, target_hand_position, &current_hand_position);
				
			//Gripper Lift up Control
			// if (counter<target_lift)
			// {
			// 	hand::Gripper_Lifter(servo,1,counter,target_lift);
			// 	counter++;
			// }

			// //Solenoid Control
			// if(state==1)
			// {
			// 	hand::Solenoid(linear,state);
			// 	state=0;  
			// }		
		}
	} 
	// cleanup
	delete robot;
	return 0;	
}