// XtremeV_controller
#include "MainHeader.h"
#include "motion.cpp"
#include "lineFollow.cpp"
#include "hand.cpp"
#include "objDetect.cpp"
#include "imageProcessing.cpp"

// names
char motorNames[2][15] = { "left_motor", "right_motor" };
char dsNames[5][15] = { "left_front", "right_front","front_ds","left_back","right_back"};
char irNames[8][4] = { "ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7"};
char linearNames[2][15] = { "left_linear", "right_linear"};
char servoName[15] = "Gripper_folder";
char psNames[2][15] = { "left_encoder", "right_encoder" };

// objects
Motor *motors[2];
Motor *linear[2];
Motor *servo;
DistanceSensor* dSensors[5];
DistanceSensor* irPanel[8];
PositionSensor* pSensors[2];
Gyro* gyro;
Camera* camera;
Display* display;


//PID variables
double prevError = 0;
double integral = 0;

//PID variables for object following
double obj_prevError = 0;

//Gripper Lifter
int target_lift=100;

//state variable
int overall_state = 0;

//function init
void angleCorrection (Robot *robot, Motor **motors, DistanceSensor **dSensors);
void wall_pid(Robot* robot, Motor** motors, DistanceSensor** dSensors, PositionSensor** pSensors, int Dir,double target);

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
	for (int i = 0; i < 2; i++)
	{
		linear[i] = robot->getMotor(linearNames[i]);
		linear[i]->setPosition(0.0);
	}

	servo =  robot->getMotor(servoName);
	servo->setPosition(0.0);

	// init Distance Sensors
	for (int i = 0; i < 5; i++)
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

	//initializing the camera
	camera = robot->getCamera("camera");
	camera->enable(TIME_STEP);

	//initializing the display
	display = robot->getDisplay("display");

	//variables for camera
	int imageWidth = camera->getWidth();
	int imageHeight = camera->getHeight();
	cv::Mat cv_image;
	cv::Mat cv_image_bgr;
	cv::Mat cv_image_hsv;

	// variables for obj coordinates
	double obj_x = -2;
	double obj_y = -2;
	bool arrived = false;

	//detecting colors
	bool arrived_at_wall = false;

	bool break_main_loop = false;
	double front_dis = 1000;
	double rotating_angle = 0;
	double obj_dis_init = 0;
	double obj_dis = 0;
	int object_type1,object_type2;
	int i = 0;
	double KP_wall = 0;
	double const_dis = 0;
	
	//variables
	string IR_Values;
	int current_hand_position = 0; //0--50
	// int target_hand_position = 20;

	bool start_case_3 = true;
	int time;
	int current_time;
	double offset;
	int IR_THRESHOLD = 250; 
	bool first_key_done = false;
	bool sec_key_done = false;
	bool box_done = false;
	bool cylinder_done = false;
	int d_count = 0;
	double hold_val;
	double right_dis = 0;
	bool change_method = false;

	double total_x_gyro_angle = 0;
	bool incline = false;
	double threshold_angle = 18;

	double wall_gyro_total = 0;

	//selecting the RED model or the BLUE mode
	std::ifstream input_file;
	input_file.open("input.txt");

	std::string colour_mode_string;
	std::getline(input_file, colour_mode_string);
	int colour_mode = std::stoi(colour_mode_string); // 0: red and 1: blue

	std::cout << "Colour mode: " << colour_mode << std::endl;

	while (robot->step(TIME_STEP) != -1)
	{
		if (break_main_loop)
		{
			motors[0]->setVelocity(0.0);
			motors[1]->setVelocity(0.0);
			break;
		}
		// XV_print(overall_state)

		switch (overall_state)
		{
		case 0:
			//Line Following Block
			// cout << "Case:0 Line Following" << endl;

			IR_Values = lineFollow::getIRValues(irPanel, IR_THRESHOLD);
			// cout << IR_Values << endl;
			lineFollow::PID(motors, IR_Values, &integral, &prevError);

			if (IR_Values == "00000000"){
				// std::cout << dSensors[0]->getValue() << " & " << dSensors[1]->getValue() << " : " << "Line following sequence is finished" << std::endl;
				overall_state = 1;
				motion::forward(robot, motors, pSensors, dSensors, 150);
				break;
			}

			break;
		
		case 1:
			//Maze navigation
			// cout << "Case:1 Maze Navigation" << endl;

			//Maze navigation
			motion::unit(robot, motors, pSensors, dSensors, gyro, &wall_gyro_total);
			IR_Values = lineFollow::getIRValues(irPanel, IR_THRESHOLD);

			if (IR_Values == "11111111"){
				motors[0]->setVelocity(0.0);
				motors[1]->setVelocity(0.0);

				motion::forward_pid(robot, motors, pSensors, 250);

				// std::cout << "Maze solving sequence finished" << std::endl;
				overall_state = 2;
			}
			break;
		
		case 2:
			// Object Detection and Key Inserting
			// cout << "Case:2 Object Detection and Key Inserting" << endl;

			obj_dis_init = pSensors[0]->getValue();
			arrived = false;
			obj_prevError = 0;
			rotating_angle = 0;
			const_dis = FWD_DISTANCE;

			while (robot->step(TIME_STEP) != -1)  // detecting and grabing the first object
			{
				// cout << "Detecting the object!!" << endl;
				const unsigned char *image = camera->getImage();
				cv::Mat img = imageProcessing::imageToMatrix(image, imageWidth, imageHeight);

				cv::Point2f object_coordinate = imageProcessing::objectTracking(img, imageWidth, imageHeight, OBJECT, display, true);
				obj_x = object_coordinate.x;
				obj_y = object_coordinate.y;
				rotating_angle += gyro->getValues()[1] * TIME_STEP * 0.001;
				// XV_print(rotating_angle)
				obj_dis = pSensors[0]->getValue() - obj_dis_init;

				///////// avoid colliding with the right wall///////////////
				right_dis  = dSensors[1] -> getValue();
				if(right_dis < 300 && !change_method)
				{	
					
					// cout << "change method!!!!" << endl;
					motion::turning(robot, motors, gyro, -rotating_angle);
					motion::forward_pid_slow(robot, motors, pSensors, -obj_dis*35*cos(rotating_angle) + 100); //come to previous position
					motion::turning(robot, motors, gyro, 1.57);
					motion::forward_pid_slow(robot, motors, pSensors, 550);
					motion::turning(robot, motors, gyro, -1.57);
					motion::forward_pid_slow(robot, motors, pSensors, 250); 
					motion::turning(robot, motors, gyro, -1.57/2);
					change_method = true;
					rotating_angle = -1.57/2;  //resetting the rotation angle
				}
				// XV_print(right_dis)

				/////////////////////////////////////////////////////////////
				objDetect::followObject(motors, obj_x, obj_y, &obj_prevError, &arrived);
				if (arrived)
				{
					hand::SetGripperPosition(robot, linear, 50, &current_hand_position); //open the hand
					motion::forward_pid_slow(robot, motors, pSensors, 170);

					hand::SetGripperPosition(robot, linear, 19, &current_hand_position); //close the hand  for ball -10 for box - 20
					// grabed  = true;

					// correct the correction of the oriantation by shaking
					motion::forward_pid(robot, motors, pSensors, -200);
					motors[0] ->setVelocity(NORMAL_SPEED*0.8);
					motors[1] ->setVelocity(NORMAL_SPEED*0.8);
					hand::SetGripperPosition(robot, linear, 40, &current_hand_position);
					hand::SetGripperPosition(robot, linear, 23, &current_hand_position);
					motors[0] ->setVelocity(0);
					motors[1] ->setVelocity(0);
					
					break;
				}
				
			}
			// XV_print(obj_dis_init);
			robot -> step(1500); //important hold for avoid grip fails

			if(!change_method) // for normal method small angles
			{
				if (rotating_angle > 0.3)
				{
					motion::turning(robot, motors, gyro, 3.142 - rotating_angle);
				}
				else
				{
					motion::turning(robot, motors, gyro, 1.57 - rotating_angle);
					motion::forward_pid(robot, motors, pSensors, 250);
					motion::turning(robot, motors, gyro, 1.57);
					const_dis -= 250;
				}

			}
			else // new method
			{
				motion::turning(robot, motors, gyro, 1.57 - rotating_angle);
				motion::forward_pid(robot, motors, pSensors, 250);
				motion::turning(robot, motors, gyro, 1.57);
				
			}
			
			while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
			{	
				// cout << "loop" << endl;
				front_dis = dSensors[2] -> getValue();
				motors[0] ->setVelocity(NORMAL_SPEED*0.8);
				motors[1] ->setVelocity(NORMAL_SPEED*0.8);
				// XV_print(front_dis)

				if (front_dis < 2300)
				{
					motors[0] ->setVelocity(0);
					motors[1] ->setVelocity(0);
					break;					
				}
			}

			motion::turning(robot, motors, gyro, -1.57);
			if (!change_method)
			{
				motion::general_forward(robot, motors, pSensors, const_dis - obj_dis*35*sin(rotating_angle));
			}
			else // new methode forward distance
			{
				motion::general_forward(robot, motors, pSensors, const_dis - 350);
			}

			robot -> step(1000); //important hold for avoid grip fails
			motion::turning(robot, motors, gyro, 1.57);
			robot -> step(1000);
			hand::SetGripperPosition(robot, linear, 26, &current_hand_position); //open the hand
			robot -> step(1000);
			motion::forward_pid(robot, motors, pSensors, -150);

			/// detecting the object///
			i = 0;
			while (robot->step(TIME_STEP) != -1)  // detecting  object
			{
				const unsigned char *image = camera->getImage();
				cv::Mat img = imageProcessing::imageToMatrix(image, imageWidth, imageHeight);
				object_type1 = imageProcessing::detectShape(img, imageWidth, imageHeight, 9, display, true);
				if ( i > 10)
				{
					// std::cout << "Shape Detected: " << object_type1 << std::endl;
					break;	
				}
				i++;
			}

			///////////////////////////
			if(object_type1 == 0) //cylinder
			{
				cout << "cylinder detected" << endl;
				motion::general_forward(robot, motors, pSensors, 210);
				hand::SetGripperPosition(robot, linear, 21, &current_hand_position); //close the grip
				motion::general_forward(robot, motors, pSensors, -100);
				// correcting the distance to the wall
				while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
				{	
					// cout << "loop" << endl;
					front_dis = dSensors[2] -> getValue();
					motors[0] ->setVelocity(NORMAL_SPEED*0.8);
					motors[1] ->setVelocity(NORMAL_SPEED*0.8);
					// XV_print(front_dis)

					if (front_dis < 2300)
					{
						motors[0] ->setVelocity(0);
						motors[1] ->setVelocity(0);
						break;					
					}
				}
				motion::turning(robot, motors, gyro, 3.142);
				motion::general_forward(robot, motors, pSensors, 75);
				motion::turning(robot, motors, gyro, 1.57);

				//go to the key hole
				arrived_at_wall = false;
				while (robot->step(TIME_STEP) != -1 && !arrived_at_wall) // go to the key hole wall 
				{	
					// cout << "loop" << endl;
					front_dis = dSensors[2] -> getValue();
					motors[0] ->setVelocity(NORMAL_SPEED*0.8); //slow down!!!!
					motors[1] ->setVelocity(NORMAL_SPEED*0.8);
					// XV_print(front_dis)
					wall_pid(robot,motors,dSensors,pSensors,'l',3550);

					if (front_dis < 900)
					{
						motors[0] ->setVelocity(0);
						motors[1] ->setVelocity(0);
						// first_wall_passed = true;
						arrived_at_wall = true;
						break;					
					}
				}
				//inserting the square key
				// cout << "inserting the key" <<endl;
				hand::SetGripperPosition(robot, linear, 30, &current_hand_position); //open the hand
				robot -> step(4000);
				motion::general_forward(robot, motors, pSensors, -200); //go backward
				hand::SetGripperPosition(robot, linear, 12, &current_hand_position); //close the hand
				motion::general_forward(robot, motors, pSensors, 215);
				robot -> step(1000); //to slow down the robot
				motion::general_forward(robot, motors, pSensors, -500); //go backward
				cylinder_done =true;

			}
			
			else if (object_type1 == 1) //box
			{
				motion::turning(robot, motors, gyro, -1.57);
				motion::forward_pid(robot, motors, pSensors, 320);
				motion::turning(robot, motors, gyro, 1.57);
				motion::forward_pid(robot, motors, pSensors, 250);
				motion::turning(robot, motors, gyro, 1.57);
				/// detecting the object///

				i = 0;
				while (robot->step(TIME_STEP) != -1)  // detecting  object
				{
					const unsigned char *image = camera->getImage();
					cv::Mat img = imageProcessing::imageToMatrix(image, imageWidth, imageHeight);
					object_type2 = imageProcessing::detectShape(img, imageWidth, imageHeight, 9, display, true);
					if ( i > 10)
					{
						// std::cout << "Shape Detected: " << object_type2 << std::endl;
						break;	
					}
					i++;
				}

				///////////////////////////////////////

				if (object_type2 == 0) //cylinder
				{
					cout << "cylinder detected" << endl;
					hand::SetGripperPosition(robot, linear, 50, &current_hand_position); //open the hand

					obj_dis_init = pSensors[0]->getValue();
					arrived = false;
					obj_prevError = 0;
					rotating_angle = 0;
					const_dis = FWD_DISTANCE;

					while (robot->step(TIME_STEP) != -1)  // detecting and grabing the cylinder
					{
						// cout << "Detecting the object!!------------------------p" << endl;
						const unsigned char *image = camera->getImage();
						cv::Mat img = imageProcessing::imageToMatrix(image, imageWidth, imageHeight);

						cv::Point2f object_coordinate = imageProcessing::objectTrackingLeft(img, imageWidth, imageHeight, OBJECT, display, true);
						obj_x = object_coordinate.x;
						obj_y = object_coordinate.y;

						rotating_angle += gyro->getValues()[1] * TIME_STEP * 0.001;
						// XV_print(rotating_angle)
						obj_dis = pSensors[0]->getValue() - obj_dis_init;
						objDetect::followObject(motors, obj_x, obj_y, &obj_prevError, &arrived);
						if (arrived)
						{
							hand::SetGripperPosition(robot, linear, 50, &current_hand_position); //open the hand
							motion::forward_pid(robot, motors, pSensors, 165);

							hand::SetGripperPosition(robot, linear, 19, &current_hand_position); //close the hand  for ball -10 for box - 20

							// correct the correction of the oriantation by shaking
							motion::forward_pid(robot, motors, pSensors, -200);
							motors[0] ->setVelocity(NORMAL_SPEED);
							motors[1] ->setVelocity(NORMAL_SPEED);
							hand::SetGripperPosition(robot, linear, 40, &current_hand_position);
							hand::SetGripperPosition(robot, linear, 23, &current_hand_position);
							motors[0] ->setVelocity(0);
							motors[1] ->setVelocity(0);
							
							break;
						}
						
					}

					////////////////////
					robot -> step (1000); // important delay to avoid gripper fails
					motion::turning(robot, motors, gyro, 1.57- rotating_angle); //correcting and rotate 90 after grabing cylinder
					motion::general_forward(robot, motors, pSensors, 350);
					motion::turning(robot, motors, gyro, 3.142);

					while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
					{	
						// cout << "loop" << endl;
						front_dis = dSensors[2] -> getValue();
						KP_wall = front_dis/4000;
						motors[0] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						motors[1] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						// XV_print(front_dis)

						if (front_dis < 2300)
						{
							motors[0] ->setVelocity(0);
							motors[1] ->setVelocity(0);
							break;					
						}
					}

					motion::forward_pid(robot, motors, pSensors, -100);
					motion::turning(robot, motors, gyro, -1.57);

					//go to the key hole
					while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
					{	
						// cout << "loop" << endl;
						front_dis = dSensors[2] -> getValue();
						KP_wall = front_dis/4000;
						motors[0] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						motors[1] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						// XV_print(front_dis)
						wall_pid(robot,motors,dSensors,pSensors,'l',3550);
						if (front_dis < 900)
						{
							motors[0] ->setVelocity(0);
							motors[1] ->setVelocity(0);
							break;					
						}
					}
					//inserting the square key
					// cout << "inserting the key" <<endl;
					hand::SetGripperPosition(robot, linear, 30, &current_hand_position); //open the hand
					robot -> step(4000);
					motion::general_forward(robot, motors, pSensors, -200); //go backward
					hand::SetGripperPosition(robot, linear, 12, &current_hand_position); //close the hand
					// hand::liftHand(servo,robot, -1,20); //low the hand
					motion::general_forward(robot, motors, pSensors, 215);
					robot -> step(1000); //to slow down the robot
					motion::general_forward(robot, motors, pSensors, -500); //go backward
					cylinder_done = true;
					

				}
				else if (object_type2 == 1) //box
				{
					cout << "box detected" << endl;

					hand::SetGripperPosition(robot, linear, 50, &current_hand_position); //open the hand
					motion::forward_pid(robot, motors, pSensors, 200);
					hand::SetGripperPosition(robot, linear, 21, &current_hand_position); //close the grip
					robot -> step(1000); //important hold for avoid grip fails
					motion::turning(robot, motors, gyro, 1.57);
					motion::general_forward(robot, motors, pSensors, 350);
					motion::turning(robot, motors, gyro, 3.142);

					while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
					{	
						// cout << "loop" << endl;
						front_dis = dSensors[2] -> getValue();
						KP_wall = front_dis/4000;
						motors[0] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						motors[1] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						// XV_print(front_dis)

						if (front_dis < 1500)
						{
							motors[0] ->setVelocity(0);
							motors[1] ->setVelocity(0);
							break;					
						}
					}

					motion::turning(robot, motors, gyro, -1.55);
					
					//go to the key hole
					while (robot->step(TIME_STEP) != -1) 
					{	
						// cout << "loop" << endl;
						front_dis = dSensors[2] -> getValue();
						KP_wall = front_dis/4000;
						motors[0] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						motors[1] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
						// XV_print(front_dis)
						wall_pid(robot,motors,dSensors,pSensors,'l',1540);
						if (front_dis < 1000)
						{
							motors[0] ->setVelocity(0);
							motors[1] ->setVelocity(0);
							break;					
						}
					}
					//inserting the square key
					// cout << "inserting the key" <<endl;
					hand::SetGripperPosition(robot, linear, 30, &current_hand_position); //open the hand
					robot -> step(4000);
					motion::general_forward(robot, motors, pSensors, -200); //go backward
					hand::SetGripperPosition(robot, linear, 12, &current_hand_position); //close the hand
					motion::forward_pid(robot, motors, pSensors, 230);
					robot -> step(1000); //to slow down the robot
					motion::general_forward(robot, motors, pSensors, -500); //go backward
					box_done = true;
				}
				else  // cannot detect
				{
					cout << "object cannot detect" << endl;

				}
			}
			else // cannot detect
			{
				cout << "object cannot detect" << endl;

			}

			/////////////terminate logic ///////////////////////////////
			
			if ( (box_done || cylinder_done) && !first_key_done)
			{

				first_key_done = true;
				// cout << "going for the secound object" << endl;

				//go to begining
				motion::turning(robot, motors, gyro, 1.57);
				while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
				{	
					// cout << "loop" << endl;
					front_dis = dSensors[2] -> getValue();
					KP_wall = front_dis/4000;
					motors[0] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);
					motors[1] ->setVelocity(NORMAL_SPEED*0.8*KP_wall);

					if (front_dis < 1500) //min gap
					{
						motors[0] ->setVelocity(0);
						motors[1] ->setVelocity(0);
						break;					
					}
				}
				motion::turning(robot, motors, gyro, 1.57);
				while (robot->step(TIME_STEP) != -1) // go to the begining - wall following
				{	
					// cout << "loop" << endl;
					front_dis = dSensors[2] -> getValue();
					motors[0] ->setVelocity(NORMAL_SPEED*0.8); //slow down!!!!
					motors[1] ->setVelocity(NORMAL_SPEED*0.8);
					// XV_print(front_dis)
					wall_pid(robot,motors,dSensors,pSensors,'r',1500);

					if (front_dis < 4000)
					{
						motors[0] ->setVelocity(0);
						motors[1] ->setVelocity(0);
						// first_wall_passed = true;
						arrived_at_wall = true;
						break;					
					}
				}
				motion::forward_pid(robot, motors, pSensors, 200);
				motion::turning(robot, motors, gyro, 1.57);

			}
			else if (first_key_done && !sec_key_done) //all keys complete
			{
				// cout << "sec_key_done" << endl;
				sec_key_done = true;
				//come back and grab the ball
				motion::turning(robot, motors, gyro, -1.57);

				obj_dis_init = pSensors[0]->getValue();
				arrived = false;
				obj_prevError = 0;
				rotating_angle = 0;
				const_dis = FWD_DISTANCE;
				while (robot->step(TIME_STEP) != -1)  // detecting and grabing the colour ball
				{
					// cout << "Detecting the object!!" << endl;
					const unsigned char *image = camera->getImage();
					cv::Mat img = imageProcessing::imageToMatrix(image, imageWidth, imageHeight);

					cv::Point2f object_coordinate = imageProcessing::objectTracking(img, imageWidth, imageHeight, 3 + colour_mode, display, true);
					obj_x = object_coordinate.x;
					obj_y = object_coordinate.y;

					rotating_angle += gyro->getValues()[1] * TIME_STEP * 0.001;
					// XV_print(rotating_angle)
					obj_dis = pSensors[0]->getValue() - obj_dis_init;
					objDetect::followObject(motors, obj_x, obj_y, &obj_prevError, &arrived);
					if (arrived)
					{
						hand::SetGripperPosition(robot, linear, 50, &current_hand_position); //open the hand
						motion::forward_pid(robot, motors, pSensors,135);
						hand::SetGripperPosition(robot, linear, 19, &current_hand_position); //close the hand  for ball -10 for box - 20
						// grabed  = true;						
						break;
					}
					
				}
				// XV_print(obj_dis_init);
				robot -> step(1500); //important hold for avoid grip fails

					motion::turning(robot, motors, gyro, 3.142 - rotating_angle);

				while (robot->step(TIME_STEP) != -1) // go to the wall near yello square
				{	
					// cout << "loop" << endl;
					front_dis = dSensors[2] -> getValue();
					motors[0] ->setVelocity(NORMAL_SPEED*0.8);
					motors[1] ->setVelocity(NORMAL_SPEED*0.8);
					// XV_print(front_dis)

					if (front_dis < 2300)
					{
						motors[0] ->setVelocity(0);
						motors[1] ->setVelocity(0);
						break;					
					}
				}

				motion::turning(robot, motors, gyro, -3.142);
				motion::forward_pid(robot, motors, pSensors, 500);
				robot -> step(1000); //important hold for avoid grip fails
				motion::turning(robot, motors, gyro, 1.57);
				robot -> step(1000);
				while (robot->step(TIME_STEP) != -1)
				{
					IR_Values = lineFollow::getIRValues(irPanel, IR_THRESHOLD);
					// cout << IR_Values << endl;
					motors[0] ->setVelocity(NORMAL_SPEED*0.8);
					motors[1] ->setVelocity(NORMAL_SPEED*0.8);
					if (IR_Values == "11111111")
					{
						motors[0] ->setVelocity(0);
						motors[1] ->setVelocity(0);
						break;					
					}
					
				}
				// hand::SetGripperPosition(robot, linear, 30, &current_hand_position); //open the hand
				robot -> step(1000);
				overall_state++;  //case

			}
			else
			{
				cout << "wrong logic in object detection ending" << endl;
			}
	
			// cout << "end of the key inserting!!!" << endl;
			break;

		case 3:
			//Line Following Block
			// std::cout << "Case 3: Dotted Line Following" << std::endl;

			if (start_case_3){
				time = std::time(0);
		
				hand::liftHand(servo,robot, 1,20);
				start_case_3 = false;
			}

			IR_Values = lineFollow::getIRValues(irPanel, IR_THRESHOLD);
			
			// count1s = std::count(IR_Values.begin(), IR_Values.end(), '1');

			current_time = std::time(0);
			// XV_print(IR_THRESHOLD)

			hold_val = gyro->getValues()[0];

			total_x_gyro_angle += hold_val * TIME_STEP * 0.001 * 180 / 3.14;
			// std::cout << "Rotation: " << total_x_gyro_angle << std::endl;

			if (incline == false && std::abs(total_x_gyro_angle) >= threshold_angle ){
				d_count++;
				incline = true;
			}
			else if (incline == true && std::abs(total_x_gyro_angle) <= 1){
				d_count++;
				incline = false;
			}

			// std::cout << "D Count: " << d_count << std::endl;

			if (d_count == 4 && colour_mode == 1){
				IR_THRESHOLD = 580;
			}

			if (IR_Values == "11111111" && current_time-time >= 10 ){
				std::cout << "Arrived at the shooting line" << std::endl;
				motors[0]->setVelocity(0.0);
				motors[1]->setVelocity(0.0);
				overall_state = 4;

				break;
			}

			lineFollow::PID(motors, IR_Values, &integral, &prevError);

			break;

		case 4:
			// Shooting
			// cout << "Case:4 Shooting" << endl;

			motion::general_forward(robot, motors, pSensors, -40);
			offset = 30;

			if (colour_mode == 0){
				motion::turning(robot, motors, gyro, -0.21867 - 0.08);
			}
			else{
				motion::turning(robot, motors, gyro, 0.21867 + 0.05);
			}
			hand::liftHand(servo,robot, 0,21);			
			hand::SetGripperPosition(robot, linear, 21, &current_hand_position);
			motion::general_forward_fast(robot, motors, pSensors, -1*offset);
			motion::general_forward_fast(robot, motors, pSensors, 2*offset);			
			overall_state = 5;
			break;

		case 5:
			break;
		}
	} 
	// cleanup
	delete robot;
	return 0;	
}

void wall_pid(Robot* robot, Motor** motors, DistanceSensor** dSensors, PositionSensor** pSensors, int Dir, double target)
{
	// cout << "wall following" << endl;
	int i=0;
	int mark=-1;  
	double distances[5];
	for(int j=0;j<5;j++)
	{
		distances[j]=dSensors[j]->getValue();
	}
	if (Dir=='l')
	{
		i=1;
		mark=1;
	}
	double align_err = (target - distances[1-i]);
	// XV_print(align_err)
	double velocity_err=WALL_KP*align_err/100;
	// XV_print(velocity_err)
	double Left_V=NORMAL_SPEED+mark*velocity_err;
	double Right_V=NORMAL_SPEED-mark*velocity_err;

	if (Left_V > NORMAL_SPEED)
		Left_V = NORMAL_SPEED;
	if (Right_V > NORMAL_SPEED)
		Right_V = NORMAL_SPEED;
	if (Left_V < -NORMAL_SPEED)
		Left_V = -NORMAL_SPEED;
	if (Right_V < -NORMAL_SPEED)
		Right_V = -NORMAL_SPEED;
		
	motors[0]->setVelocity(Left_V);
	motors[1]->setVelocity(Right_V);
}
