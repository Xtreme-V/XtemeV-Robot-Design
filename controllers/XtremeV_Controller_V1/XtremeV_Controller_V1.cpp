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

//Overall
int Overall_state=0;
//PID
const double KP = 1.7;
const double KI = 0;
const double KD = 0.1;
double prevError = 0;
double error = 0;
double integral = 0;
double correction = 0;

//Gripper Lifter
int target_lift=36;


//objects
Motor *motors[2];
Motor *linear[3];
Motor *servo;
DistanceSensor* dSensors[3];
DistanceSensor* irPanel[8];

//Functions 
string getIRValues();
void PID(string IRVal);
void Solenoid(int state);
void Gripper(int dir, int current_pos);
void Gripper_Lifter(bool iflift,int counter);


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
  // double ir_val;  // used for printing IR values
  string IR_Values;
  int counter = 0;
  int state=1;
  int current_hand_position = 0; //0--50
  int target_hand_position = 20;
  
  while (robot->step(TIME_STEP) != -1)
  {
    // Print IR Panel Values
    // for (int i = 0; i < 8; i++)
    // {
      // ir_val = irPanel[i] -> getValue();
      // cout << ir_val << "  ";
    // }
    // cout << endl;
    switch (Overall_state)
    {
    case 0:
        //Line Following Block
        IR_Values = getIRValues();
        cout << IR_Values << endl;
        PID(IR_Values);
        break;
    case 1:
        //Maze 
        break;
    case 3:
        //Horizontal Move Control of the hand
        if (current_hand_position<target_hand_position)
         {
            Gripper(1, current_hand_position);
            current_hand_position+=1;
         }
         else if (current_hand_position>target_hand_position)
         {
            Gripper(-1, current_hand_position);
            current_hand_position-=1;
            
         }
          
         //Gripper Lift up Control
         if (counter<target_lift)
         {
            Gripper_Lifter(1,counter);
            counter++;
         }
        
         //Solenoid Control
         if(state==1)
         {
            Solenoid(state);
            state=0;
         }
         break;
    }
     
  } 
///// END of the Main Function  /////

  // cleanup
  delete robot;
  return 0;
}

//Functions

string getIRValues()
{
    string panelVals = "";
    for (int i = 0; i < 8; i++)
    {
        if (irPanel[i] -> getValue() > IR_THRESHOLD) // For Balck
            panelVals += "0";
        else  //For White
            panelVals += "1"; 
    }
    return panelVals;
}

void PID(string IRVal)
{
    int panelWeights[8] = {-400,-300,-200,-100,100,200,300,400};
    double position = 0;
    int count = 0;
    for (int i = 0; i < 8; i++)
    {
        if (IRVal[i] == '1') // For White
        {
            position += panelWeights[i];
            count++;
        } 
    }
    if (count == 0) //checking for zero division
      count = 1;
      
    error = position/(count * 10);   
    integral = integral + error;
    print(error)
    
    correction = KP * error + KI * integral + KD*(error - prevError); //calculating the correction
    
    print(correction)
    double leftSpeed = NORMAL_SPEED + correction;
    double rightSpeed = NORMAL_SPEED - correction; 
    
    //Limiting the max speed
    if (leftSpeed > MAX_SPEED)
        leftSpeed = MAX_SPEED;
    if (rightSpeed > MAX_SPEED)
        rightSpeed = MAX_SPEED;
    if (leftSpeed < -MAX_SPEED)
        leftSpeed = -MAX_SPEED;
    if (rightSpeed < -MAX_SPEED)
        rightSpeed = -MAX_SPEED;
     
    motors[0]->setVelocity(leftSpeed);
    motors[1]->setVelocity(rightSpeed);
    
    prevError = error;
}

void Solenoid(int state)
{
    if (state==1){linear[2]->setPosition(0.01);}
    else{linear[2]->setPosition(0.0);}
}

void Gripper_Lifter(bool iflift,int counter) //counter loop to 35 to get to 60 degree position
{
     if (iflift){servo->setPosition(counter*0.03);}
     else{servo->setPosition((target_lift-1-counter)*0.03);}
}


void Gripper(int dir, int current_pos)
{
    linear[0]->setPosition(current_pos*0.001+dir*0.001);
    linear[1]->setPosition(current_pos*0.001+dir*0.001);
}

