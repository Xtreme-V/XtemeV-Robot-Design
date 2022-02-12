#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define TIME_STEP 64
#define MAX_SPEED 6.28

//prototyping functions
void start_motors(Motor** motors, double speed, int mode);
void stop_motors(Motor** motors);
void left_turn(Motor** motors, PositionSensor** encoders, double* initial_encoder_values, bool* turn_left);

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  std::string motor_names[2] = {"left_motor", "right_motor"};
  std::string encoder_names[2] = {"left_encoder", "right_encoder"};

  Motor* motors[2];
  PositionSensor* encoders[2];


  for (int i=0; i<2; i++){
    motors[i] = robot->getMotor(motor_names[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0);

    encoders[i] = robot->getPositionSensor(encoder_names[i]);
    encoders[i]->enable(TIME_STEP);
  }

  start_motors(motors, MAX_SPEED, 2);

  double limit = 1;
  double wheel_radius = 0.035; //35mm
  
  double initial_encoder_values[2];
  bool saved_encoder_values = false;
  bool* pturn_left;
  bool turn_left = true;

  pturn_left = &turn_left;
  /*
  initial_encoder_values[0] = (double)encoders[0]->getValue();
  initial_encoder_values[1] = (double)encoders[1]->getValue();
  
  std::cout << initial_encoder_values[0] << " & " << initial_encoder_values[1] << std::endl;
  std::cout << encoders[0]->getValue() << " & " << encoders[0]->getValue() << std::endl;
  */

  while (robot->step(TIME_STEP) != -1) {
    if (saved_encoder_values == false){
      initial_encoder_values[0] = encoders[0]->getValue();
      initial_encoder_values[1] = encoders[1]->getValue();

      saved_encoder_values = true;
    }

    std::cout << "Left Encoder Value: " << encoders[0]->getValue() - initial_encoder_values[0] << " & " << "Right Encoder Value: " << encoders[1]->getValue() - initial_encoder_values[1] << std::endl;
    std::cout << "Left Wheel Distance: " << ( encoders[0]->getValue() - initial_encoder_values[0] ) * wheel_radius << " & " << "Right Wheel Distance: " << ( encoders[1]->getValue() - initial_encoder_values[1] ) * wheel_radius << std::endl;

    if( ( encoders[0]->getValue() - initial_encoder_values[0] ) * wheel_radius >= limit && ( encoders[1]->getValue() - initial_encoder_values[1] ) * wheel_radius >= limit){
      stop_motors(motors);
    }

    //left_turn(motors, encoders, initial_encoder_values, pturn_left);
    //std::cout << encoders[0]->getValue() << " & " << encoders[0]->getValue() << std::endl;
  };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}

void start_motors(Motor** motors, double speed, int mode){
  //straight
  if (mode == 0){
    for (int i=0; i<2; i++){
      motors[i]->setVelocity(speed);
    }
  }
  //right turn
  else if(mode == 1){
    motors[0]->setVelocity(speed);
    motors[1]->setVelocity(-speed);
  }
  //left turn
  else if(mode == 2){
    motors[0]->setVelocity(-speed);
    motors[1]->setVelocity(speed);
  }
}

void stop_motors(Motor** motors){
  for (int i=0; i<2; i++){
    motors[i]->setVelocity(0.0);
  }
}

void left_turn(Motor** motors, PositionSensor** encoders, double* initial_encoder_values, bool* turn_left){
  if(*turn_left == true){
    start_motors(motors, MAX_SPEED, 2);
    initial_encoder_values[0] = encoders[0]->getValue();
    initial_encoder_values[1] = encoders[1]->getValue();

    *turn_left = false;
  }
  
  if (encoders[0]->getValue() - initial_encoder_values[0] <= -4.0391 && encoders[1]->getValue() - initial_encoder_values[1] >= 4.0391){
    stop_motors(motors);
  }
}