#include <iostream>
#include <algorithm>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>

#include "motion.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define TIME_STEP 32
#define NORMAL_SPEED 5
#define MAX_SPEED 7

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
  Gyro* gyro = robot->getGyro("gyro");

  for (int i=0; i<2; i++){
    motors[i] = robot->getMotor(motor_names[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0);

    encoders[i] = robot->getPositionSensor(encoder_names[i]);
    encoders[i]->enable(TIME_STEP);
  }

  gyro->enable(TIME_STEP);
  bool left_rotate = true;
  double angle_error = 1.5708;
  double motor_speed;
  double integral_rotate = 0;
  double prev_angle_error = angle_error;
  /*
  double KP_rotate = 2.5;
  double KI_rotate = 0;
  double KD_rotate = 0.2;
  */
  //start_motors(motors, MAX_SPEED, 2);

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
  motion::init_turn(motors, -2*1.5708);


  while (robot->step(TIME_STEP) != -1) {
    if (saved_encoder_values == false){
      initial_encoder_values[0] = encoders[0]->getValue();
      initial_encoder_values[1] = encoders[1]->getValue();

      saved_encoder_values = true;
    }

    if(motion::turn_flag){
      /*
      angle_error -= gyro->getValues()[1] * TIME_STEP * 0.001;
      integral_rotate += angle_error;

      motor_speed = KP_rotate * angle_error + KI_rotate * integral_rotate + KD_rotate * (angle_error - prev_angle_error);

      prev_angle_error = angle_error;

      start_motors(motors, std::min(motor_speed, (double)MAX_SPEED), 2);

      if(std::abs(angle_error) <= 0.01){
        left_rotate = false;
        integral_rotate = 0;
        stop_motors(motors);

        std::cout << "Rotated angle error: " << angle_error << std::endl;
      }

      std::cout << "Rotated angle error: " << angle_error << std::endl;
      */

      motion::turn(motors, gyro);
    }

    //std::cout << gyro->getValues()[0] << " & " << gyro->getValues()[1] << " & " << gyro->getValues()[2] << std::endl;
    /*
    std::cout << "Left Encoder Value: " << encoders[0]->getValue() - initial_encoder_values[0] << " & " << "Right Encoder Value: " << encoders[1]->getValue() - initial_encoder_values[1] << std::endl;
    std::cout << "Left Wheel Distance: " << ( encoders[0]->getValue() - initial_encoder_values[0] ) * wheel_radius << " & " << "Right Wheel Distance: " << ( encoders[1]->getValue() - initial_encoder_values[1] ) * wheel_radius << std::endl;

    if( ( encoders[0]->getValue() - initial_encoder_values[0] ) * wheel_radius >= limit && ( encoders[1]->getValue() - initial_encoder_values[1] ) * wheel_radius >= limit){
      stop_motors(motors);
    }
    */
    //left_turn(motors, encoders, initial_encoder_values, pturn_left);
    //std::cout << encoders[0]->getValue() << " & " << encoders[0]->getValue() << std::endl;
  };

  //Enter here exit cleanup code.
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