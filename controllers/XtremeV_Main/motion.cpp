#include "MainHeader.h"

void motion::init_turn(Motor** motors, double angle){
    motion::turn_flag = true;
    motion::angle_error = angle;
    motion::prev_angle_error = angle;
    motion::integral_rotate = 0;

    motion::start_motors(motors, MAX_SPEED, 0);
}

void motion::turn(Motor** motors, Gyro* gyro){
    motion::angle_error -= gyro->getValues()[1] * TIME_STEP * 0.001;
    motion::integral_rotate += motion::angle_error;

    double motor_speed = motion::KP_rotate * motion::angle_error + motion::KI_rotate * motion::integral_rotate + motion::KD_rotate * (motion::angle_error - motion::prev_angle_error);

    motion::prev_angle_error = motion::angle_error;

    motion::start_motors(motors, std::min(motor_speed, (double)MAX_SPEED), 2);

    if(std::abs(motion::angle_error) <= 0.01){
        motion::turn_flag = false;
        motion::integral_rotate = 0;
        motion::stop_motors(motors);
    }

    std::cout << "Rotated angle error: " << angle_error << std::endl;
}

void motion::start_motors(Motor** motors, double speed, int mode){
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

void motion::stop_motors(Motor** motors){
  for (int i=0; i<2; i++){
    motors[i]->setVelocity(0.0);
  }
}