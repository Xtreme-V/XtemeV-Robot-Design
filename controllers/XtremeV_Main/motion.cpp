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

    if (std::abs(motor_speed) >= std::abs(MAX_SPEED)){
      motor_speed = MAX_SPEED * motor_speed / std::abs(motor_speed);
    }

    motion::start_motors(motors, motor_speed, 2);

    if(std::abs(motion::angle_error) <= 0.01){
        motion::turn_flag = false;
        motion::integral_rotate = 0;
        motion::stop_motors(motors);
    }

    std::cout << "Rotated angle error: " << angle_error << std::endl;
}

void motion::start_motors(Motor** motors, double speed, int mode){
  std::cout << speed << std::endl;

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

double motion::increase_speed(double ref_speed, double offset){
  return std::min( std::max( ref_speed + offset, -1*(double)MAX_SPEED ), (double)MAX_SPEED );
}

void motion::lineFollowerForward(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, int distance)
{
  //variables for PID
  double Kp = 10;
  double Ki = 0;
  double Kd = 0;

  double acc_error = 0;
  double prev_error = 0; 

  double error; //positive if the left distance sensor is larger than the right distance sensor
  double correction;
  double left_motor_speed;
  double right_motor_speed;
  
  double lambda = distance/35; //wheel radius = 35 mm
  double ang= pSensors[0] -> getValue();
  double ang1=ang;

  std::cout << ang1 << std::endl;

  while (robot->step(TIME_STEP) != -1)
  {
    
    error = ( dSensors[0]->getValue() - dSensors[1]->getValue() ) / 4000;
    acc_error += error;

    correction = Kp * error + Ki * acc_error + Kd * (error - prev_error); 
    prev_error = error; 
    
    left_motor_speed = motion::increase_speed(NORMAL_SPEED, -correction);
    right_motor_speed = motion::increase_speed(NORMAL_SPEED, correction);
    
    std::cout << "Left wheel speed: " << left_motor_speed << " & " << "Right wheel speed: " << right_motor_speed << std::endl; 
    
    std::cout << ang1 << std::endl;
    ang1 = pSensors[0] -> getValue();

    if (ang1-ang>lambda)////-//////////////////////[12.5 change kranw]
    {
      for(int k=0;k<2;k++)
      {
        motors[k]->setVelocity(0);
      }
      return;
    }
    
    motors[0]->setVelocity(left_motor_speed);
    motors[1]->setVelocity(right_motor_speed);
    
  /*
   motors[0]->setVelocity(MAX_SPEED);
   motors[1]->setVelocity(MAX_SPEED);
   */
  }
}

void motion::stop(Motor** motors)
{
  for(int k=0;k<2;k++)
  {
    motors[k]->setVelocity(0);
  }
}

bool motion::is_wall(DistanceSensor** dSensors, char x)
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

void motion::turning(Robot* robot, Motor** motors, Gyro* gyro, double angle){
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

void motion::unit(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, Gyro* gyro)
{
  motion::lineFollowerForward(robot, motors, pSensors, dSensors, 370);
  if (!(motion::is_wall(dSensors, 'l')))
  {
    motion::turning(robot, motors, gyro, 90 * 3.14/180);
    return;
  }
  else if (!(motion::is_wall(dSensors, 'f')))
  {
    return;
  }
  else if (!(motion::is_wall(dSensors, 'r')))
  {
    motion::turning(robot, motors, gyro, -90 * 3.14/180);
    return;
  } 
  else
  {
    motion::turning(robot, motors, gyro, 180 * 3.14/180);
    return;
  } 
}

void motion::align_to_walls(Robot* robot, Motor** motors, DistanceSensor** dSensors){
  double Kp = 1.2;
  double Ki = 0;
  double Kd = 0;

  double acc_error = 0;
  double prev_error = 0; 

  double error; //positive if the left distance sensor is larger than the right distance sensor
  double correction;
  double left_motor_speed;
  double right_motor_speed;

  while (robot->step(TIME_STEP) != -1){
    std::cout << dSensors[0]->getValue() << " and " << dSensors[1]->getValue() << std::endl;
    error = ( dSensors[0]->getValue() - dSensors[1]->getValue() ) / 4000;
    acc_error += error;

    correction = Kp * error + Ki * acc_error + Kd * (error - prev_error); 
    prev_error = error; 
    
    left_motor_speed = NORMAL_SPEED + correction;
    right_motor_speed = NORMAL_SPEED - correction;

    motors[0]->setVelocity(left_motor_speed);
    motors[1]->setVelocity(right_motor_speed);

    if (std::abs(error) <= 0.01){
      break;
    }
  }
}