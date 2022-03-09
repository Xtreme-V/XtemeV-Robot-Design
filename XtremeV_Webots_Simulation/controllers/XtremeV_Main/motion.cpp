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

    // std::cout << "Rotated angle error: " << angle_error << std::endl;
}

void motion::start_motors(Motor** motors, double speed, int mode){
  // std::cout << speed << std::endl;

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

void motion::forward(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, int distance)
{
  double lambda = distance/35;
  double ang=pSensors[0] -> getValue();
  double ang1=ang;
  // cout<<ang1<<endl;
  while (robot->step(TIME_STEP) != -1)
  {
    //cout<<ang1<<endl;
    ang1=pSensors[0] -> getValue();
    // cout<<ang1<<endl;
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

int motion::Wall_Follower(Robot* robot, Motor** motors, DistanceSensor** dSensors, Gyro* gyro, PositionSensor** pSensors, double wall_prev_val, double* wall_gyro_total){
  double distances[5];
  for(int j=0;j<5;j++)
  {
    distances[j]=dSensors[j]->getValue();
  }

  double center_err=distances[0]-900;
  // double align_err=distances[1]-distances[4];
  double velocity_err;

  *wall_gyro_total += gyro->getValues()[1] * TIME_STEP * 0.001;

  // std::cout << "Gyro Value: " << *wall_gyro_total << std::endl;

  if (wall_prev_val == NULL){
    velocity_err=motion::kp1*center_err;
  }
  else{
    velocity_err = motion::kp1 * center_err + motion::kd1 * (center_err - wall_prev_val);
  }

  double Left_V=NORMAL_SPEED-velocity_err;
  double Right_V=NORMAL_SPEED+velocity_err;

  // std::cout << "Current motor speed: " << motors[0]->getVelocity() << " " << motors[1]->getVelocity() << std::endl;
  // std::cout << "Distance sensor values: " << dSensors[0]->getValue() << " " << dSensors[1]->getValue() << " " << dSensors[2]->getValue() << std::endl;
  // std::cout << "Next motor speed: " << std::max(std::min(double(MAX_SPEED),Left_V),double(-MAX_SPEED)) << " " << std::max(std::min(double(MAX_SPEED),Right_V),double(-MAX_SPEED)) << std::endl;

  //std::cout << center_err<<endl;//std::max(std::min(float(MAX_SPEED),Left_V),float(-MAX_SPEED))<<"  "<< std::max(std::min(float(MAX_SPEED),Right_V),float(-MAX_SPEED)) <<endl;
  if (distances[1]>2500 || distances[0]>2500 || distances[2]<1500)
  {
    motion::turning(robot, motors, gyro, -1*(*wall_gyro_total));
    *wall_gyro_total = 0;

    if (distances[2]<3000)
    {
      if(distances[0]>distances[1])
      {
      motion::turning(robot, motors, gyro, -180 * 3.14/180);
      return 1;
      }
      else
      {
      motion::turning(robot, motors, gyro, -180 * 3.14/180);
      return 1;
      }
      
    }
    motion::forward(robot, motors, pSensors, dSensors, motion::d);
    // std::cout << distances[0]<< " 2Wall "<<distances[1]<<endl;
    return 0;
  }

  motors[0]->setVelocity(std::max(std::min(double(MAX_SPEED),Left_V),double(-MAX_SPEED)));
  motors[1]->setVelocity(std::max(std::min(double(MAX_SPEED),Right_V),double(-MAX_SPEED)));

  wall_prev_val = center_err;

  return 2;
}

int motion::One_Wall_Follow(Robot* robot, Motor** motors, DistanceSensor** dSensors, PositionSensor** pSensors, char Dir)
{
  int i=0;
  int mark=1;  
  double distances[5];
  for(int j=0;j<5;j++)
  {
    distances[j]=dSensors[j]->getValue();
  }
  if (Dir=='l')
  {
    i=1;
    mark=-1;
  }
  double align_err=distances[1-i]-distances[4-i];
  float velocity_err=motion::kp2*align_err;
  float Left_V=NORMAL_SPEED+mark*velocity_err;
  float Right_V=NORMAL_SPEED-mark*velocity_err;
    
  motors[0]->setVelocity(std::max(std::min(float(MAX_SPEED),Left_V),float(-MAX_SPEED)));
  motors[1]->setVelocity(std::max(std::min(float(MAX_SPEED),Right_V),float(-MAX_SPEED)));
  if (distances[i]<2500)
  {
    //std::cout << " exit "<<endl;
    motion::forward(robot, motors, pSensors, dSensors, motion::d);
    // std::cout << distances[0]<< " 1Wall "<<distances[1]<<endl;
    return 0;
  }
  return 3+i;
}

int motion::No_Wall_Follow(Robot* robot, Motor** motors, DistanceSensor** dSensors, PositionSensor** pSensors)
{
  double distances[5];
  for(int j=0;j<5;j++)
  {
    distances[j]=dSensors[j]->getValue();
  }
  motors[0]->setVelocity(MAX_SPEED);
  motors[1]->setVelocity(MAX_SPEED);
  if (distances[0]<2500 || distances[1]<2500)
  {
    //std::cout << " exit "<<endl;
    motion::forward(robot, motors, pSensors, dSensors, motion::d);
    // std::cout << distances[0]<< " 0Wall "<<distances[1]<<endl;
    return 0;
  }
  return 5;
}

void motion::unit(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, Gyro* gyro, double* wall_gyro_total)
{
  double distances[5];
  double wall_prev_val;

  for(int j=0;j<5;j++)
  {
    distances[j]=dSensors[j]->getValue();
  }
  if (motion::A) 
  { 
    if ((distances[1]<2500 && distances[0]<2500 && A==1) || A==2)
    {
    // cout<<motion::A<<endl;
    motion::A=motion::Wall_Follower(robot, motors, dSensors, gyro, pSensors, wall_prev_val, wall_gyro_total);
    }
    else if ((distances[0]<2500 && distances[1]>2500 && A==1) || A==4)
    {
      // cout<<motion::A<<endl;
      motion::A=motion::One_Wall_Follow(robot, motors, dSensors, pSensors, 'l');
    }
    else if ((distances[1]<2500 && distances[0]>2500 && A==1) || A==3)
    {
      // cout<<motion::A<<endl;
      motion::A=motion::One_Wall_Follow(robot, motors, dSensors, pSensors, 'r');
    }
    else
    {
      // cout<<motion::A<<endl;
      motion::A=No_Wall_Follow(robot, motors, dSensors, pSensors);
    }
  }
  else
  {
    //forward(370);
    if (!(is_wall(dSensors, 'l')))
    {
      // cout<<"turn90"<<endl;
      motion::turning(robot, motors, gyro, 90 * 3.14/180);
      
    }
    else if (!(is_wall(dSensors, 'f')))
    {
      
    }
    else if (!(is_wall(dSensors, 'r')))
    {
      // cout<<"turn90r"<<endl;
      motion::turning(robot, motors, gyro, -90 * 3.14/180);
      
    } 
    else
    {
      // cout<<"turn180"<<endl;
      
      if(distances[0]>distances[1])
      {
      motion::turning(robot, motors, gyro, 180 * 3.14/180);
      //return 1;
      }
      else
      {
      motion::turning(robot, motors, gyro, -180 * 3.14/180);
      //return 1;
      }
      
    }
      //motion::turning(robot, motors, gyro, 180 * 3.14/180);
    

    
    motion::A=1;
  }
}
/*
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
*/
void motion::general_forward(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance){
   double expected_angle = distance/35; // wheel radius = 35 mm
   double initial_reading = pSensors[0]->getValue();
   double initial_reading_alt = pSensors[1]->getValue();

  while( robot->step(TIME_STEP) != -1 ){
    if (distance < 0){
      motors[0]->setVelocity(-NORMAL_SPEED);
      motors[1]->setVelocity(-NORMAL_SPEED);
    }
    else{
      motors[1]->setVelocity(NORMAL_SPEED);
      motors[0]->setVelocity(NORMAL_SPEED);
    }

  //  std::cout << initial_reading - pSensors[0]->getValue() << ", " << initial_reading_alt - pSensors[1]->getValue() << std::endl;

    if (std::abs( pSensors[0]->getValue() - initial_reading ) >= std::abs(expected_angle) && std::abs(pSensors[1]->getValue() - initial_reading_alt) >= std::abs(expected_angle) ){
      break;
    }
  }

  motors[0]->setVelocity(0.0);
  motors[1]->setVelocity(0.0);
}

void motion::general_forward_fast(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance){
   double expected_angle = distance/35; // wheel radius = 35 mm
   double initial_reading = pSensors[0]->getValue();
   double initial_reading_alt = pSensors[1]->getValue();

  while( robot->step(TIME_STEP) != -1 ){
    if (distance < 0){
      motors[0]->setVelocity(-MAX_SPEED);
      motors[1]->setVelocity(-MAX_SPEED);
    }
    else{
      motors[1]->setVelocity(MAX_SPEED);
      motors[0]->setVelocity(MAX_SPEED);
    }

  //  std::cout << initial_reading - pSensors[0]->getValue() << ", " << initial_reading_alt - pSensors[1]->getValue() << std::endl;

    if (std::abs( pSensors[0]->getValue() - initial_reading ) >= std::abs(expected_angle) && std::abs(pSensors[1]->getValue() - initial_reading_alt) >= std::abs(expected_angle) ){
      break;
    }
  }

  motors[0]->setVelocity(0.0);
  motors[1]->setVelocity(0.0);
}

void motion::forward_pid(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance)
{
  double expected_angle = distance/35;
  double initial_reading = pSensors[0]->getValue();

  double kp_f = 8;
  double ki_f = 0;
  double kd_f = 0;

  double acc_error = 0;
  double prev_error_f;
  double error;

  double motor_speed;

  // std::cout << "Initial reading: " << initial_reading << std::endl;

  while (robot->step(TIME_STEP) != -1){
    error = expected_angle - (pSensors[0]->getValue() - initial_reading);
    acc_error += error;

    if (prev_error_f != NULL){
      motor_speed = kp_f * error + ki_f * acc_error + kd_f * (error - prev_error_f);
    }
    else{
      motor_speed = kp_f * error + ki_f * acc_error;
    }

    prev_error_f = error;

    if (std::abs(error) <= 0.01){
      motors[0]->setVelocity(0);
      motors[1]->setVelocity(0);
      break;
    }

    motor_speed = std::max( std::min(motor_speed, (double)MAX_SPEED), -1*(double)MAX_SPEED );

    motors[0]->setVelocity(motor_speed);
    motors[1]->setVelocity(motor_speed);
  }

  // std::cout << "Final value: " << pSensors[0]->getValue() << std::endl;
  // std::cout << "Distance: " << std::abs( initial_reading - pSensors[0]->getValue() ) * 35 << std::endl;
}
void motion::forward_pid_slow(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance)
{
  double expected_angle = distance/35;
  double initial_reading = pSensors[0]->getValue();

  double kp_f = 1;
  double ki_f = 0;
  double kd_f = 0;

  double acc_error = 0;
  double prev_error_f;
  double error;

  double motor_speed;

  // std::cout << "Initial reading: " << initial_reading << std::endl;

  while (robot->step(TIME_STEP) != -1){
    error = expected_angle - (pSensors[0]->getValue() - initial_reading);
    acc_error += error;

    if (prev_error_f != NULL){
      motor_speed = kp_f * error + ki_f * acc_error + kd_f * (error - prev_error_f);
    }
    else{
      motor_speed = kp_f * error + ki_f * acc_error;
    }

    prev_error_f = error;

    if (std::abs(error) <= 1){
      motors[0]->setVelocity(0);
      motors[1]->setVelocity(0);
      break;
    }

    motor_speed = std::max( std::min(motor_speed, (double)MAX_SPEED*0.5), -1*(double)MAX_SPEED*0.5 );

    motors[0]->setVelocity(motor_speed);
    motors[1]->setVelocity(motor_speed);
  }

  // std::cout << "Final value: " << pSensors[0]->getValue() << std::endl;
  // std::cout << "Distance: " << std::abs( initial_reading - pSensors[0]->getValue() ) * 35 << std::endl;
}
