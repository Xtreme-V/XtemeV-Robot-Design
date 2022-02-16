#ifndef MAINHEADER_H
#define MAINHEADER_H

#include <algorithm>
#include <string>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 32
#define NORMAL_SPEED 5
#define MAX_SPEED 7
#define IR_THRESHOLD 250
#define KP 1.0
#define KI 0
#define KD 0.5

#define print(x)  cout << #x": " << x << "\n";

using namespace webots;
using namespace std;

namespace lineFollow
{
    string getIRValues(DistanceSensor **irPanel);
    void PID(Motor **motors, string IRVal, double *integral, double *prevError);
}

namespace hand
{
    void SetGripperPosition(Robot* robot, Motor **linear, int target_hand_position, int *current_hand_position);
    void Gripper( Motor **linear, int dir, int current_pos);
    void Gripper_Lifter(Motor *servo, bool iflift,int counter,int target_lift);
    void Solenoid(Motor **linear, int state);
}

namespace motion{
    bool turn_flag = false;
    double angle_error;
    double integral_rotate;
    double prev_angle_error;
    double motor_speed;

    double KP_rotate = 8;
    double KI_rotate = 0;
    double KD_rotate = 0;

    void init_turn(Motor** motors, double angle);
    void turn(Motor** motors, Gyro* gyro);
    void start_motors(Motor** motors, double speed, int mode);
    void stop_motors(Motor** motors);
    void lineFollowerForward(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, int distance);
    void stop(Motor** motors);
    bool is_wall(DistanceSensor** dSensors, char x);
    void turning(Robot* robot, Motor** motors, Gyro* gyro, double angle);
    void unit(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, Gyro* gyro);
    void align_to_walls(Robot* robot, Motor** motors, DistanceSensor** dSensors);
    double increase_speed(double ref_speed, double offset);
}

#endif