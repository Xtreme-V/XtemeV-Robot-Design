#ifndef MAINHEADER_H
#define MAINHEADER_H

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>

#define TIME_STEP 32
#define NORMAL_SPEED 5
#define MAX_SPEED 7
#define IR_THRESHOLD 250
#define KP 1.7
#define KI 0
#define KD 0.1

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
    void Gripper( Motor **linear, int dir, int current_pos);
    void Gripper_Lifter(Motor *servo, bool iflift,int counter,int target_lift);
    void Solenoid(Motor **linear, int state);
}

#endif