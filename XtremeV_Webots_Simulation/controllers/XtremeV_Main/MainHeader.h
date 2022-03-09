#ifndef MAINHEADER_H
#define MAINHEADER_H

#include <algorithm>
#include <string>
#include <vector>
#include <math.h> 
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <fstream>

#define TIME_STEP 16
#define NORMAL_SPEED 5
#define MAX_SPEED 7
#define KP 0.2
#define KI 0.01
#define KD 0.02
#define OBJ_KP 1.5
#define OBJ_KD 0.1
#define CAMERA_WIDTH 64
#define STEP_ANGLE 0.1 // for color detection
#define K_TURN 0.8
#define WALL_KP 1.5
#define FWD_DISTANCE 850


//define objects
#define OBJECT 0
#define HOLE_CIRCLE 1
#define HOLE_SQUARE 2
#define RED_BALL 3
#define BLUE_BALL 4

//define TYPE
#define CYLINDER 0
#define CUBE 1
#define NO_SHAPE_DETECTED 2

//define colors
#define CYAN 0
#define MAGENTA 1
#define YELLOW 2
#define BLACK 3

#define XV_print(x)  cout << #x": " << x << "\n";

using namespace webots;
using namespace std;

namespace lineFollow
{
    string getIRValues(DistanceSensor **irPanel, int IR_THRESHOLD);
    void PID(Motor **motors, string IRVal, double *integral, double *prevError);
}

namespace hand
{
    void SetGripperPosition(Robot* robot, Motor **linear, int target_hand_position, int *current_hand_position);
    void Gripper( Motor **linear, int dir, int current_pos);
    void Gripper_Lifter(Motor *servo, bool iflift,int counter,int target_lift);
    void Solenoid(Motor **linear, int state);
    void liftHand(Motor *servo,Robot *robot, bool iflift,int target_lift);
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

    double kp1 = 0.02;
    double kd1 = 0;
    double kp2 = 0.06;
    int A = 0; // select one wall follow or two wall follow
    int d = 250; // distance going forward

    void init_turn(Motor** motors, double angle);
    void turn(Motor** motors, Gyro* gyro);
    void start_motors(Motor** motors, double speed, int mode);
    void stop_motors(Motor** motors);
    void forward(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, int distance);
    void stop(Motor** motors);
    bool is_wall(DistanceSensor** dSensors, char x);
    void turning(Robot* robot, Motor** motors, Gyro* gyro, double angle);
    void unit(Robot* robot, Motor** motors, PositionSensor** pSensors, DistanceSensor** dSensors, Gyro* gyro, double* wall_gyro_total);
    double increase_speed(double ref_speed, double offset);
    void general_forward(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance);
    void forward_pid(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance);
    int One_Wall_Follow(Robot* robot, Motor** motors, DistanceSensor** dSensors, PositionSensor** pSensors, char Dir);
    int No_Wall_Follow(Robot* robot, Motor** motors, DistanceSensor** dSensors, PositionSensor** pSensors);
    void forward_pid(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance);
    void general_forward_fast(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance);
    int Wall_Follower(Robot* robot, Motor** motors, DistanceSensor** dSensors, Gyro* gyro, PositionSensor** pSensors, double wall_prev_val, double* wall_gyro_total);
    void forward_pid_slow(Robot* robot, Motor** motors, PositionSensor** pSensors, double distance);
}

namespace objDetect
{
    void followObject(Motor **motors, double obj_x, double obj_y, double *obj_prevError, bool *arrived);
}

namespace imageProcessing
{
    int object[6] = {10, 191, 0, 25, 255, 255};
    int holes[6]  = {90, 0, 0, 125, 5, 255};

    //color List
    int cyan[6] = { 80, 190, 190, 100, 255, 255 };    // {90, 227.46, 222.1}
    int magenta[6] = { 140, 190, 190, 150, 255, 255}; // {150, 227.46, 210}
    int yellow[6] = { 20, 190, 190, 40, 255, 255}; // {30, 224, 229}
    int black[6] = { 0, 0, 0, 5, 5, 5}; //{0, 0, 0}
    int* colors[4] = {cyan, magenta, yellow, black};

    cv::Mat imageToMatrix(const unsigned char* image, int width, int height);
    bool detectColor(cv::Mat src, int width, int height, int color, float colorThreshold);
    std::vector<cv::Point2f> centroidDetection(cv::Mat threshold);
    cv::Point2f objectTracking(cv::Mat img, int width, int height, int object_type, webots::Display* display, bool activateDisplay);
    void displayImage( int width, int height, cv::Mat src, webots::Display* display);
    int detectShape(cv::Mat src, int width, int height, int threshold, webots::Display* display, bool activate_display);
    vector<vector<double>> detectShapes(cv::Mat src, int width, int height, int threshold, webots::Display* display, bool activate_display);
}

#endif