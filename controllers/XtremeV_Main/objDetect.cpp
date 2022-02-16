#include "MainHeader.h"

namespace objDetect
{
    void followObject(Motor **motors, double obj_x, double obj_y, double *obj_prevError, bool *arrived)
    {
        if (obj_y > 100)
        {
            motors[0]->setVelocity(0);
            motors[1]->setVelocity(0);
            *arrived = true;
            cout << "Robot Arrived" << endl;
            return;
        }
        double error = (obj_x - CAMERA_WIDTH)/10;
        double correction = 0;
        
        correction = OBJ_KP * error + OBJ_KD*(error - *obj_prevError); //calculating the correction
        
        XV_print(correction)
        double leftSpeed = NORMAL_SPEED + correction;
        double rightSpeed = NORMAL_SPEED - correction; 
        XV_print(leftSpeed)
        XV_print(rightSpeed)
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
        
        *obj_prevError = error;

    }
}