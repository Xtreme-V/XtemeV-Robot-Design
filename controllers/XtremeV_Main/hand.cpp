#include "MainHeader.h"

namespace hand
{
    void Gripper(Motor **linear, int dir, int current_pos)
    {
        linear[0]->setPosition(current_pos*0.001+dir*0.001);
        linear[1]->setPosition(current_pos*0.001+dir*0.001);
    }

    void Gripper_Lifter(Motor *servo, bool iflift,int counter,int target_lift) //counter loop to 35 to get to 60 degree position
    {
        if (iflift)
            servo->setPosition(counter*0.03);
        else
            servo->setPosition((target_lift-1-counter)*0.03);
    }

    void Solenoid(Motor **linear, int state)
    {
        if (state==1)
            linear[2]->setPosition(0.01);
        else
            linear[2]->setPosition(0.0);
    }

}