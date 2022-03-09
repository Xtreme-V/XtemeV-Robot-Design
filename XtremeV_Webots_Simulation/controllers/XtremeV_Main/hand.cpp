#include "MainHeader.h"

namespace hand
{
    void liftHand(Motor *servo,Robot *robot, bool iflift,int target_lift)
    {   
        int count = 1;
        while (robot->step(TIME_STEP) != -1)
        {
            if (count<target_lift)
            {
                hand::Gripper_Lifter(servo,iflift,count,target_lift);
                count++;
            }
            else
            {
                break;
            }
        }
    }
        
    void SetGripperPosition(Robot* robot, Motor **linear, int target_hand_position, int *current_hand_position)
    {

        while (robot->step(TIME_STEP) != -1)
        {   
            
            //Horizontal Move Control of the hand
            if (*current_hand_position<target_hand_position)
            {
                Gripper(linear,1, *current_hand_position);
                *current_hand_position+=1;
            }
            else if (*current_hand_position>target_hand_position)
            {
                Gripper(linear,-1, *current_hand_position);
                *current_hand_position-=1;    
            } 
            else
                break; 

        } 

    } 
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

    // void Solenoid(Motor **linear, int state)
    // {
    //     if (state==1)
    //         linear[2]->setPosition(0.01);
    //     else
    //         linear[2]->setPosition(0.0);
    // }

}