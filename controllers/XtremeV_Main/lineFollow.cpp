#include "MainHeader.h"

namespace lineFollow
{
    string getIRValues(DistanceSensor **irPanel)
    {
        string panelVals = "";
        for (int i = 0; i < 8; i++)
        {
            if (irPanel[i] -> getValue() > IR_THRESHOLD) // For Black
                panelVals += "0";
            else  //For White
                panelVals += "1"; 
        }
        return panelVals;
    }

    void PID(Motor **motors, string IRVal, double *integral, double *prevError)
    {
        int panelWeights[8] = {-400,-300,-200,-100,100,200,300,400};
        double position = 0;
        double error = 0;
        double correction = 0;
        int count = 0;

        for (int i = 0; i < 8; i++)
        {
            if (IRVal[i] == '1') // For White
            {
                position += panelWeights[i];
                count++;
            } 
        }
        
        //checking for zero division
        if (count == 0) 
        count = 1;
        
        error = position/(count * 10);   
        *integral += error;
        print(error)
        
        correction = KP * error + KI * (*integral) + KD*(error - *prevError); //calculating the correction
        
        // print(correction)
        double leftSpeed = NORMAL_SPEED + correction;
        double rightSpeed = NORMAL_SPEED - correction; 
        
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
        
        *prevError = error;
    }
}