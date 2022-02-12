"""Mazesolver controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot

# create the Robot instance.
robot = Robot()


timestep = 1
motors=[]
Motornames=['wheel1','wheel2','wheel3','wheel4']

ds=[]
dsnames=['ds_l','ds_r','ds_f']

for i in range(4):
    motors.append(robot.getDevice(Motornames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)

for j in range(3):
    ds.append(robot.getDevice(dsnames[j]))
    ds[j].enable(timestep)

encoder=robot.getDevice('ps_1')
encoder.enable(timestep)

compass=robot.getDevice('compass')
compass.enable(timestep)

#########################################

def forward():
    ang=encoder.getValue()
    ang1=ang
    #print(ang)
    
    
    while robot.step(timestep) != -1:
        ang1=encoder.getValue()
        if ang1-ang>12.5:
            for k in range(4):
                motors[k].setVelocity(0)
            #print('Y')
            return
        for k in range(4):
            motors[k].setVelocity(14.5 )
 
def Bearing_inDegrees(A):
    rad=math.atan2(A[0],A[2])
    deg=(rad-1.5708)/math.pi*180
    if deg<0:
        deg=deg+360
    return round(deg,2)  

def turning(theta):
    if theta>0:
        cons=1
    else:
        cons=-1
    ang1=Bearing_inDegrees(compass.getValues())   
    ang2=ang1
    print(ang1)
    while (ang2-ang1)*cons<90:        
        
        left=4*cons
        right=-4*cons
        motors[0].setVelocity(left)
        motors[1].setVelocity(right)
        motors[2].setVelocity(left)
        motors[3].setVelocity(right)
        robot.step(timestep)
        ang2=Bearing_inDegrees(compass.getValues())
        if ang1<=100 and cons<0 and ang2>260:
            ang2=ang2-360
        if ang1>260 and cons>0 and ang2<100:
            ang2=ang2+360  
        #print('ang2',ang2-ang1)
    error=90-cons*(ang2-ang1)
    prev_error=0
    Kp=0.5
    Kd=0
    base_speed=0.01
    while abs(error)>0.01:
        #print(error)
        ang2=Bearing_inDegrees(compass.getValues())
        if ang1<100 and cons<0 and ang2>=260:
            ang2=ang2-360
        if ang1>260 and cons>0 and ang2<=100:
            ang2=ang2+360        
        error=90-(ang2-ang1)*cons
        if error<0:
            cons1=-1
        else:
            cons1=1
        Control=error*Kp+(error-prev_error)*Kd
        left=cons*(cons1*base_speed+Control)
        right=cons*(-cons1*base_speed-Control)
        motors[0].setVelocity(left)
        motors[1].setVelocity(right)
        motors[2].setVelocity(left)
        motors[3].setVelocity(right)
        robot.step(timestep)
        
def stop():
    for k in range(4):
        motors[k].setVelocity(0)        

def is_wall(x):
    if x=='left':
        if ds[0].getValue()<4000:
            return True
        return False
    elif x=='front':
        if ds[2].getValue()<4000:
            return True
        return False
    else:
        if ds[1].getValue()<4000:
            return True
        return False
        

def unit():
    forward()
    if not(is_wall('left')):
        turning(-90)
        return
    elif not(is_wall('front')):
        return
    elif not(is_wall('right')):
        turning(90)
        print('x')
        return
    else:
        turning(90)
        turning(90)
        return

while robot.step(timestep) != -1:
    unit()

