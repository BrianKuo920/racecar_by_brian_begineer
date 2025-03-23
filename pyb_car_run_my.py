# import  module

import pybullet as p
import pybullet_data     # you can get the build - in resourse , such as ground, car ... etc
import time     # set the rate of simulating


# setting up the simulation environment 
# connect to GUI
p.connect(p.GUI)  
print("Physics server connected:", p.isConnected())
# set search path : The reason is that you do not tpye full path to get the resourse 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
# getDataPath() is a function in pybullet_data .
# It can get the path of folder which built into pybullet

# set gravity
p.setGravity (0 , 0 , -9.8 )

# load plane module
p.loadURDF("plane.urdf")

# load car module
racecar = p.loadURDF( "racecar/racecar.urdf" , [0,0,0.2] , useFixedBase = False , globalScaling = 3)
# Common functions
# basePosition : 初始位置
# useFixedBase : 是否固定車體 if you type "True",the car will fix on the "basePosition" forever.
# globalScaling : 縮放整個模型 

# get serial numbers of the wheel joint and the steering joint
wheel_joints = [2,3,5,7] # these are the indexes of the joints in the "racecar.urtf"
steering_joints = [4,6]

# you can search the ID of joint by following code ( chatgpt )
# 取得總關節數
num_joints = p.getNumJoints(racecar)
# 印出每個關節的資訊
print("==== joint information ====")
for i in range(num_joints):
    joint_info = p.getJointInfo(racecar, i)
    joint_name = joint_info[1].decode("utf-8")  # 取得關節名稱 (bytes 轉 string)
    joint_type = joint_info[2]  # 取得關節類型
    print(f"Joint ID: {i}, Name: {joint_name}, Type: {joint_type}")

    

# set the control parameter of car
max_velocity = 10 # speed of car
max_force = 10 # torque
steering_angle = 0.5 # the max angular of steering

# define the control function of the car  
def racecar_control() :
    
    # system input
    key = p.getKeyboardEvents() # it can return a dictionary which let you know which key is pressed , released or kept pressing .

    throttle = 0 # initial velocity ( Similar to the concept of defining int )
    steering = 0 # initial steering


    if ord('e') in key :
        # this condition function can check whether you type the "e" key
        # ord() : return ASCII code that you type
        throttle = max_velocity # move forward
    elif ord('r') in key :
        throttle = - max_velocity # move backward
    else :
        throttle = 0 
    
    if p.B3G_LEFT_ARROW in key :
        steering = -steering_angle # turn left ( the angle is negetive )
    elif p.B3G_RIGHT_ARROW in key :
        steering = steering_angle # turn right 
    else : 
        steering = 0 
    
    # system output
    for wheel in wheel_joints : 
        p.setJointMotorControl2 ( racecar , wheel , p.VELOCITY_CONTROL , targetVelocity = throttle  , force=max_force )
    for steer in steering_joints : 
        p.setJointMotorControl2 ( racecar , steer , p.POSITION_CONTROL , targetPosition = steering )


while True:
   racecar_control()

   p.stepSimulation() 
   # If you execude this command , pybullet will calculate all the physical state once in this enviroment .
   # Therefore , if you wants to let your car move continuously , you must keep doing this command .

   time.sleep(1./240.)
   # This code runs the simulation at 240 Hz so that the animation doesn't happen too fast.
