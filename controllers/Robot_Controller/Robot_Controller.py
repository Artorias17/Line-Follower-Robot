"""Robot_Controller"""
from controller import Robot
import random

# create the Robot instance.
robot = Robot()

# Set Timestep as multiples of WorldInfo/basicTimeStep
TIME_STEP = int(robot.getBasicTimeStep())*1

# Rotational Motors named as the wheel they are attached to
WHEEL_NAMES = ["wheelFrontRight", "wheelBackRight", "wheelBackLeft", "wheelFrontLeft"]

# List of distance sensor names
IR_SENSOR_NAMES = ["IRSensorMiddleLeft", "IRSensorMiddleRight", "IRSensorRight", "IRSensorLeft"]

# Max velocity in rads/sec (due to Rotational Motor)
BASE_VELOCITY = 5
MAX_VELOCITY = 10

# Instantiating the end-effectors and sensors
wheel = {wheel_name:robot.getDevice(wheel_name) for wheel_name in WHEEL_NAMES}
IR_sensor = {sensor_name:robot.getDevice(sensor_name) for sensor_name in IR_SENSOR_NAMES}

# Set the rotationalMotor target position to infinity for endless rotational motion
# based on velocity. 
for wheel_name in wheel.keys():
    wheel[wheel_name].setPosition(float("inf"))
    wheel[wheel_name].setVelocity(0.0)

# Enable the distance sensor(s) and use TIME_STEP for update interval 
for sensor_name in IR_sensor.keys():
    IR_sensor[sensor_name].enable(TIME_STEP)


# Some methods for robot movement
def move(balance):
    
    vel_right = BASE_VELOCITY - balance
    vel_left = BASE_VELOCITY + balance
    
    if vel_right > 10:
        vel_right= MAX_VELOCITY
    elif vel_right < -10:
        vel_right= -MAX_VELOCITY
        
    
    if vel_left > 10:
        vel_left= MAX_VELOCITY
    elif vel_left < -10:
        vel_left= -MAX_VELOCITY
    
    wheel["wheelFrontRight"].setVelocity(vel_right)
    wheel["wheelBackRight"].setVelocity(vel_right)
    
    wheel["wheelFrontLeft"].setVelocity(vel_left)
    wheel["wheelBackLeft"].setVelocity(vel_left)
    


# PID variables
last_error = 0
balance=0
kp= 4
kd = 2
ki = 0.5
intg = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    
    # Reading the sensors and print the data
    IR_sensor_data = {"Middle Left": IR_sensor["IRSensorMiddleLeft"].getValue(),
                      "Middle Right": IR_sensor["IRSensorMiddleRight"].getValue(),
                      "Left": IR_sensor["IRSensorLeft"].getValue(),
                      "Right": IR_sensor["IRSensorRight"].getValue()}
    
    # Binarize values
    # 1 == error
    IR_sensor_data["Middle Left"] = 1 if IR_sensor_data["Middle Left"] == 1000 else 0
    IR_sensor_data["Middle Right"] = 1 if IR_sensor_data["Middle Right"] == 1000 else 0
    IR_sensor_data["Left"] = 1 if IR_sensor_data["Left"] == 1000 else 0
    IR_sensor_data["Right"] = 1 if IR_sensor_data["Right"] == 1000 else 0
    
    
    # total number of active sensors (detected error)
    d = IR_sensor_data["Middle Right"] + IR_sensor_data["Middle Left"] + IR_sensor_data["Left"] + IR_sensor_data["Right"]
    
    # Wighting the sensors
    IR_sensor_data["Left"] *= -2
    IR_sensor_data["Middle Left"] = -1
    IR_sensor_data["Middle Right"] = 1
    IR_sensor_data["Right"] *= 2
    
    
    # Weighted average to get the postion of deviation
    n = IR_sensor_data["Middle Right"] + IR_sensor_data["Middle Left"] + IR_sensor_data["Left"] + IR_sensor_data["Right"]
    
    if d != 0: 
        avg = n/d
    else:
        avg = n
    
    # PID error and balance calculations
    error = avg
    intg += error
    balance = kp*error + kd*(error-last_error) + ki*(intg)
    last_error = error
    
    print("Current Error", error)
    print("Balance", balance)
    
    
    move(balance)
