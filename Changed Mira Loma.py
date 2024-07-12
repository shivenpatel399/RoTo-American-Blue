#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
# Value Error: This won't detect the device properly. Disconnect and Reconnect the cables and check Device Brower to see if it detects correctly
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
the_feeler = TouchSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=101)
watch = StopWatch()

# Write your program here.
Position  = "North"
mod_angle = 0
Error = 0 # All
Integral = 0 # Integral
Derivative = 0 # Derivative
Last_Error = 0 # Derivative
Kp = 0.3 # Proportional
Ki = 0.001 # Integral
Kd = 0.2 # Derivative
Turns = 0
Straights = 0.5
Moving = 0 # 0 is false
Remaining_Seconds = 0
Total_Distance = 0
Speed = 0
Straight_Distance = 0
def gyro_reset():
    gyro_sensor.reset_angle(0)
    while True:
        if gyro_sensor.angle() == 0:
            break
    while True:
        if gyro_sensor.speed() == 0:
            break
# AKA Gyro Calibration


def going_straight(distance):
    robot.reset()
    global Error
    global Integral
    global Derivative
    global Last_Error
    global Kp
    global Ki
    global Kd
    global Speed
    """
    Target = gyro_sensor.angle() # Integral and Proportional
    while robot.distance() <= distance:
        Angle = gyro_sensor.angle() # Both
        Error = Target - Angle # Proportional
        Integral = Integral + Error # Integral
        Derivative = Error - Last_Error # Derivative
        P = Error * Kp # Proportional
        I = Integral * Ki # Integral
        D = Derivative *  Kd # Derivative
        PID = P + I + D # PID
        robot.drive(90, PID)
        Error = Last_Error
    robot.stop()
    """
    # PID ^
    
    target = gyro_sensor.angle()
    gain = 5
    while robot.distance() <= distance:
        correction = target - gyro_sensor.angle()
        turn_power = correction * gain
        robot.drive(Speed, turn_power)
    robot.stop()
    
    # PD ^
    # This code wud've been fairly short, but due to errors involving drift in the motors, I'm using PID Tuning algorithm.
    """What this code is showing is while the distance requirement is not met, the robot will keep on runnning straight. But also, if the 
    angle changes due to drift, the gain multiplied by the difference in change of angle will be the turnpower (basically accelerating the motor when
    angle is changed), making the robot reorient to the angle it has to be on (more likely 0)"""
    # Once distance is met, robot stops and ur robot should be at same angle as before

def going_back(distance):
    robot.reset()
    global Error
    global Integral
    global Derivative
    global Last_Error
    global Kp
    global Ki
    global Kd
    global Speed
    """
    Target = gyro_sensor.angle() # Integral and Proportional
    while robot.distance() >= distance:
        Angle = gyro_sensor.angle() # Both
        Error = Target - Angle # Proportional
        Integral = Integral + Error # Integral
        Derivative = Error - Last_Error # Derivative
        P = Error * Kp # Proportional
        I = Integral * Ki # Integral
        D = Derivative *  Kd # Derivative
        PID = P + I + D # PID
        robot.drive(-90, PID)
        Error = Last_Error
    
    # PID ^
    """
    target = gyro_sensor.angle()
    gain = 5
    while robot.distance() >= distance:
        correction = target - gyro_sensor.angle()
        turn_power = correction * gain
        robot.drive((Speed * -1), turn_power)
    robot.stop() 
    
    # PD ^
    # Same idea as moving forward, but different motor rotational movement, now moving backwards. It definitely works.
def mod_gyro_angle(gyro_angle):
    global mod_angle
    if gyro_angle in range(-10,10):
        mod_angle = 0
    elif gyro_angle in range(-100,-80):
        mod_angle = -90
    elif gyro_angle in range(-190,-170):
        mod_angle = -180
    elif gyro_angle in range(-280,-260):
        mod_angle = -270
    elif gyro_angle in range(-370,-350):
        mod_angle = -360
    elif gyro_angle in range(80,100):
        mod_angle = 90
    elif gyro_angle in range(170,190):
        mod_angle = 180
    elif gyro_angle in range(260,280):
        mod_angle = 270
    elif gyro_angle in range(350,370):
        mod_angle = 360
def going_right(degrees): # degrees is a variable where when you write going_right(), you can write your desired degree into the parenthesis and it will work. However, its not 100% going to be the number you expect, so it can range from -2 to +2 of the number you ask for.
    mod_gyro_angle(gyro_sensor.angle())
    final = mod_angle + degrees
    while gyro_sensor.angle() < final:
        left_motor.run(143.5)
        right_motor.run(-143.5)
    left_motor.stop()
    right_motor.stop()
    # As your initial gyro angle is supposedly less than the expected gyro angle, it will turn right towards it and once it hits it, it stops smoothly 
    # No correction needed as angle is being corrected at the place (2 motor turn, not 1)
def going_left(degrees):
    mod_gyro_angle(gyro_sensor.angle())
    final = mod_angle + degrees
    while gyro_sensor.angle() > final:
        right_motor.run(143.5)
        left_motor.run(-143.5)
    right_motor.stop()
    left_motor.stop()
    # Same with Right Function, but different thing is initial angle is greater than final, so it works inversely.    

def north():
    global Position
    global Moving
    global Straights
    global Turns
    global Straight_Distance
    if Moving == 0:
        if Position == "North":
            Straights += 1
            Position = "North"
        elif Position == "East":
            Turns += 1
            Straights += 1
            Position = "North"
        elif Position == "West":
            Turns += 1
            Straights += 1
            Position = "North"
        elif Position == "South":
            Straights += 1
            Position = "South"
    elif Moving == 1:   
        if Position == "North":
            going_straight(Straight_Distance + 30)
            Position = "North"
        elif Position == "East":
            going_left(-90)
            going_straight(Straight_Distance + 5)
            Position = "North"
        elif Position == "West":
            going_right(90)
            going_straight(Straight_Distance + 5)
            Position = "North"
        elif Position == "South":
            going_back((-1 * Straight_Distance) + 10)
            Position = "South"

def south():
    global Position
    global Straights
    global Turns
    global Moving
    global Straight_Distance
    if Moving == 0:
        if Position == "North":
            Straights += 1
            Position == "North"
        elif Position == "East":
            Turns += 1
            Straights += 1
            Position = "South"
        elif Position == "West":
            Turns += 1
            Straights += 1
            Position = "South"
        elif Position == "South":
            Straights += 1
            Position = "South"
    elif Moving == 1:
        if Position == "North":
            going_back((-1 * Straight_Distance) + 10)
            Position == "North"
        elif Position == "East":
            going_right(90)
            going_straight(Straight_Distance + 5)
            Position = "South"
        elif Position == "West": 
            going_left(-90)
            going_straight(Straight_Distance + 5)
            Position = "South"
        elif Position == "South":
            going_straight(Straight_Distance + 30)
            Position = "South"
    
def east():
    global Position
    global Straights
    global Turns
    global Moving
    global Straight_Distance
    if Moving == 0:
        if Position == "North":
            Turns += 1
            Straights += 1
            Position = "East"
        elif Position == "East":
            Straights += 1
            Position = "East"
        elif Position == "West":
            Straights += 1
            Position = "West"
        elif Position == "South":
            Turns += 1
            Straights += 1
            Position = "East"
    elif Moving == 1:
        if Position == "North": 
            going_right(90)
            going_straight(Straight_Distance + 5)
            Position = "East"
        elif Position == "East":
            going_straight(Straight_Distance + 30)
            Position = "East"
        elif Position == "West":
            going_back((-1 * Straight_Distance) + 10)
            Position = "West"
        elif Position == "South": 
            going_left(-90)
            going_straight(Straight_Distance + 5)
            Position = "East"
def west():
    global Position
    global Turns
    global Straights
    global Moving
    global Straight_Distance
    if Moving == 0:
        if Position == "North":
            Turns += 1
            Straights += 1
            Position = "West"
        elif Position == "East":
            Straights += 1
            Position = "East"
        elif Position == "West":
            Straights += 1
            Position = "West"
        elif Position == "South":
            Turns += 1
            Straights += 1
            Position = "West"
    elif Moving == 1:
        if Position == "North":
            going_left(-90)
            going_straight(Straight_Distance + 5)
            Position = "West"
        elif Position == "East":
            going_back((-1 * Straight_Distance) + 10)
            Position = "East"
        elif Position == "West":
            going_straight(Straight_Distance + 30)
            Position = "West"
        elif Position == "South":
            going_right(90)
            going_straight(Straight_Distance + 5)
            Position = "West"
def time_function(Time):
    global Remaining_Seconds
    global Total_Distance
    global Speed
    Remaining_Seconds = Time - (Turns * (108057/100000))
    Total_Distance = Straights * 510
    Speed = Total_Distance/Remaining_Seconds

# Still need to write the 4 main functions, each with the 4 gyro conditionals - Done

Steps = [14,13,23,24,34,44,34,33,43,42,32,31,41,31,32,22,21,11,12] # Write Steps Here / Coords
Cycle = len(Steps) - 1
Final_Change = Cycle - 1
for i in range(Cycle):
    if Steps[i+1]-Steps[i] == 1:
        east()
    elif Steps[i+1]-Steps[i] == -1:
        west()
    elif Steps[i+1]-Steps[i] == 10:
        north()
    elif Steps[i+1]-Steps[i] == -10:
        south()
    robot.reset()
    i += 1
Moving = 1
Position = "North"
time_function(75)
watch.reset()
gyro_reset()
going_straight(305)
while True:
    if the_feeler.pressed() is True:
        for j in range(Cycle):
            if j != Final_Change:
                Straight_Distance = 477.5
                if Steps[j+1]-Steps[j] == 1:
                    east()
                elif Steps[j+1]-Steps[j] == -1:
                    west()
                elif Steps[j+1]-Steps[j] == 10:
                    north()
                elif Steps[j+1]-Steps[j] == -10:
                    south()
            elif j == Final_Change:
                Straight_Distance = 465
                if Steps[j+1]-Steps[j] == 1:
                    east()
                elif Steps[j+1]-Steps[j] == -1:
                    west()
                elif Steps[j+1]-Steps[j] == 10:
                    north()
                elif Steps[j+1]-Steps[j] == -10:
                    south()
            robot.reset()
            j += 1
        watch.pause()
        Clock_Time = str(watch.time())
        ev3.screen.draw_text(40,50,Clock_Time,text_color = Color.BLACK, background_color = None)
        wait(5000)
        ev3.screen.clear()



"""
41 42 43 44
31 32 33 34
21 22 23 24
11 12 13 14
"""
# This is like the framework for our logic. its very logical. We will start coding the backup algo if neccesary. 
# The steps list, that is where we input the coordinates the robot should go through. 11, 12, 22, 21 is not the actual path. 
# differences from one value to another are calculated and input into Moves list. Move is a starting variable for the loop
# here, we used step 1 to make the half step. But i think i should put it after the for loop
# We have another for loop, where the actual action begins. If it detects either 1, -1, 10, or -10, it will perform one of the 4 functions with those 4 conditionals based on chacking the list and the gyro angle (for the 4 conditions)
# The 4 conditionals matter here because robot position can vary (it can face 0, 90, 180, or 270 degrees based on the gyro) and if we don't use conditions, the robot will take these uneccesary moves and this will flag us a Stalling Penalty
# Once one cycle is done, it adds i and removes the move that is first on the list, so it will be different for each cycle. The program will stop when i = len of Moves
# Once we code speed, if everything goes as planned (Speed, Movement, Gates, Etc.), we should land on the target reaached at perfect time, covering 3 gates, giving us a score ranging 55-60 (Lowest Possible)
# Thats the array shown of the map that is integrated into the Robot's "Brain"
# I need to write speed formula. Its a matter of testing and recording data.
"""
The main functions should look something like this:

def blahblahblah():
    if angle is 0:
        it will do this
    elif 90:
        or that
    elif 180:
        or that
    elif 270:
        and that'
These are used for up, down, left, right.
I will finish this today or tomorrow after my bot finishes charging.



"""