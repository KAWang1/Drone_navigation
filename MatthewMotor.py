import math
from .PCA9685 import PCA9685
from .ADC import *
import time
import math
import sys

# Ensure the script can access ROS packages
ros_package_path = '/opt/ros/humble/local/lib/python3.10/dist-packages'
ros_package_path2 = '/opt/ros/humble/local/lib/python3.10/site-packages'
freenove_package_path = ''
if ros_package_path not in sys.path:
    sys.path.append(ros_package_path)

if ros_package_path2 not in sys.path:
    sys.path.append(ros_package_path2)

if freenove_package_path not in sys.path:
    sys.path.append(freenove_package_path)


# Now your ROS imports should work
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    """
    #x = self.latest_pose.pose.orientation.x
    #y = self.latest_pose.pose.orientation.y
    #z = self.latest_pose.pose.orientation.z
    #w = self.latest_pose.pose.orientation.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

   # print(yaw)
    return yaw  # return only yaw angle (in radians)


def calculate_turn_angle(current_position, current_angle, destination):
    # Calculate direction vector to destination
    direction = (destination[0] - current_position[0], destination[1] - current_position[1])

    # Calculate desired orientation
    desired_orientation = ((math.atan2(direction[1], direction[0])) - (math.pi/2))

    # Calculate turn angle to face the correct orientation
    turn_angle = desired_orientation - current_angle

    # Normalize turn angle to be within the range of -π to +π
    turn_angle = (turn_angle + math.pi) % (2 * math.pi) - math.pi
    return turn_angle


class CarControl(Node):
    def __init__(self):
        super().__init__('car_control')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/AimsCar/pose',
            self.pose_callback,
            10)

        self.latest_pose = None

        self.starting_pose = None

        self.PWM = Motor()

        self.movement_start_time = None  # Track when movement starts

        self.stop_indefinitely = False  # Flag to indicate stopping indefinitely


        #self.timer = self.create_timer(0.02, self.pose_callback)  # 20ms timer


    def pose_callback(self, msg):
        # If stopped indefinitely, simply return without doing anything
        if self.stop_indefinitely:
            return

        # Initialize starting_pose if not already set
        if self.starting_pose is None:
            self.starting_pose = msg

        self.latest_pose = msg
        curr_pos_test = (self.latest_pose.pose.position.x, self.latest_pose.pose.position.y)
        dest_pos_test = (-1.5, -2.4)
        current_angle = quaternion_to_euler(self.latest_pose.pose.orientation.x, self.latest_pose.pose.orientation.y, self.latest_pose.pose.orientation.z, self.latest_pose.pose.orientation.w)
        print("Curr angle", current_angle)
        print("turn angle", calculate_turn_angle(curr_pos_test, current_angle, dest_pos_test))
        # print(quaternion_to_euler())
        # print(quaternion_to_euler(self.latest_pose.pose.orientation.x, self.latest_pose.pose.orientation.y, self.latest_pose.pose.orientation.z, self.latest_pose.pose.orientation.w))
        # Calculate the condition to move forward
        should_move_forward = self.latest_pose.pose.position.x < self.starting_pose.pose.position.x + 1
        # print(self.latest_pose.pose.orientation.x, self.latest_pose.pose.orientation.y, self.latest_pose.pose.orientation.z, self.latest_pose.pose.orientation.w)
        # Check if the car is already moving by seeing if movement_start_time is set
        if self.movement_start_time is not None:
            # Calculate elapsed time since the car started moving
            elapsed_time = time.time() - self.movement_start_time
            # If more than 5 seconds have passed, stop the car indefinitely
            if elapsed_time > 1000:
                self.PWM.setMotorModel(0, 0, 0, 0)  # Stop
                print("Stopping indefinitely due to exceeding 5 seconds of movement")
                self.stop_indefinitely = True  # Set the flag to stop indefinitely
                self.movement_start_time = None  # Reset the timer
            elif should_move_forward:
                # Car should continue moving forward, nothing to do here
                pass
            else:
                # If the car should not move forward and hasn't exceeded time, stop it.
                self.PWM.setMotorModel(0, 0, 0, 0)  # Stop
                self.movement_start_time = None
                print("Stop")
        elif should_move_forward:
            # If the car should move forward and isn't already, start moving and record the start time
            # self.PWM.setMotorModel(2000, 2000, 2000, 2000)  # Forward
            self.movement_start_time = time.time()  # Record the time when movement starts
            print("Forward")


class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.time_proportion = 2.5  # Depend on your own car,If you want to get the best out of the rotation mode,
        # change the value by experimenting.
        self.adc = Adc()


    @staticmethod
    def duty_range(duty1, duty2, duty3, duty4):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095

        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095

        if duty3 > 4095:
            duty3 = 4095
        elif duty3 < -4095:
            duty3 = -4095

        if duty4 > 4095:
            duty4 = 4095
        elif duty4 < -4095:
            duty4 = -4095
        return duty1, duty2, duty3, duty4

    def left_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(0, 0)
            self.pwm.setMotorPwm(1, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(1, 0)
            self.pwm.setMotorPwm(0, abs(duty))
        else:
            self.pwm.setMotorPwm(0, 4095)
            self.pwm.setMotorPwm(1, 4095)

    def left_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(3, 0)
            self.pwm.setMotorPwm(2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(2, 0)
            self.pwm.setMotorPwm(3, abs(duty))
        else:
            self.pwm.setMotorPwm(2, 4095)
            self.pwm.setMotorPwm(3, 4095)

    def right_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(6, 0)
            self.pwm.setMotorPwm(7, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(7, 0)
            self.pwm.setMotorPwm(6, abs(duty))
        else:
            self.pwm.setMotorPwm(6, 4095)
            self.pwm.setMotorPwm(7, 4095)

    def right_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(4, 0)
            self.pwm.setMotorPwm(5, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(5, 0)
            self.pwm.setMotorPwm(4, abs(duty))
        else:
            self.pwm.setMotorPwm(4, 4095)
            self.pwm.setMotorPwm(5, 4095)

    def setMotorModel(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)

    def Rotate(self, n):
        angle = n
        bat_compensate = 7.5 / (self.adc.recvADC(2) * 3)
        while True:
            W = 2000

            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))

            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W

            PWM.setMotorModel(FL, BL, FR, BR)
            print("rotating")
            time.sleep(5 * self.time_proportion * bat_compensate / 1000)
            angle -= 5



#rclpy.init()  # Initialize the ROS client library
#PWM = Motor()
#Odometry = CarControl()
#rclpy.spin(Odometry)

def loop():
    print("Starting LoopdeLoop")

    print("Starting rclpy")
    rclpy.init()  # Initialize the ROS client library
    print("Starting Odom")
    Odometry = CarControl()
    print("Starting Odom Spin")
    rclpy.spin(Odometry)
    #print("Starting Moving")

    #while Odometry.latest_pose is None:
    #    time.sleep(.5)

#    while Odometry.latest_pose.x < Odometry.starting_pose.x + 1:
#        PWM.setMotorModel(2000, 2000, 2000, 2000)  # Forward
    #time.sleep(3)
    #PWM.setMotorModel(-2000, -2000, -2000, -2000)  # Back
    #time.sleep(3)
    #PWM.setMotorModel(-500, -500, 2000, 2000)  # Left
    #time.sleep(3)
    #PWM.setMotorModel(2000, 2000, -500, -500)  # Right
    #time.sleep(3)
    #PWM.setMotorModel(0, 0, 0, 0)  # Stop


def destroy():
    PWM.setMotorModel(0, 0, 0, 0)


if __name__ == '__main__':
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
    finally:
        Odometry.destroy_node()
        rclpy.shutdown()

