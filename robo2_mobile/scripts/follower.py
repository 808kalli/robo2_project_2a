#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt, asin
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
from pid import PID
# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])
def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)
        self.state = "Forward"
        self.turn_executed = False

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.v_x = rospy.Publisher("/velocity_Linear_x", Float64, queue_size=100)
        self.v_y = rospy.Publisher("/velocity_Linear_y", Float64, queue_size=100)
        self.omega_z = rospy.Publisher("/velocity_Angular_z", Float64, queue_size=100)
        self.yaw_angle = rospy.Publisher("/yaw", Float64, queue_size=100)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        # print(self.imu_yaw)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()

        pid_vel = PID(0.5,0.0,0.1)
        pid_rot = PID(8,0.0,0.0)

        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        while not rospy.is_shutdown():

            sonar_front = self.sonar_F.range
            sonar_front_left = self.sonar_FL.range
            sonar_left = self.sonar_L.range
            sonar_front_right = self.sonar_FR.range
            sonar_right = self.sonar_R.range

            #self.imu_callback(self.imu)
                
            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9
            # print("Time interval: ", dt)
            # print("=====================")
            
            # d = abs(sonar_front*sin(self.imu_yaw))
            side_A = sonar_front_left+sqrt(0.02)
            side_B = sonar_front+0.1
            side_C = sqrt(side_B**2+side_A**2-sqrt(2)*side_A*side_B)
            term = side_B*sin(pi/4)/side_C
            current_angle_to_wall_left = asin(term)
            side_A = sonar_front_right+sqrt(0.02)
            side_B = sonar_front+0.1
            side_C = sqrt(side_B**2+side_A**2-sqrt(2)*side_A*side_B)
            term = side_A*sin(pi/4)/side_C
            current_angle_to_wall_right = asin(term)
            
            if ((self.imu_yaw >= pi/2 and self.imu_yaw <= pi) or (self.imu_yaw >= -pi/2 and self.imu_yaw <= 0.0)):
                d = sonar_front*sin(current_angle_to_wall_left)
            else:
                d = sonar_front*sin(current_angle_to_wall_right)
            # print(np.degrees(current_angle_to_wall))

            if (dt == 0.0):
                pass
            else:
                # print(np.degrees(self.imu_yaw))
                if (self.state == "Forward"):
                    self.velocity.linear.x = pid_vel.PID_calc(0.0,0.4-d,dt)
                    if (self.turn_executed == True):
                        self.velocity.angular.z = -pid_rot.PID_calc(0.0, (sonar_left-sonar_front_left*cos(pi/4)), dt)                            
                    if ((abs(d - 0.4) < 0.1) and (abs(self.velocity.linear.x) < 0.01)):
                        self.turn_executed = False
                        self.velocity.linear.x = 0.0
                        self.state = "Turning"
                        if sonar_front_right < sonar_front:     #/_
                            corner_between_walls = current_angle_to_wall_right
                            yaw_target = self.imu_yaw - pi/2 - (pi/2-corner_between_walls)
                        else:                                   #_/
                            corner_between_walls = current_angle_to_wall_left + pi/4
                            yaw_target = self.imu_yaw - pi/2 - (pi/2-corner_between_walls)
                        yaw_target = normalize_angle(yaw_target)
                        print(np.degrees(yaw_target))
                elif (self.state == "Turning"):
                    self.velocity.angular.z = pid_rot.PID_calc(0.0, normalize_angle(yaw_target-self.imu_yaw), dt)
                    if (abs(self.velocity.linear.z) < 0.01 and abs(yaw_target - self.imu_yaw) < 0.001):
                        self.turn_executed = True
                        self.velocity.angular.z = 0.0
                        self.state = "Forward" 
                    
                    # self.velocity.angular.z = -pid_rot.PID_calc(turn_angle,self.imu_yaw,dt)
                    # if (abs(self.velocity.angular.z) < 0.001 and abs(pi/2 - current_angle_to_wall_left) < 0.001):
                    #     self.velocity.angular.z = 0.0
                    #     self.state = "Forward"
                    # print("d: ", abs(sonar_front*sin(self.imu_yaw)))
                    # print("velocity: ", self.velocity.linear.x)

            # Publish the new joint's angular positions
            self.v_x.publish(self.velocity.linear.x)
            self.v_y.publish(self.velocity.linear.y)
            self.omega_z.publish(self.velocity.angular.z)
            self.yaw_angle.publish(self.imu_yaw)

            self.velocity_pub.publish(self.velocity)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
