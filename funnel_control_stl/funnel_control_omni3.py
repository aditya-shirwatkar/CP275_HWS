#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import math
from math import pow, atan2, sqrt
import time
import numpy as np
# from phasespace_msgs.msg import Markers # Uncomment for Mocap on hardware

# Function to rotate a 2D vector by an angle theta in radians
def rotate_vector(x,y, theta):
    """Rotates a 2D vector by an angle theta in radians"""
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    rotated_x = x * cos_theta - y * sin_theta
    rotated_y = x * sin_theta + y * cos_theta
    return (rotated_x, rotated_y)

# STL class definition
class STL:
    def __init__(self, x, y, t_range, tolerance):
        self.x = x
        self.y = y
        self.des_config = np.array([x, y])
        self.t_range = t_range
        self.tolerance = tolerance

# Function defining the switching function
class STL_Controller:
    def __init__(self, stl, init_config, stations):
        self.stl = stl
        self.swf_const = 10
        self.init_config = init_config
        self.stations = stations
        self.sz_stl = len(self.stl)
        self.beta = np.zeros(self.sz_stl)
        self.funnel_rate = 0.5
        self.gamma_init_scale = 2
        self.start_time_s = 0 #rospy.Time.now().to_sec()
        self.velocity_gain = 25

    def switching_func(self, current_time):
        delta = 0.1
        # for i in range(self.sz_stl):
            # a = 1/(1 + np.exp(-self.swf_const*((current_time - self.start_time_s) - (self.stl[i].t_range[0] + delta)))) # type: ignore
            # b = 1/(1 + np.exp(-self.swf_const*((current_time - self.start_time_s) - (self.stl[i].t_range[1] + delta)))) # type: ignore
            # self.beta[i] =  a - b

        if ((current_time - self.start_time_s) < self.stl[0].t_range[1]):
            self.beta[0] = 1
        else:
            self.beta[0] = 0
        
        if ((current_time - self.start_time_s) < self.stl[1].t_range[1]) and ((current_time - self.start_time_s) > self.stl[0].t_range[1]):
            self.beta[1] = 1
        else:
            self.beta[1] = 0

        return self.beta

    def gamma_h_calc(self, current_time):
        sz_stl = len(self.stl)

        gamma_h_0 = np.zeros(sz_stl)
        gamma_h_inf = np.zeros(sz_stl)
        gamma_h_vec = np.zeros(sz_stl)

        l = self.funnel_rate
        gamma_i_sc = self.gamma_init_scale

        for i in range(sz_stl):
            gamma_h_0[i] = gamma_i_sc * (np.linalg.norm(self.stations[i] - self.stl[i].des_config))
            gamma_h_inf[i] = self.stl[i].tolerance
            gamma_h_vec[i] = (gamma_h_0[i] - gamma_h_inf[i]) * np.exp(-l*((current_time - self.start_time_s)-self.stl[i].t_range[0])) + gamma_h_inf[i] # type: ignore

        gamma_h = np.diag(gamma_h_vec)
        gamma_h_inv = np.diag(1/gamma_h_vec)

        return gamma_h, gamma_h_inv
    
    def get_h(self, stl_task, robot_state):
        h = np.zeros(len(stl_task))
        for i in range(len(stl_task)):
            # Errors
            h[i] = sqrt(pow(stl_task[i].x - robot_state[0], 2) + 
                          pow(stl_task[i].y - robot_state[1], 2))
        return h

    def get_h_bar(self, stl_task, robot_state, current_time):
        h = self.get_h(stl_task, robot_state)
        # dot product of the error and the switching function
        beta = self.switching_func(current_time)
        # elementwise multiplication of the switching function and the error to get a vector
        h_bar = beta * h
        return h_bar
    
    def norm_error(self, stl_task, robot_state, current_time):
        h_bar = self.get_h_bar(stl_task, robot_state, current_time)
        gamma_h, gamma_h_inv = self.gamma_h_calc(current_time)
        # matrix to vector multiplication of the inverse of the gamma_h matrix and the h_bar vector
        norm_error = gamma_h_inv @ h_bar
        # Keep norm error between -1 and 1
        for i in range(len(stl_task)):
            if norm_error[i] > 1:
                norm_error[i] = 0.999
            elif norm_error[i] < -1:
                norm_error[i] = -0.999
        return norm_error
    
    def transformed_error(self, stl_task, robot_state, current_time):
        norm_error = self.norm_error(stl_task, robot_state, current_time)
        transformed_error = np.zeros(len(stl_task))
        for i in range(len(stl_task)):
            transformed_error[i] = math.log((1 + norm_error[i])/(1-norm_error[i]))
        return transformed_error
    
    def transformed_derror(self, stl_task, robot_state, current_time):
        norm_error = self.norm_error(stl_task, robot_state, current_time)
        transformed_derror = np.zeros(len(stl_task))
        for i in range(len(stl_task)):
            transformed_derror[i] = 2/(1 - pow((norm_error[i]), 2))
        return transformed_derror
    
    def dh_bar_d_x_o_calc(self, robot_state, h, stl,current_time):
        sz_stl = len(stl)
        beta = self.switching_func(current_time)
        dh_bar_dx_o = np.zeros((sz_stl, 2))

        for i in range(sz_stl):
            for j in range(2):
                dh_bar_dx_o[i][j] = (beta[i] * (robot_state[j] - stl[i].des_config[j])) / h[i]

        return dh_bar_dx_o

    def get_velocity_input(self,stl_task, robot_state, current_time):
        # mutliply the transformed error and the transformed derivative of the error elemetwise
        # and then sum them up
        velocity_input = np.zeros(len(robot_state))

        # calculate the transformed error and the transformed derivative of the error
        transformed_error = self.transformed_error(stl_task, robot_state, current_time)
        transformed_derror = self.transformed_derror(stl_task, robot_state, current_time)

        # calculate gamma h inv
        gamma_h, gamma_h_inv = self.gamma_h_calc(current_time)

        # calculate the dh/dx_o matrix
        h = self.get_h(stl_task, robot_state)
        dh_bar_dx_o = self.dh_bar_d_x_o_calc(robot_state, h, stl_task, current_time)

        # transpose the dh/dx_o matrix
        dh_bar_dx_o_t = np.transpose(dh_bar_dx_o)

        # calculate the velocity input
        error_mult = transformed_error * transformed_derror
        error_mult2 = gamma_h_inv @ error_mult
        error_mult3 = dh_bar_dx_o_t @ error_mult2
        velocity_input = -1*self.velocity_gain*error_mult3
        return velocity_input

# Class definition for the omni robot 
class Omni_robot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('funnel_control_stl', anonymous=True)

        # Publisher which will publish wheel velocity.
        self.publ = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
        self.pubb = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
        self.pubr = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        # If required uncomment the below line
        # self.velocity_publisher = rospy.Publisher('/cmd_vel',
        #                                           Twist, queue_size=10)

        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type Pose is received.
        self.sub_x = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        # For Mocap system on Hardware
        # # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # # when a message of type Pose is received.
        # self.pose_subscriber = rospy.Subscriber('/phasespace/markers', Markers, self.update_pose)

        self.L = 0.04
        self.wheel_radius = 0.2
        self.MAX_LIN_SPEED = 10.0
        self.MAX_ANG_SPEED = 3.5
        self.x = float()
        self.y = float()
        self.theta = float()
        self.vx = float()
        self.vy = float()
        self.rate = rospy.Rate(100)
    
    def update_pose (self, data):
        """Callback function which is called when a new message of type X is
        received by the subscriber."""
        # Find the index of "my_robot" in the model_states message
        index = data.name.index("open_base")

        # Get the state of "my_robot"
        pose = data.pose[index]
        twist = data.twist[index]

        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = pose.orientation.z
        self.vx = twist.linear.x
        self.vy = twist.linear.y
	
        # For Mocap system on Hardware
        # Update from markers
        # # Put the led number as per the position on the robot.
        # # dyn-> led on the dynamics point of the robot
        # # ori-> led on the center of the robot
        # marker_dyn = data.markers[1]
        # marker_ori = data.markers[0]

        # self.x = round(marker_dyn.x, 4)
        # self.y = round(marker_dyn.y, 4)
        
        # dy = marker_dyn.y - marker_ori.y
        # dx = marker_dyn.x - marker_ori.x
        
        # self.theta = round(atan2(dy, dx), 4)

        # # print(self.pose.x, self.pose.y, self.pose.theta)

    def euclidean_distance(self, goal_x, goal_y):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_x - self.x), 2) +
                    pow((goal_y - self.y), 2))
    
    def move2goal(self,start_time_s):
        # Initial Config
        init_config = np.array([self.x, self.y])

        # Goal 1 X and Y ranges
        Goal1_x_range = [0.60, 0.645]
        Goal1_y_range = [0.15, 0.20]
        time_span_1 = [10, 25]
        Goal1_x = sum(Goal1_x_range)/2 # (3.00+3.45)/2
        Goal1_y = sum(Goal1_y_range)/2 # (1.5+2.0)/2


        # Goal 2 X and Y ranges
        Goal2_x_range = [0.10, 0.145]
        Goal2_y_range = [0.80, 0.85]
        time_span_2 = [35, 50]
        Goal2_x = sum(Goal2_x_range)/2 # (1.00+1.45)/2
        Goal2_y = sum(Goal2_y_range)/2 # (2.0+2.5)/2

        # Radius Biggest circle inscribing Goal 1 and Goal 2
        r1 = sqrt(pow(Goal1_x - Goal1_x_range[0], 2) + pow(Goal1_y - Goal1_y_range[0], 2))
        r2 = sqrt(pow(Goal2_x - Goal2_x_range[0], 2) + pow(Goal2_y - Goal2_y_range[0], 2))
        r_tol = 0.1 # tolerance in radius
        r1 = r1 + r_tol
        r2 = r2 + r_tol

        # point of intersection of the line joining [init_config[0], init_config[1]] and [Goal1_x, Goal1_y]
        # and the circle of radius r1 centered at [Goal1_x, Goal1_y]
        station_x1 = Goal1_x - (Goal1_x - init_config[0]) * r1 / sqrt(pow((Goal1_x - init_config[0]), 2) + pow((Goal1_y - init_config[1]), 2))
        station_y1 = Goal1_y - (Goal1_y - init_config[1]) * r1 / sqrt(pow((Goal1_x - init_config[0]), 2) + pow((Goal1_y - init_config[1]), 2))
        
        # same for Goal 2
        station_x2 = Goal2_x - (Goal2_x - Goal1_x) * r2 / sqrt(pow((Goal2_x - Goal1_x), 2) + pow((Goal2_y - Goal1_y), 2))
        station_y2 = Goal2_y - (Goal2_y - Goal1_y) * r2 / sqrt(pow((Goal2_x - Goal1_x), 2) + pow((Goal2_y - Goal1_y), 2))

        # # Stationary point
        station1  = np.array([station_x1, station_y1])
        station2  = np.array([station_x2, station_y2])
        stations = [station1, station2]

        # Defining the STL goal
        stl_task = []
        stl_task_1 = STL(Goal1_x,Goal1_y,time_span_1,0.02)
        stl_task_2 = STL(Goal2_x,Goal2_y,time_span_2,0.02)

        stl_task.append(stl_task_1)
        stl_task.append(stl_task_2)

        sz_stl = len(stl_task)
        
        # Defining the STL controller
        stl_controller = STL_Controller(stl_task, init_config, stations)
        

        # Defining the safety controller
        max_lin_speed_record = 0
        max_lin_speed_record_actual = 0
        current_time = start_time_s

        # time.sleep(1)

        # while (rospy.Time.now().to_sec() - start_time_s) <= 55:
        while (current_time <= 55) or (rospy.is_shutdown() == False):
            # get the current state of the robot
            robot_state = np.array([self.x, self.y])

            # current time
            # current_time = rospy.Time.now().to_sec() - start_time_s
            current_time = 0.01 + current_time

            # get the velocity input from the STL controller and PID controller
            # STL controller
            velocity_input = stl_controller.get_velocity_input(stl_task, robot_state, current_time) 

            # Change the names of the velocity input
            vel_x_wf = velocity_input[0]
            vel_y_wf = velocity_input[1]
            angular_z = 0

            # # Rotate the velocity vector from world frame to robot frame
            vel_x, vel_y = rotate_vector(vel_x_wf, vel_y_wf, -self.theta)
            # vel_x = vel_x_wf
            # vel_y = vel_y_wf

            # Safety controller
            # # clip velocities above a threshold
            linear_speed = sqrt(pow(vel_x,2) + pow(vel_y,2))
            linear_speed_actual = sqrt(pow(self.vx,2) + pow(self.vy,2))
            
            if linear_speed > self.MAX_LIN_SPEED:
                alpha = self.MAX_LIN_SPEED / linear_speed
                vel_x = vel_x * alpha
                vel_y = vel_y * alpha
            angular_z = max(min(angular_z, self.MAX_ANG_SPEED), -self.MAX_ANG_SPEED)
            
            # print current time and output velocities
            print("------------------")
            print("Current time: ", current_time)
            print("x: ", self.x)
            print("y: ", self.y)
            print("theta: ", self.theta)
            print("vel_x: ", vel_x)
            print("vel_y: ", vel_y)
            # print("angular_z: ", angular_z)

            # max_lin_speed_record 
            if linear_speed > max_lin_speed_record:
                max_lin_speed_record = linear_speed

            if linear_speed_actual > max_lin_speed_record_actual:
                max_lin_speed_record_actual = linear_speed_actual

            # twist = Twist()
            # twist.linear.x = vel_x
            # twist.linear.y = vel_y
            # twist.angular.z = angular_z

            # self.velocity_publisher.publish(twist)

            # Inverse kinematics to get the wheel velocities
            vell = -vel_x/2.0 - (sqrt(3.0)/2.0)*vel_y + self.L*angular_z
            velb = vel_x + self.L*angular_z
            velr = -vel_x/2.0 + (sqrt(3.0)/2.0)*vel_y + self.L*angular_z

            # print current time and output velocities
            # print("Current time: ", current_time)
            # print("vell: ", vell)
            # print("velb: ", velb)
            # print("velr: ", velr)

            # Publishing our velocities
            self.publ.publish(vell)
            self.pubb.publish(velb)
            self.pubr.publish(velr)

            # Publish at the desired rate.
            self.rate.sleep()
        
        # Stopping our robot after the movement is over.
        vell = 0.0
        velb = 0.0
        velr = 0.0
        self.publ.publish(vell)
        self.pubb.publish(velb)
        self.pubr.publish(velr)

        # twist = Twist()
        # twist.linear.x = 0
        # twist.linear.y = 0
        # twist.angular.z = 0
        # self.velocity_publisher.publish(twist)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    try:
        Omni = Omni_robot()
        # start_time_s = rospy.Time.now().to_sec()
        time.sleep(1)
        start_time_s = 0
        Omni.move2goal(start_time_s)
  
    except Exception as e:
        print(e)

    finally:
        vell = Float64()
        velb = Float64()
        velr = Float64()
	
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
