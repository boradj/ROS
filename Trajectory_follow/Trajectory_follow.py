'''
we are five person in group:
Jaydip Borad
Shresth sharma
Dharmesh patel
milind milind
Aayush Surana

to execute this file we have to install sympy by using pip install sympy
'''
import rospy
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import tf.transformations as tftr
from traj_generator import oval, gener_traj

from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
class Task1:

    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)

        self.dt = 0.0
        self.time_start = 0.0
        self.end = False
        self.t = 0.0
        self.t_ = 0.0
        self.dist2goal = 0.0

        self.T = rospy.get_param('~T', 1)
        self.oval = oval(self.T)
        
        self.pose_init = [0.0, 0.0, 0.0]
        self.flag = True
        self.toggle = True

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 2.0])
             
        # in 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        print(self.pose_des.T)

        self.rot_z_des = 0.0

        "ROS stuff"
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        #self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)


    def transform_pose(self, pose_w):
        # in 'body' frame
        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]
            
    def odometry_callback(self, msg):
        self.lock.acquire()
        # read current robot state
        self.cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        cur_rot_z = cur_rpy[2]
        self.cur_rot = cur_rot_z

        if self.flag:
            self.zero_pose = [self.cur_position.x, self.cur_position.y, self.cur_position.z]
            self.flag = False


        '''CALCULATE ERRORS HERE AND DISTANCE TO GOAL'''
        e_x = self.pose_des[0] - self.cur_position.x
        e_y = self.pose_des[1] - self.cur_position.y
        e_z = self.pose_des[2] - cur_rot_z
        
        self.dist2goal = sqrt((e_x**2) + (e_y**2))
        error_angle = -arctan2(e_y, e_x) + cur_rot_z
        if error_angle > np.pi:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z - 2*np.pi
        elif error_angle < -np.pi:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z + 2*np.pi
        else:
        	error_angle = -arctan2(e_y, e_x) + cur_rot_z
        
        # set control
        velocity = Twist()
        self.Kp_Linear = 0.15*self.dist2goal*cos(error_angle)
        self.Kp_Angular = -0.8*error_angle
        self.Ki_Linear = 0.20*self.dist2goal*cos(error_angle)*(self.dt)
        self.Ki_Angular = 0.5*error_angle*(self.dt)
        velocity.linear.x = self.Kp_Linear + self.Ki_Linear
        velocity.angular.z = self.Kp_Angular + self.Ki_Angular
        self.pub_cmd_vel.publish(velocity)
        self.dist2goal_prev = self.dist2goal
        self.prev_error_angle = error_angle
        print('ERROR:' , e_x, e_y)
        print('ORIENTATION:' ,error_angle)

        self.lock.release()
        
    def map_plot(self, x1, y1, X, Y):
        self.ax[0].cla()
        self.ax[1].scatter(x1, y1, s = 5, c = [0, 1, 0], zorder=0)
        self.ax[0].scatter(X, Y, s = 5, c = [0, 1, 0], zorder=0)
        l = self.cur_rot*180/np.pi        
        m = MarkerStyle(">")
        m._transform.rotate_deg(l)
        self.ax[0].scatter(x1, y1, marker=m, s=100, color="crimson")
        self.fig.canvas.draw_idle()
        plt.pause(0.000001)

    def spin(self):
        rospy.loginfo('Task started!')
        X = []
        Y = []
        rate = rospy.Rate(self.RATE)
        self.t = (self.t_*np.pi)/180
        x, y, v, omega = self.oval.velocity(time_step=self.t)
        x = np.array(x, dtype=np.float64) 
        y = np.array(y, dtype=np.float64)

        time_step = 5.0
        self.end = False

        time_prev = 0.0
        self.time_start = rospy.get_time()
        self.pose_des = self.transform_pose([x, y, 0.0])
        plt.ion()
        self.fig, self.ax = plt.subplots(2)
        self.fig.suptitle('Trajectory Scan')
        self.ax[0].set_xlim(-8.5, 0.5)
        self.ax[0].set_ylim(-3, 3)
        self.ax[1].set_xlim(-8.5, 0.5)
        self.ax[1].set_ylim(-2, 2)
        
        for i in range(0, 360):
            p = (i*np.pi)/180
            x, y, v, omega = self.oval.velocity(time_step=p)
            X.append(x)
            Y.append(y)                

        while not rospy.is_shutdown():
            t = rospy.get_time() - self.time_start
            self.dt = t - time_prev
            time_prev = t
            if p == t:
                self.toggle = True
            if t > 1:
                self.t_ = self.t_ + 1
                self.t = (self.t_*np.pi)/180
                x, y, v, omega = self.oval.velocity(time_step=self.t)
                x = np.array(x, dtype=np.float64)
                y = np.array(y, dtype=np.float64)
                self.toggle = False          	
            self.pose_des = self.transform_pose([x, y, 0.0])
            rate.sleep()
            time.sleep(0.8)
            self.map_plot(self.cur_position.x, self.cur_position.y, X, Y)
            
        rospy.loginfo('Task completed!')


if __name__ == "__main__":
    rospy.init_node('task1_node')
    task1 = Task1()
    task1.spin()
