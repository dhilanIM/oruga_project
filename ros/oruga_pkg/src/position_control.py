#!/usr/bin/env python 
# coding=utf-8

from math import pi,sin,cos,pow, sqrt,atan2
#from time import thread_time
import rospy
from geometry_msgs.msg import Twist,PoseStamped, Pose2D #Twist for publish Velocities    Pose2D to get position x,y,theta
from nav_msgs.msg import Odometry
from rospy.core import is_shutdown
from tf.transformations import euler_from_quaternion, quaternion_from_euler

L = 0.24 #Distance in meters from the center of the wheel to the center o the robot
R = 0.055 #Radio of the wheel

x_d = 1.0; #Desired x position
y_d = 0.00;  #Desired y position

D = 0.05

vel_msg = Twist()

#t = 0.0; t0 = 0.0; #Timer, initial time
#T = 100.0;  #Trajectory period, 
kx = 0.2 #controller gains kx = ky = k
ky = 0.5
v_max = 0.15; w_max = 2.5; # maximum velocities

#ex = 0.0; ey = 0.0; V = 0.0; W = 0.0 #Errors and velocities initializations
x_i = 0; y_i = 0; theta_i = 0

def poseStamped_callback(msg):
    pass
    """
    global x_i
    global y_i
    global theta_i
    #x_i = msg.x
    #y_i = msg.y
    
    x_i = msg.pose.position.x
    y_i = msg.pose.position.y

    quaternion = msg.pose.orientation
    quats = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]

    (roll,pitch,theta_i) = euler_from_quaternion(quats)
    
    quaternion = msg.pose.orientation
    quats = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]

    (roll,pitch,theta_i) = euler_from_quaternion(quats)
    """
    
def pose2D_callback(msg):
    #pass

    global x_i
    global y_i
    global theta_i

    x_i = msg.x
    y_i = msg.y
    theta_i = msg.theta

def odom_callback(msg):
    pass
    """
    global x_i
    global y_i
    global theta_i

    
    x_i = msg.pose.pose.position.x
    y_i = msg.pose.pose.position.y

    quaternion = msg.pose.pose.orientation
    quats = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]

    (roll,pitch,theta_i) = euler_from_quaternion(quats)
    """
    
    


"""
This node is going to publish  velocities trough Twist 
and is going to suscribe to Odometry
"""
if __name__=="__main__":
    try:
        rospy.init_node('position_control_node',anonymous=False)
        rate = rospy.Rate(50)

        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Publisher
        rospy.Subscriber('/pose2D',Pose2D,pose2D_callback)   #Suscriber
        #rospy.Subscriber('/odom',Odometry,odom_callback)   #Suscriber

        x_error = 99
        y_error = 99

        while(1):
            v_error = sqrt(pow(x_error,2) + pow(y_error,2)) #Lineal velocity error
            if(v_error>0.05):
                x_p = x_i + D*cos(theta_i)
                y_p = y_i + D*sin(theta_i)

                ex = kx*(x_d-x_p)
                ey = ky*(y_d-y_p)
                #Acción de control
                wr=ex*(2*D*cos(theta_i) - L*sin(theta_i))/(2*D*R) + ey*(L*cos(theta_i) + 2*D*sin(theta_i))/(2*D*R)
                wl=ex*(2*D*cos(theta_i) + L*sin(theta_i))/(2*D*R) - ey*(L*cos(theta_i) - 2*D*sin(theta_i))/(2*D*R)
                v = R*(wl+wr)/2
                w = R*(wr-wl)/L
            else:
                v = 0
                w = 0

            
            #wheel_l = (2*v - w*L)/(2*R)
            #wheel_r = (2*v + w*L)/(2*R)
            
            # Velocities Saturation 
            #if (abs(v)>v_max):
            #    v = v_max*abs(v)/v
            #if (abs(w)>w_max):
            #    w = w_max*abs(w)/w

            vel_msg.linear.x = v
            vel_msg.angular.z = w
            #vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
            vel_pub.publish(vel_msg)
            
            x_error = x_d - x_i
            y_error = y_d - y_i

            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        vel_pub.publish(vel_msg)
        rospy.loginfo("Goal Reached!!")
            
    except rospy.ROSInterruptException:
        pass
