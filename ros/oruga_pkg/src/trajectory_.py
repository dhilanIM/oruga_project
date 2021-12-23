#!/usr/bin/env python 

from math import pi,sin,cos,pow, sqrt,atan2
#from time import thread_time
import rospy
from geometry_msgs.msg import Twist,PoseStamped, Pose2D #Twist for publish Velocities    Pose2D to get position x,y,theta
from nav_msgs.msg import Odometry
from rospy.core import is_shutdown
from tf.transformations import euler_from_quaternion, quaternion_from_euler

L = 0.24 #Distance in meters from the center of the wheel to the center o the robot
R = 0.055 #Radio of the wheel
D = 0.05 # Offset

x_d = 0.0; #Desired x position
y_d = 0.00;  #Desired y position

vel_msg = Twist()

#t = 0.0; t0 = 0.0; #Timer, initial time
#T = 100.0;  #Trajectory period, 
k1 = 0.01 #controller gains kx = ky = k
k2 = 0.01
v_max = 0.15; w_max = 2.5; # maximum velocities

#ex = 0.0; ey = 0.0; V = 0.0; W = 0.0 #Errors and velocities initializations
x_i = 0; y_i = 0; theta_i = 0

def poseStamped_callback(msg):
    pass

    
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


if __name__=="__main__":
    try:
        rospy.init_node('trajectory_node',anonymous=False)
        rate = rospy.Rate(50)

        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Publisher
        rospy.Subscriber('/pose2D',Pose2D,pose2D_callback)   #Suscriber

        x_error = x_d - x_i
        y_error = y_d - y_i

        while(1):
            t = rospy.Time.now().to_sec()
            #Seno 
            T = 2.0
            A = 0.5
            x_d = 0.1*t
            y_d = A*sin(2*pi*(1/T)*x_d)

            x_p = x_i + D* cos(theta_i)
            y_p = y_i + D* sin(theta_i)
            ex = k1 *(x_d - x_p)
            ey = k2 *(y_d - y_p)
            wr = ex*(2*D*cos(theta_i) - L*sin(theta_i))/(2*D*R) + ey*(L*cos(theta_i) + 2*D*sin(theta_i))/(2*D*R)
            wl = ex*(2*D*cos(theta_i) + L*sin(theta_i))/(2*D*R) - ey*(L*cos(theta_i) - 2*D*sin(theta_i))/(2*D*R)

            v_error = sqrt(pow(x_error,2) + pow(y_error,2)) #Lineal velocity error
            if(v_error>0.05):
                v = R*(wl+wr)/2
                w = R*(wr-wl)/L
            else:
                v = 0
                w = 0
            
            vel_msg.linear.x = v
            vel_msg.angular.z = w
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

