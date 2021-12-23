#! /usr/bin/env python
# encoding: utf-8

import rospy
import actionlib
from pos_control.msg import DoPositionControlAction,DoPositionControlGoal
#from math import pi

# Imprime el feedback recibido desde el servidor
def feedback_callback(msg):
    print('[FEEDBACK] Position error -> ', str(msg))

# LLama al servidor
def call_server():
    client = actionlib.SimpleActionClient('do_position_control', DoPositionControlAction)
    client.wait_for_server()

    goal = DoPositionControlGoal()
    goal.p_d = [1.5,1.5] #[x,y]

    client.send_goal(goal,feedback_cb=feedback_callback)
    client.wait_for_result()

    result = client.get_result()
    
    return result


if __name__ == "__main__":
    try:
        rospy.init_node('action_client_node')
        result = call_server()
        print('The position (x,y) is: ',result)
    except rospy.ROSInterruptException:
        pass

