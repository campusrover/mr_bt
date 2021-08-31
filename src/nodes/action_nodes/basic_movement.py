#!/usr/bin/env python3

import sys
# sys.path.append("..") # Adds higher directory to python modules path.

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from ..nodes import Action



'''
Publishes a Twist message to the "cmd_vel" topic with a linear x velocity specified by the 
user at initialization of the node.
'''
class LinearStatic(Action):


    def __init__(self, vel):

        self.twist = Twist()

        self.twist.linear.x = vel

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        self.pub.publish(self.twist)

        blackboard['moving forward'] = True

        print('Moving forward')

        return "success"


'''
Publishes a Twist message with a linear x velocity which is accessed through the blackboard.
The user specifies the name of the blackboard variable (which is expected to be a float/int/double type).
'''
class LinearDynamic(Action):


    def __init__(self, var_name):

        self.var_name = var_name

        self.twist = Twist()

        self.pub = self.rospy.publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        try:

            self.twist.linear.x = blackboard[self.var_name]

            self.pub.publish(self.twist)

            return "success"

        except:

            return "failure"


class AngularStatic(Action):


    def __init__(self, vel):

        self.twist = Twist()

        self.twist.angular.z = vel

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        self.pub.publish(self.twist)

        print("Turning")

        return "success"


class AngularDynamic(Action):


    def __init__(self, var_name):

        self.var_name = var_name

        self.twist = Twist()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        try:

            self.twist.angular.z = blackboard[self.var_name]

            self.pub.publish(self.twist)
            print(self.twist)
            return "success"

        except:

            return "failure"


class LinearAngularStatic(Action):


    def __init__(self, lin_vel, ang_vel):

        self.twist = Twist()

        self.twist.linear.x = lin_vel

        self.twist.angular.z = ang_vel

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        self.pub.publish(self.twist)

        return "success"


class LinearAngularDynamic(Action):


    def __init__(self, linear_var_name, angular_var_name):

        self.var_name = [linear_var_name, angular_var_name]

        self.twist = Twist()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        try:

            if isinstance(self.var_name, list):

                lin_vel, ang_vel = blackboard[self.var_name[0]], blackboard[self.var_name[1]]

            else:

                lin_vel, ang_vel = blackboard[self.var_name][0], blackboard[self.var_name[1]]

            
            self.twist.linear.x = lin_vel
            self.twist.angular.z = ang_vel

            self.pub.publish(self.twist)

            return "success"

        except:

            return "failure"


'''
Publishes a cmd_vel topic to cease all linear and angular movement.
'''
class Stop(Action):


    def __init__(self):

        self.twist = Twist()

        self.twist.linear.x = 0
        self.twist.angular.z = 0

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    
    def tick(self, blackboard):

        self.pub.publish(self.twist)

        return "success"

