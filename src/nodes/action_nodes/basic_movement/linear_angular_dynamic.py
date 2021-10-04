#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist

from ...nodes import Action



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

