#!/usr/bin/env  python
import numpy as np
import rospy
import tf
import math
from sensor_msgs.msg import JointState

class DegreeToRadians:
    def __init__(self):
        rospy.init_node("radians_joint_node", anonymous = True)
        rospy.Subscriber("qt_robot/joints/state", JointState, self.handle_degree)
        self.joint_pub = rospy.Publisher("qt_robot/joints/state_rad", JointState, queue_size = 1)
        
    def handle_degree(self, data):
        degrees = data.position
        radians = [math.radians(d) for d in degrees]
        joint_rad = JointState()
        joint_rad.position = radians
        joint_rad.name = data.name
        joint_rad.velocity = data.velocity
        joint_rad.effort = data.effort
        joint_rad.header = data.header
        self.joint_pub.publish(joint_rad)

if __name__ == '__main__':
    try:
        DegreeToRadians()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
