#!/usr/bin/env python
import roslib
import rospy 
import math
import tf
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped


class QTtflistener:
   
    def __init__(self):
        self.head_publisher = rospy.Publisher("/qt_robot/head_position/command", Float64MultiArray, queue_size=5)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1)
        target_frame = "TargetFrame"
        source_frame = "NeckFrame"
        self.transformationFrame(source_frame, target_frame)
        self.rate.sleep()
        
    def orientationMatrix(self, point):
        yaw = math.atan2(point[1], point[0])
        pitch = math.atan2(point[2], point[0])
        yaw_deg = math.degrees(yaw)  
        pitch_deg = math.degrees(pitch)
        if yaw_deg > 90 and yaw_deg < 180:
            yaw_deg =  - (180 - yaw_deg)
        elif yaw_deg < 90:
            yaw_deg =  (180 +  yaw_deg)
        elif yaw_deg  == 0 or yaw_deg  == 180 or yaw_deg  == -180:
            yaw_deg = 0 
        else:
            yaw_deg = "Out of range"
        
        if pitch_deg > 90 and pitch_deg < 180:
            pitch_deg = 180 - pitch_deg
        elif pitch_deg < 90:
            pitch_deg = - (180 + pitch_deg)
        elif pitch_deg  == 0 or pitch_deg  == 180 or pitch_deg  == -180:
            pitch_deg = 0 
        else:
            pitch_deg = "Out of range"
        # Head commands for QT (HeadYaw, HeadPitch)
        return(pitch_deg, yaw_deg)

    def transformationPoint(self, target_frame, point):
        while not rospy.is_shutdown():
            try:
                point_target_frame = self.listener.transformPoint(target_frame, point)
                point_target_frame = point_target_frame.point
                self.orientationMatrix(point_target_frame)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        rate.sleep()
    
    def transformationFrame(self, source_frame, target_frame):
        while not rospy.is_shutdown():
            try:
                # we want to trasform from par[1] frame to par[0] frame
                # timing at which we want that transformation: this instant (time(0))
                (trans,rot) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                [pitch, yaw] = self.orientationMatrix(trans)
                self.head_command(pitch, yaw)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.rate.sleep()
          
          
    def head_command(self, pitch, yaw):
        head_commands = Float64MultiArray()
        head_commands.data = [yaw, pitch]
        print(str(head_commands.data))
        self.head_publisher.publish(head_commands)
                        

        


if __name__ == '__main__':
    rospy.init_node('qt_robot_tf', anonymous=True)
    QTtflistener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
