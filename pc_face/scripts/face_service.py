#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import math
import numpy as np
from collections import deque
from harmoni_common_lib.child import HardwareControlServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager

class Status():
    """ Status of the face service """
    INIT = 0 # init the service
    EXPRESSING = 1 # start express
    NOT_EXPRESSING = 2 # stop express
    END = 3  # terminate the service


class FaceService(HarmoniExternalServiceManager):
    """
    Face service
    """

    def __init__(self, name, param):
        """ Initialization of variables and face parameters """
        rospy.loginfo("FaceService initializing")
        self.name = name
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.chunk_size = param["chunk_size"]
        """ Setup the face """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16  # How can we trasform it in a input parameter?
        self.stream = None
        """Setup the face service as server """
        self.status = Status.INIT 
        super(FaceService, self).__init__(self.status)
        return

    def actuation_update(self, actuation_completed):
        """Update the actuation status """
        rospy.loginfo("Update face status")
        super(FaceService, self).update(status = self.status, actuation_completed=actuation_completed)
        return

    def test(self):
        super(FaceService, self).test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def do(self, data):
        """ Do the expression"""
        self.status = Status.SPEAKING
        self.actuation_update(actuation_completed = False)
        data = super(FaceService, self).do(data)
        try:
            rospy.loginfo("Writing data for face")
            self.stram.write(data)
            self.close_stream()
            self.status = Status.NOT_SPEAKING
            self.actuation_update(actuation_completed = True)
        except:
            self.status = Status.END
            self.actuation_update(actuation_completed = True)
        return

    def setup_face(self):
        """ Setup the face """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()
        return


def main():
    try:
        service_name = "face"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+service_name+"_param/")
        s = FaceService(service_name, param)
        hardware_control_server = HardwareControlServer(name=service_name, service_manager=s)
        hardware_control_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
