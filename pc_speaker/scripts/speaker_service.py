#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import pyaudio
import math
import audioop
import numpy as np
from collections import deque
from harmoni_common_lib.constants import State
from harmoni_common_lib.child import HardwareControlServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager
from audio_common_msgs.msg import AudioData

class SpeakerService(HarmoniExternalServiceManager):
    """
    Speaker service
    """

    def __init__(self, name, param):
        """ Initialization of variables and speaker parameters """
        rospy.loginfo("SpeakerService initializing")
        self.name = name
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.chunk_size = param["chunk_size"]
        """ Setup the speaker """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16  # How can we trasform it in a input parameter?
        self.stream = None
        """Setup the speaker service as server """
        self.state = State.INIT
        super().__init__(self.state)
        return

    def actuation_update(self, actuation_completed):
        """Update the actuation state """
        rospy.loginfo("Update speaker state")
        super().update(state = self.state, actuation_completed=actuation_completed)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def do(self, data):
        """ Do the speak"""
        self.state = State.REQUEST
        self.actuation_update(actuation_completed = False)
        data = super().do(data)
        try:
            rospy.loginfo("Writing data for speaker")
            self.stram.write(data)
            self.close_stream()
            self.state = State.RESPONSE
            self.actuation_update(actuation_completed = True)
        except:
            self.state = State.END
            self.actuation_update(actuation_completed = True)
        return

    def setup_speaker(self):
        """ Setup the speaker """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the audio output stream")
        self.stream = self.p.open(
            input = False,
            output = True,
            format=self.audio_format,
            channels=self.total_channels,
            rate=self.audio_rate,
            output_device_index=self.output_device_index,
            frames_per_buffer=self.chunk_size
        )
        return

    def close_stream(self):
        """Closing the stream """
        rospy.loginfo("Closing the audio output stream")
        self.stream.stop_stream()
        self.stream.close()
        return

    def get_index_device(self):
        """ 
        Find the output audio devices configured in ~/.asoundrc. 
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            if device["name"] == self.device_name:
                rospy.loginfo("Found device with name " + self.device_name)
                self.output_device_index = i
                return

def main():
    try:
        service_name = "speaker"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+service_name+"_param/")
        s = SpeakerService(service_name, param)
        hardware_control_server = HardwareControlServer(name=service_name, service_manager=s)
        hardware_control_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
