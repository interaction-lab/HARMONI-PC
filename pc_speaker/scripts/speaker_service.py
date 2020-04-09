#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import pyaudio
import math
import audioop
import numpy as np
from collections import deque
from harmoni_common_lib.child import HarwareReadingServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from audio_common_msgs.msg import AudioData

class Status():
    """ Status of the speaker service """
    INIT = 0 # init the service
    SPEAKING = 1 # start speak
    NOT_SPEAKING = 2 # stop speak
    END = 3  # terminate the service


class SpeakerService(HarmoniExternalServiceManager):
    """
    Speaker service
    """

    def __init__(self, name, param):
        """ Initialization of variables and speaker parameters """
        rospy.loginfo("SpeakerService initializing")
        self.name = name
       
        """ Setup the speaker """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16  # How can we trasform it in a input parameter?
        self.stream = None
        """Setup the speaker service as server """
        self.status = Status.INIT 
        super(SpeakerService, self).__init__(self.status)
        return

    def status_update(self):
        super(SpeakerService, self).update(self.status)
        return

    def test(self):
        super(SpeakerService, self).test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def start(self):
        rospy.loginfo("Start the %s service" % self.name)
        rate = ""
        super(SpeakerService, self).start(rate)
        if self.status == 0:
            self.status = Status.LISTENING
            self.status_update()
            try:
                self.open_stream()
                self.listen() # Start the speaker service at the INIT
            except:
                self.status = Status.END
        else:
            self.status = Status.LISTENING
        self.status_update()
        #self.listen()

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        super(SpeakerService, self).stop()
        self.close_stream()
        self.status = Status.END
        self.status_update()

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        super(SpeakerService, self).pause()
        self.status = Status.NOT_LISTENING
        self.status_update()

    def setup_speaker(self):
        """ Setup the speaker """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the audio input stream")
        self.stream = self.p.open(
            format=self.audio_format,
            channels=self.total_channels,
            rate=self.audio_rate, input=True,
            output_device_index=self.output_device_index,
            frames_per_buffer=self.chunk_size
        )
        return

    def close_stream(self):
        """Closing the stream """
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
