#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import pyaudio
import math
import audioop
import numpy as np
import ast
from collections import deque
from harmoni_common_lib.constants import State, RouterActuator, HelperFunctions
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
        self.device_name = param["device_name"]
        self.output_device_index = None
        """ Setup the speaker """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16  # How can we trasform it in a input parameter?
        self.stream = None
        """Setup the speaker service as server """
        self.setup_speaker()
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
            self.open_stream()
            rospy.loginfo("Writing data for speaker")
            #data = ast.literal_eval(data)
            self.stream.write(data)
            rospy.sleep(1)
            self.close_stream()
            self.state = State.SUCCESS
            self.actuation_update(actuation_completed = True)
        except:
            rospy.loginfo("Speaker failed")
            self.state = State.FAILED
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
        self.stream = self.p.open(format=self.audio_format,
            channels=self.total_channels,
            rate=self.audio_rate,
            input = False,
            output = True,
            output_device_index=self.output_device_index,
            frames_per_buffer=self.chunk_size)
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

    def wav_to_data(self, path):
        """ 
        WAV to audiodata
        """
        file_handle = path
        data = np.fromfile(file_handle, np.uint8)[24:] #Loading wav file
        data = data.astype(np.uint8).tostring()
        return data

def main():
    test = rospy.get_param("/test/")
    input_test = rospy.get_param("/input_test/")
    id_test = rospy.get_param("/id_test/")
    try:
        service_name = RouterActuator.SPEAKER.value
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param("/"+service_id+"_param/")
            s = SpeakerService(service, param)
            service_server_list.append(HardwareControlServer(name=service, service_manager=s))
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" %(service))
                data = s.wav_to_data(input_test)
                s.do(data)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
