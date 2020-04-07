#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
#import pyaudio
#from harmoni_common_lib.child import HardwareReadingServer
from harmoni_common_lib.service_manager import HarmoniServiceManager

class MicrophoneService(HarmoniServiceManager):
    """
    Microphone service
    """
    def __init__(self, mic_param):
        rospy.loginfo("MicrophoneService initializing")
        status = ""
        super(MicrophoneService, self).__init__(status)
        #self.p = pyaudio.PyAudio()
        return


def main():
    try: 
        service_name = "microphone"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        mic_param = rospy.get_param("/mic_param/")
        print(mic_param)
        # I am not 100% sure but I think you need to pass the same set of args to a parent init
        # Or possible use *args, *kwargs
        ms = MicrophoneService(mic_param)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

    
    