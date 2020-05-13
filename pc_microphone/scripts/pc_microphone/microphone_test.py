#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import pyaudio
import wave
from audio_common_msgs.msg import AudioData

class MicrophoneTest():
    """
    Microphone service testing
    """

    def __init__(self, name, param):
        """ Initialization of variables and microphone parameters """
        rospy.loginfo("Microphone Test initializing")
        self.name = name
        self.file_path = param["test_outdir"]
        self.audio_format_width = param["audio_format_width"]
        self.chunk_size = param["chunk_size"]
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.first_audio_frame = True
        """ Setup the microphone """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16
        """Init the subscriber """
        self.mic_sub = rospy.Subscriber("/harmoni/sensing/listening/microphone", AudioData, self._record_audio_data_callback, queue_size=1) # Publishing the voice data
        return


    def _record_audio_data_callback(self, data):
        if self.first_audio_frame:
            self.wf = wave.open(self.file_path, 'wb')
            self.wf.setnchannels(self.total_channels)
            self.wf.setsampwidth(self.p.get_sample_size(self.audio_format))
            self.wf.setframerate(self.audio_rate)
            self.wf.setnframes(self.chunk_size)
            self.wf.writeframes(''.join(data.data))
            self.first_audio_frame = False
        else:
            self.wf.writeframes(''.join(data.data))
        return


def main():
    try:
        service_name = "microphone"
        id_service = "def"
        rospy.init_node(service_name + "_test_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+id_service+"_param/")
        s = MicrophoneTest(service_name, param)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
