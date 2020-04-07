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

class MicrophoneService(HarmoniServiceManager):
    """
    Microphone service
    """
    def __init__(self, mic_param):
        """ Initialization of variables and microphone parameters """
        rospy.loginfo("MicrophoneService initializing")
        self.audio_format_width = mic_param["audio_format_width"]
        self.chunk_size = mic_param["chunk_size"]
        self.total_channels = mic_param["total_channels"]
        self.audio_rate = mic_param["audio_rate"]
        self.silence_limit_seconds = mic_param["silence_limit_seconds"]
        self.previous_audio_seconds = mic_param["previous_audio_seconds"]
        self.total_silence_samples = mic_param["total_silence_samples"]
        self.silence_threshold = mic_param["silence_threshold"]
        self.loudest_sound_value = mic_param["loudest_sound_value"]
        self.device_name = mic_param["device_name"]
        self.set_threshold = mic_param["set_threshold"]
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16 # How can we trasform it in a input parameter?
        self.stream = None
        self.state = "Listening"
        self.setup_microphone()
        status = ""
        super(MicrophoneService, self).__init__(status)

    def setup_microphone(self):
        """ Setup the microphone """
        rospy.loginfo("Setting up the microphone")
        self.get_index_device() # get index of the input audio device
        self.determine_silence_threshold(self.set_threshold)
        return

    def open_stream(self):
        rospy.loginfo("Opening the audio input stream")
        self.stream = self.p.open(
            format = self.audio_format, 
            channels= self.total_channels, 
            rate=self.audio_rate, input=True, 
            input_device_index=self.input_device_index, 
            frames_per_buffer=self.chunk_size
            )
        return

    def close_stream(self):
        self.stream.stop_stream()
        self.stream.close()
        return

    def listen(self):
        self.open_stream()
        rospy.loginfo("The microphone is listening")
        current_audio = ""
        chunks_per_second = int(self.audio_rate / self.chunk_size)
        sliding_window = deque(maxlen = self.silence_limit_seconds * chunks_per_second)
        prev_audio = deque(maxlen = self.previous_audio_seconds * chunks_per_second)
        self.started = False
        while not rospy.is_shutdown():
            latest_audio_data = self.stream.read(self.chunk_size)
            if self.state == "Listening":
                sliding_window.append(math.sqrt(abs(audioop.avg(latest_audio_data, self.audio_format_width))))
                if any([x > self.silence_threshold for x in sliding_window]):
                    if not self.started:
                        rospy.loginfo("Sound detected")
                        self.started = True
                    current_audio += latest_audio_data
                elif self.started:
                    rospy.loginfo("Finished detecting")
                    all_audio_data = "".join(prev_audio) + current_audio
                    #self.stream.stop_stream()
                    self.state = "Stop listening"
                    audio_bitstream = np.fromstring(all_audio_data, np.uint8)
                    audio = audio_bitstream.tolist()
                    self.started = False
                    sliding_window.clear()
                    prev_audio.clear()
                    current_audio = ""
                else:
                    prev_audio.append(latest_audio_data)
        return

    def get_index_device(self):
        """ 
        Find the input audio devices configured in ~/.asoundrc. 
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            if device["name"] == self.device_name:
                rospy.loginfo("Found device with name "+ self.device_name)
                self.input_device_index = i
                return

    def determine_silence_threshold(self, mode):
        loudest_sound_cohort_size = 0.2
        silence_threshold_multiplier = 1.6
        if mode == "default":
            self.open_stream()
            tss = self.total_silence_samples
            values = [math.sqrt(abs(audioop.avg(self.stream.read(self.chunk_size), self.audio_format_width))) for _ in range(tss)]
            values = sorted(values, reverse=True)
            sum_of_loudest_sounds= sum(values[:int(tss * loudest_sound_cohort_size)])
            total_samples_in_cohort = int(tss * loudest_sound_cohort_size)
            average_of_loudest_sounds = sum_of_loudest_sounds / total_samples_in_cohort
            self.close_stream()        
        elif mode == "constant":
            average_of_loudest_sounds = self.loudest_sound_value
        rospy.loginfo("Average audio intensity is "+ str(average_of_loudest_sounds))
        self.silence_threshold = average_of_loudest_sounds * silence_threshold_multiplier
        rospy.loginfo("Silence threshold set to " + str(self.silence_threshold))
        return




def main():
    try: 
        service_name = "microphone"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        mic_param = rospy.get_param("/mic_param/")
        # I am not 100% sure but I think you need to pass the same set of args to a parent init
        # Or possible use *args, *kwargs
        ms = MicrophoneService(mic_param)
        ms.listen()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

    
    