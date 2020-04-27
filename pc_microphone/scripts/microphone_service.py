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
from harmoni_common_lib.child import HarwareReadingServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from audio_common_msgs.msg import AudioData

class MicrophoneService(HarmoniServiceManager):
    """
    Microphone service
    """

    def __init__(self, name, param):
        """ Initialization of variables and microphone parameters """
        rospy.loginfo("MicrophoneService initializing")
        self.name = name
        self.audio_format_width = param["audio_format_width"]
        self.chunk_size = param["chunk_size"]
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.silence_limit_seconds = param["silence_limit_seconds"]
        self.previous_audio_seconds = param["previous_audio_seconds"]
        self.total_silence_samples = param["total_silence_samples"]
        self.silence_threshold = param["silence_threshold"]
        self.loudest_sound_value = param["loudest_sound_value"]
        self.device_name = param["device_name"]
        self.set_threshold = param["set_threshold"]
        """ Setup the microphone """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16  # How can we trasform it in a input parameter?
        self.stream = None
        self.setup_microphone()
        """Init the publisher """
        self.mic_pub = rospy.Publisher("/harmoni/sensing/listening/microphone", AudioData, queue_size=1) # Publishing the voice data
        self.mic_raw_pub = rospy.Publisher("/harmoni/sensing/microphone", AudioData, queue_size=1) # Publishing raw_data
        """Setup the microphone service as server """
        self.state = State.INIT 
        super().__init__(self.state)
        return

    def state_update(self):
        super().update(self.state)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        super().start(rate)
        if self.state == State.INIT:
            self.state = State.START
            self.state_update()
            try:
                self.open_stream()
                self.listen() # Start the microphone service at the INIT
            except:
                self.state = State.FAILED
        else:
            self.state = State.START
        self.state_update()
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        super().stop()
        try:
            self.close_stream()
            self.state = State.SUCCESS
            self.state_update()
        except:
            self.state = State.FAILED
            self.state_update()
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        super().pause()
        self.state = State.SUCCESS
        self.state_update()
        return

    def setup_microphone(self):
        """ Setup the microphone """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()  # get index of the input audio device
        self.determine_silence_threshold(self.set_threshold)
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the audio input stream")
        self.stream = self.p.open(
            format=self.audio_format,
            channels=self.total_channels,
            rate=self.audio_rate, input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.chunk_size
        )
        return

    def close_stream(self):
        """Closing the stream """
        self.stream.stop_stream()
        self.stream.close()
        return

    def listen(self):
        """Listening from the microphone """
        rospy.loginfo("The %s is listening" % self.name)
        current_audio = ""
        chunks_per_second = int(self.audio_rate / self.chunk_size)
        sliding_window = deque(maxlen=self.silence_limit_seconds * chunks_per_second)
        prev_audio = deque(maxlen=self.previous_audio_seconds * chunks_per_second)
        started = False
        while not rospy.is_shutdown():
            if self.state != State.FAILED:
                latest_audio_data = self.stream.read(self.chunk_size, exception_on_overflow = False)
                raw_audio_bitstream = np.fromstring(latest_audio_data, np.uint8)
                raw_audio = raw_audio_bitstream.tolist()
                self.mic_raw_pub.publish(raw_audio) # Publishing raw AudioData
                if self.state == State.START:
                    sliding_window.append(math.sqrt(abs(audioop.avg(latest_audio_data, self.audio_format_width))))
                    if any([x > self.silence_threshold for x in sliding_window]):
                        if not started:
                            rospy.loginfo("Sound detected")
                            started = True
                        print("Current audio append")
                        current_audio += latest_audio_data
                        print("End append")
                    elif started:
                        rospy.loginfo("Finished detecting")
                        all_audio_data = "".join(prev_audio) + current_audio
                        self.state = State.SUCCESS
                        audio_bitstream = np.fromstring(all_audio_data, np.uint8)
                        audio = audio_bitstream.tolist()
                        self.mic_pub.publish(audio)  # Publishing AudioData of voice
                        started = False
                        sliding_window.clear()
                        prev_audio.clear()
                        current_audio = ""
                        self.state_update()
                    else:
                        prev_audio.append(latest_audio_data)
            else:
                rospy.loginfo("Break the while")
                break
        return

    def get_index_device(self):
        """ 
        Find the input audio devices configured in ~/.asoundrc. 
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            if device["name"] == self.device_name:
                rospy.loginfo("Found device with name " + self.device_name)
                self.input_device_index = i
                return

    def determine_silence_threshold(self, mode):
        """Determine silence threshold from the mic or setting a constant value """
        loudest_sound_cohort_size = 0.2
        silence_threshold_multiplier = 1.6
        if mode == "default":
            self.open_stream()
            tss = self.total_silence_samples
            values = [math.sqrt(abs(audioop.avg(self.stream.read(self.chunk_size), self.audio_format_width))) for _ in range(tss)]
            values = sorted(values, reverse=True)
            sum_of_loudest_sounds = sum(values[:int(tss * loudest_sound_cohort_size)])
            total_samples_in_cohort = int(tss * loudest_sound_cohort_size)
            average_of_loudest_sounds = sum_of_loudest_sounds / total_samples_in_cohort
            self.close_stream()
        elif mode == "constant":
            average_of_loudest_sounds = self.loudest_sound_value
        rospy.loginfo("Average audio intensity is " + str(average_of_loudest_sounds))
        self.silence_threshold = average_of_loudest_sounds * silence_threshold_multiplier
        rospy.loginfo("Silence threshold set to " + str(self.silence_threshold))
        return


def main():
    try:
        service_name = "microphone"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+service_name+"_param/")
        s = MicrophoneService(service_name, param)
        hardware_reading_server = HarwareReadingServer(name=service_name, service_manager=s)
        hardware_reading_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
