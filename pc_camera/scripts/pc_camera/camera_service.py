#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from harmoni_common_lib.constants import State, RouterSensor, HelperFunctions
from harmoni_common_lib.child import HarwareReadingServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from sensor_msgs.msg import Image

class CameraService(HarmoniServiceManager):
    """
    Camera service
    """

    def __init__(self, name, param):
        """ Initialization of variables and camera parameters """
        rospy.loginfo("CameraService initializing")
        self.name = name
        self.input_device_index = param["input_device_index"]
        self.show = param["show"]
        self.video_format = param["video_format"]
        """ Setup the camera """
        self.cv_bridge = CvBridge()
        """ Init the camera publisher"""
        self._video_pub = rospy.Publisher("/harmoni/sensing/watching/pc_camera", Image, queue_size=1)
        """Setup the camera service as server """
        self.setup_camera()
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
                self.watch() # Start the camera service at the INIT
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

    def setup_camera(self):
        """ Setup the camera """
        rospy.loginfo("Setting up the %s" % self.name)
        self.video_cap = cv2.VideoCapture(self.input_device_index)
        self.open_stream()
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the video input stream")
        self.width = int(self.video_cap.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5)
        self.height = int(self.video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5)
        self.size = (self.width, self.height)
        self.fps = self.video_cap.get(cv2.CAP_PROP_FPS)
        rospy.set_param("/"+self.name+"_param/fps/", self.fps)
        return

    def close_stream(self):
        """Closing the stream """
        self.video_cap.release()
        if self.show:
            cv2.destroyAllWindows()
        return

    def watch(self):
        while not rospy.is_shutdown():
            _, frame = self.video_cap.read()
            image = self.cv_bridge.cv2_to_imgmsg(frame, self.video_format)
            self._video_pub.publish(image)
            if self.show:
                cv2.imshow("PcCameraVideo", frame)
            if cv2.waitKey(1) and (0xFF == ord('x')) and self.show:
                break
        return

def main():
    args = sys.argv
    try:
        service_name = RouterSensor.CAMERA.value
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param("/"+service_id+"_param/")
            s = CameraService(service, param)
            service_server_list.append(HarwareReadingServer(name=service, service_manager=s))
            if eval(args[1]): #FIX IT 
                s.start()
        for server in service_server_list:
            server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
