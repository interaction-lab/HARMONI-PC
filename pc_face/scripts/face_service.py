#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import ast
import os
import json
from pc_face.msg import FaceRequest
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
        self.min_duration_viseme = param["min_duration_viseme"]
        self.speed_viseme = param["speed_viseme"]
        self.timer_interval = param["timer_interval"]
        """ Setup the publisher for the face """
        self.face_pub = rospy.Publisher("harmoni/actuating/expressing/face", FaceRequest, queue_size=1)
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
        data = super(FaceService, self).do(data)
        [valid_face_expression, visemes] = self.get_face_data(data)
        try:
            self.status = Status.EXPRESSING
            self.actuation_update(actuation_completed = False)
            viseme_ids = map(lambda b: b["id"], visemes)
            viseme_times = map(lambda b: b["start"], visemes)
            viseme_request = FaceRequest(visemes=viseme_ids, viseme_ms=self.speed_viseme, times=viseme_times)
            t = Timer(self.timer_interval, self.send_face_request(viseme_request))
            t.start()
            start_time = rospy.Time.now()
            rospy.loginfo("The last viseme lasts %i" %viseme_times[-1])
            rospy.sleep(viseme_times[-1] + self.min_duration_viseme)
            for f in validated_face_expr:
                aus = map(lambda s: s[2:], f['aus'])
                au_ms = f['au_ms']*1000
                face_request = FaceRequest(aus=aus, au_degrees=f['au_degrees'], au_ms=au_ms)
                t = Timer(self.timer_interval, self.send_face_request(face_request))
                t.start()
                start_time = rospy.Time.now()
                if f == validated_face_expr[len(validated_face_expr)-1]:
                    rospy.sleep(au_ms)
            self.status = Status.NOT_EXPRESSING
            self.actuation_update(actuation_completed = True)
        except:
            self.status = Status.END
            self.actuation_update(actuation_completed = True)
        return

    def setup_face(self):
        """ Setup the face """
        rospy.loginfo("Setting up the %s" % self.name)
        rospy.loginfo("Checking that face is connected to ROS websocket")
        rospy.wait_for_service("harmoni/actuating/face/is_connected")
        rospy.loginfo("Done, face is connected to ROS websocket")
        self.face_expression = self.get_facial_expressions_list()
        self.visemes =  visemes = ["BILABIAL","LABIODENTAL","INTERDENTAL","DENTAL_ALVEOLAR","POSTALVEOLAR","VELAR_GLOTTAL","CLOSE_FRONT_VOWEL","OPEN_FRONT_VOWEL","MID_CENTRAL_VOWEL","OPEN_BACK_VOWEL","CLOSE_BACK_VOWEL", 'IDLE']
        return

    def send_face_request(self, face_request):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face")
        self.face_pub.publish(face_request)
        return

    def get_facial_expressions_list(self):
        """ Get facial expression list from the resource file"""
        facial_expression_list = []
        face_expression_au = []
        base_dir = os.path.dirname(__file__)
        with open(base_dir + '/resource/cordial_face_expression.json', "r") as json_file:
            data = json.load(json_file)
            for facial_expression in data:
                facial_expression_list.append(facial_expression)
                au_name = str(facial_expression)
                aus = []
                for dofs in data[facial_expression]['dofs']:
                    aus.append(str(dofs))
                for keyframe in data[facial_expression]['keyframes']:
                    au_degrees = keyframe['pose']
                    au_ms = keyframe['time']
                    face_expression_au.append({ "au_name": au_name,
                                "aus": aus, 
                                "au_degrees": au_degrees,
                                "au_ms": au_ms})
        return face_expression_au

    def get_valid_face_data(self, data):
        """ Get the validated data of the face"""
        data = ast.literal_eval(data)
        viseme = filter(lambda b: b["id"] in self.visemes, data)
        facial_expression= filter(lambda b: b["id"] in self.face_expression, data)
        ordered_facial_data = sorted(facial_expression, key=lambda face: face["start"])
        validated_face_expr = []
        for au in self.face_expression:
            for fexp in  ordered_facial_data:
                if au['au_name'] == fexp['id']:
                    validated_face_expr.append(au)
        for i in range(0,len(viseme)-1):
                viseme[i]["duration"]=viseme[i+1]["start"]-viseme[i]["start"]
        viseme[-1]["duration"]=self.min_duration_viseme
        viseme_behaviors=filter(lambda b: b["duration"]>= self.min_duration_viseme, viseme)
        ordered_visemes = sorted(viseme_behaviors, key=lambda b: b["start"])
        return (validated_face_expr, visemes)

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
