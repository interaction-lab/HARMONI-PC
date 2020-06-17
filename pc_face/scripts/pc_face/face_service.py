#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import ast
import os
import json
from threading import Timer
from pc_face.msg import FaceRequest
from harmoni_common_lib.constants import State, RouterActuator, HelperFunctions
from harmoni_common_lib.child import HardwareControlServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager

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
        self.service_id =  HelperFunctions.get_child_id(self.name)
        """ Setup the face """
        self.setup_face()
        """ Setup the publisher for the face """
        self.face_pub = rospy.Publisher(RouterActuator.face.value + self.service_id +"/expressing", FaceRequest, queue_size=1)
        """Setup the face service as server """
        self.state = State.INIT 
        super().__init__(self.state)
        return

    def actuation_update(self, actuation_completed):
        """Update the actuation state """
        rospy.loginfo("Update face state")
        super().update(state = self.state, actuation_completed=actuation_completed)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def do(self, data):
        """ Do the expression"""
        data = super().do(data)
        [valid_face_expression, visemes] = self.get_face_data(data)
        try:
            self.state = State.REQUEST
            self.actuation_update(actuation_completed = False)
            if visemes != []:
                viseme_ids = list(map(lambda b: b["id"], visemes))
                viseme_times = list(map(lambda b: b["start"], visemes))
                self.face_request = FaceRequest(visemes=viseme_ids, viseme_ms=self.speed_viseme, times=viseme_times)
                rospy.loginfo("The viseme request is %s" %self.face_request)
                t = Timer(self.timer_interval, self.send_face_request)
                t.start()
                start_time = rospy.Time.now()
                rospy.loginfo("The last viseme lasts %i" %viseme_times[-1])
                time_sleep = int(viseme_times[-1]) + self.min_duration_viseme
                rospy.sleep(time_sleep)
            if valid_face_expression != []:
                rospy.loginfo("Valid face expression not null")
                if len(valid_face_expression) > 1:
                    for ind, f in range(0, len(validated_face_expr)-1):
                        rospy.loginfo("The valid expression is %s" %f)
                        aus = list(map(lambda s: s[2:], f['aus']))
                        au_ms = f['au_ms']*1000
                        self.face_request = FaceRequest(aus=aus, au_degrees=f['au_degrees'], au_ms=au_ms)
                        rospy.loginfo("The face expression request is %s" %self.face_request)
                        t = Timer(self.timer_interval, self.send_face_request)
                        t.start()
                        start_time = rospy.Time.now()
                aus = list(map(lambda s: s[2:], valid_face_expression[-1]['aus']))
                au_ms =  valid_face_expression[-1]['au_ms']*1000
                self.face_request = FaceRequest(aus=aus, au_degrees= valid_face_expression[-1]['au_degrees'], au_ms=au_ms)
                rospy.logdebug("The face expression request is %s" %self.face_request)
                t = Timer(self.timer_interval, self.send_face_request)
                t.start()
                start_time = rospy.Time.now()
                rospy.loginfo("The last facial expression")
                rospy.sleep(valid_face_expression[-1]['au_ms'])
            self.state = State.SUCCESS
            self.actuation_update(actuation_completed = True)
        except:
            self.state = State.FAILED
            self.actuation_update(actuation_completed = True)
        return

    def setup_face(self):
        """ Setup the face """
        rospy.loginfo("Setting up the %s" % self.name)
        rospy.loginfo("Checking that face is connected to ROS websocket")
        rospy.wait_for_service("/harmoni/actuating/face/is_connected")
        rospy.loginfo("Done, face is connected to ROS websocket")
        [self.face_expression, self.face_expression_names] = self.get_facial_expressions_list()
        self.visemes =  ["BILABIAL","LABIODENTAL","INTERDENTAL","DENTAL_ALVEOLAR","POSTALVEOLAR","VELAR_GLOTTAL","CLOSE_FRONT_VOWEL","OPEN_FRONT_VOWEL","MID_CENTRAL_VOWEL","OPEN_BACK_VOWEL","CLOSE_BACK_VOWEL", 'IDLE']
        return

    def send_face_request(self):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face")
        self.face_pub.publish(self.face_request)
        return

    def get_facial_expressions_list(self):
        """ Get facial expression list from the resource file"""
        facial_expression_list = []
        face_expression_au = {}
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
                    face_expression_au[au_name] = {
                                "aus": aus, 
                                "au_degrees": au_degrees,
                                "au_ms": au_ms}
        return face_expression_au, facial_expression_list

    def get_face_data(self, data):
        """ Get the validated data of the face"""
        rospy.logdebug("The face expressions available are %s" %self.face_expression)
        data = ast.literal_eval(data)
        viseme = list(filter(lambda b: b["id"] in self.visemes, data))
        facial_expression= list(filter(lambda b: b["id"] in self.face_expression_names, data))
        rospy.logdebug("The facial expressions are %s" %facial_expression)
        ordered_facial_data = list(sorted(facial_expression, key=lambda face: face["start"]))
        rospy.logdebug("The list of au is %s" %self.face_expression)
        validated_face_expr = []
        for fexp in ordered_facial_data:
            validated_face_expr.append(self.face_expression[fexp["id"]])
        for i in range(0, len(viseme)-1):
                viseme[i]["duration"]=viseme[i+1]["start"]-viseme[i]["start"]
        viseme[-1]["duration"]=self.min_duration_viseme
        viseme_behaviors=list(filter(lambda b: b["duration"]>= self.min_duration_viseme, viseme))
        ordered_visemes = list(sorted(viseme_behaviors, key=lambda b: b["start"]))
        rospy.logdebug("The facial expressions are %s" %validated_face_expr)
        return (validated_face_expr, ordered_visemes)

def main():
    test = rospy.get_param("/test/")
    input_test = rospy.get_param("/input_test/")
    id_test = rospy.get_param("/id_test/")
    try:
        service_name = RouterActuator.face.name
        rospy.init_node(service_name)
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param("~"+service_id+"_param/")
            s = FaceService(service, param)
            service_server_list.append(HardwareControlServer(name=service, service_manager=s))
        if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" %(service))
                s.do(input_test)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
