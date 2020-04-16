#!/usr/bin/env python3                                                          
import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerResponse
import os

web_param = rospy.get_param("/web_param/")

def callback_srv(data):
    #os.system("pkill luakit")
    process = subprocess.Popen(web_param["shell_command"], shell=True)
    # process = subprocess.Popen("echo hi", shell=True)

    resp = TriggerResponse()
    resp.success = True
    resp.message = "Browser started with PID: %d" % process.pid
    return resp

if __name__ == "__main__":
    rospy.init_node("browser_starter_server")
    srv = rospy.Service('start_browser', Trigger, callback_srv)
    rospy.spin()
