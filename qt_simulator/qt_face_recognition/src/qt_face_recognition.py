#!/usr/bin/env python
from __future__ import print_function

# import sys
import rospy
import cv2
import threading
import math
import numpy as np
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo
import face_recognition


width_frame = 848
height_frame = 480



class image_converter:
    #faces = None
    #faces_time = None
    

    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.count = 0
        self.face_locations = []
        self.face_encodings = []
        self.face_names = []
        self.face_frame_pub = rospy.Publisher("qtpc/face_frame", Pose2D, queue_size=10)
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image",Image, self.image_callback)
        #self.face_sub = rospy.Subscriber("/qt_nuitrack_app/faces", Faces, self.face_callback)
        


    def face_track(self, frame, small_frame):
        self.face_locations = face_recognition.face_locations(small_frame)
        self.face_encodings = face_recognition.face_encodings(small_frame, self.face_locations)
        if self.face_locations == []:
            print("Not in the frame")
        #    face_names = []
        for face_encoding in self.face_encodings:
            name = "Unknown"
            self.face_names.append(name)
        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            # Draw a box around the face
            cv2.rectangle(frame, (left, top),(right, bottom), (0, 0, 255), 2)
            width_rec = bottom - top
            top_rec = right - left
            Z_top_frame = top_rec/2 + top
            X_left_frame = width_rec/2 + left
            Z_origin_frame = height_frame/2
            X_origin_frame = width_frame/2
            x_vec = X_left_frame - X_origin_frame
            z_vec = Z_origin_frame - Z_top_frame
            theta_deg = 0
            '''
            theta = math.atan(z_vec/x_vec)
            if (x_vec > 0 and z_vec > 0) or (x_vec < 0 and z_vec < 0):
                theta = np.pi/2 - theta
                theta_deg = math.degrees(theta)
                if x_vec < 0 and z_vec < 0:
                    theta_deg = -(theta_deg)
            elif (x_vec < 0 and z_vec > 0) or (x_vec > 0 and z_vec < 0):
                theta = np.pi/2 + theta
                theta_deg = math.degrees(theta)
                if x_vec < 0 and z_vec > 0:
                    theta_deg = - theta_deg
            '''
            pixel_column = X_left_frame
            pixel_row = Z_top_frame
            #  Draw the frames origin of the image (green) and of the target (blue)
            cv2.rectangle(frame, (int(X_origin_frame), int(Z_origin_frame)), (int(X_origin_frame) + 1, int(Z_origin_frame) + 1), (255, 0, 0), 2)
            cv2.rectangle(frame, (int(X_left_frame), int(Z_top_frame)), (int(X_left_frame) + 1, int(Z_top_frame) + 1), (0, 255, 0), 2)
            self.face_frame_pub.publish(float(pixel_column), float(pixel_row),float(theta_deg))
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            #pass
        except CvBridgeError as e:
            print(e)
        
    
    def face_callback(self, data):
        # print("face_callback")
        self.lock.acquire()
        self.faces = data.faces
        self.faces_time = rospy.Time.now()
        self.lock.release()

    def image_callback(self,data):
        self.count += 1
        if (self.count % 10) == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            (rows, cols, channels) = cv_image.shape
            #self.lock.acquire()
            small_frame = cv2.resize(cv_image, (0, 0), fx=0.25, fy=0.25)
            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = small_frame[:, :, ::-1]
            self.face_track(cv_image, rgb_small_frame)
        #self.lock.release()
        
        """
        new_faces = self.faces
        new_faces_time = self.faces_time
        self.lock.release()

        if new_faces and (rospy.Time.now()-new_faces_time) < rospy.Duration(5.0):
            for face in new_faces:
                rect = face.rectangle
                cv2.rectangle(cv_image, (int(rect[0]*cols),int(rect[1]*rows)),
                                      (int(rect[0]*cols+rect[2]*cols), int(rect[1]*rows+rect[3]*rows)), (0,255,0), 2)
                x = int(rect[0]*cols)
                y = int(rect[1]*rows)
                w = int(rect[2]*cols)
                h = int(rect[3]*rows)
                #cv2.putText(cv_image, "Gender:", (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), lineType=cv2.LINE_AA)
                cv2.putText(cv_image, "Gender: %s" % face.gender, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
                cv2.putText(cv_image, "Age: %d" % face.age_years, (x, y+h+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)

                # Neutral
                cv2.putText(cv_image, "Neutral:", (x, y+h+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
                cv2.rectangle(cv_image, (x+80,y+h+50),
                                      (x+80+int(face.emotion_neutral*100), y+h+10+50), (0,255,0), cv2.FILLED)
                cv2.rectangle(cv_image, (x+80,y+h+50),
                                      (x+80+100, y+h+10+50), (255,255,255), 1)
                # Angry
                cv2.putText(cv_image, "Angry:", (x, y+h+80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
                cv2.rectangle(cv_image, (x+80,y+h+70),
                                      (x+80+int(face.emotion_angry*100), y+h+10+70), (0,255,0), cv2.FILLED)
                cv2.rectangle(cv_image, (x+80,y+h+70),
                                      (x+80+100, y+h+10+70), (255,255,255), 1)

                # Happy
                cv2.putText(cv_image, "Happy:", (x, y+h+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
                cv2.rectangle(cv_image, (x+80,y+h+90),
                                      (x+80+int(face.emotion_happy*100), y+h+10+90), (0,255,0), cv2.FILLED)
                cv2.rectangle(cv_image, (x+80,y+h+90),
                                      (x+80+100, y+h+10+90), (255,255,255), 1)

                cv2.putText(cv_image, "Surprise:", (x, y+h+120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
                cv2.rectangle(cv_image, (x+80,y+h+110),
                                      (x+80+int(face.emotion_surprise*100), y+h+10+110), (0,255,0), cv2.FILLED)
                cv2.rectangle(cv_image, (x+80,y+h+110),
                                      (x+80+100, y+h+10+110), (255,255,255), 1)
        """
       


if __name__ == '__main__':
    rospy.init_node('qt_face_recognition', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
