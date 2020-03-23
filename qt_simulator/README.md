# Instructions

## Robot state

### Setup RVIZ on QT
The commands are:

* Run the following command for converting the QT joints from degrees to radians: 
```
rosrun qt_robot_state qt_joint_radians.py 
```
* Run the following command for running the RViz simulator of QTRobot:
```
roslaunch qt_robot_state qt_robot_state.launch
```



### Setup RVIZ simulator of QT model
* Run the following command for running the RViz simulator of QTRobot:
```
roslaunch qt_robot_state simulator_robot_state.launch
```

## Face recognition 

### Setup QT for face recognition

* Run the following command for running the nuitrack:
```
roslaunch nuitrack_body_tracker nuitrack_body_trackelaunch
```

### Run the face recognition
* Run the following command for running the tf listener:
```
roslaunch qt_robot_state qt_robot_state.launch
```
* Run the following command for running the face recognition:
```
roslaunch qt_face_recognition qt_face_recognition.launch
```

* Run the following command for getting the depth information from camera:
```
rosrun qt_face_recognition depht_rgb_coordinator.py
```

