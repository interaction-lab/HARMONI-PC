# HARMONI-PC
PC Hardware wrapper for HARMONI low-level interfaces

Currently based on:

* https://github.com/robotpt/qt-face-tracking (original source)
* https://github.com/shinselrobots/body_tracker_msgs
* https://github.com/shinselrobots/nuitrack_body_tracker
* https://github.com/ros/robot_state_publisher

Note that the use of git submodules is presently being avoided to encourage modification of any dependencies on an as needed basis. We may later transition to a more proper repository structure as our codebase becomes more mature.

See simulator instructions [here](qt_simulator/README.md)


## Setup Instructions

1. Clone the repository in <your_catkin_workspace>/src

~~~~
$ git clone https://github.com/interaction-lab/HARMONI-PC.git 
~~~~
2. Install package dependencies:
~~~~
$ sudo apt-get install python3-boto3
$ pip3 install boto3 --upgrade
$ sudo apt-get install python3-pyaudio
$ sudo apt-get install python3-soundfile
~~~~

3. Make everything:
~~~~
$ cd ..
$ catkin init 
$ catkin config -DPYTHON_EXECUTABLE:=/usr/bin/python3
$ catkin build 
~~~~

4. Now you can test it, following the instructions in the HARMONI repo
