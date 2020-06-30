# HARMONI-PC
PC Hardware wrapper for HARMONI low-level interfaces

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