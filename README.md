# thirdarm_arbotix
The Third Arm project - arbotix speech command and keyboard control
## Version control
Readme.md created on 02/01/2017 by, Rain Zhou <br />
Last commit: Nov 27, 2016, by Rain Zhou


## Dependency
Please see package.xml in the main folder for dependency. Note that this package is depended on 
arbotix_python package (http://wiki.ros.org/arbotix_python).

## Running the arm
First, launch arbotix.launch in launch folder. This runs the arbotix controller driver in the package arbotix_python, 
which reads arbotix.yaml. This yaml file configures communication settings, joints (motors) and controllers parameters.

#### Speech Command
To use speech control, launch thirdarm_speech.launch in launch folder.
This launches command_listener.py, speech_listener.py, and speech_recognizer.py files in src folder.
All other files in the data folder, besides arbotix.yaml, are relevant to the speech command. Modification of
commands requires updates on all data files.


#### Keyboard Control
To use keyboard command, launch teleop_twist_keyboard.py in the src folder.
This python file requires no other files.
