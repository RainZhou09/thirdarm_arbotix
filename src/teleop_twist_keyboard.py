#!/usr/bin/env python
import roslib; roslib.load_manifest('thirdarm')
import rospy
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to servo/command!
---------------------------
first player:
	w     
   a    s    d

second player:
   k    l
   ,    .

CTRL-C to quit
"""

moveBindings = {
		'w': (0.0,-0.1,0.0,0.0), 
		's': (0.0,0.1,0.0,0.0),
		'a': (-0.25,0.0,0.0,0.0),
		'd': (0.25,0.0,0.0,0.0),
		'.': (0.0,0.0,-0.25,0.0),
		',': (0.0,0.0,0.25,0.0),
		'l': (0.0,0.0,0.0,1.1),
		'k': (0.0,0.0,0.0,-1.1),
		   }

def callbackPose(msg): 
	global base_pose, elbow_pose, arm_pose, hand_pose
	elbow_pose = msg.position[0]
	base_pose = msg.position[1]
	#wrist_pose = msg.position[2]
	arm_pose = msg.position[3]
	hand_pose = msg.position[4]

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub_base = rospy.Publisher('/base/command', Float64, queue_size=1)
	pub_elbow = rospy.Publisher('/elbow/command', Float64, queue_size=1)
	pub_arm = rospy.Publisher('/arm/command', Float64, queue_size=1)
	#pub_wrist = rospy.Publisher('/wrist/command', Float64, queue_size=1)
	pub_hand = rospy.Publisher('/hand/command', Float64, queue_size=1)

	rospy.init_node('teleop_twist_keyboard')

	# sub to current servo positions
	rospy.Subscriber('/joint_states', JointState, callbackPose)

	# set up set_speed service
	elbow_speed = rospy.ServiceProxy('/elbow/set_speed', arbotix_msgs.srv.SetSpeed)
	elbow_speed(0.157)

	# re-declare global variales and initialize them
	global base_pose, elbow_pose, arm_pose, hand_pose

	base_pose = 0.0
	elbow_pose = 0.0
	#wrist_pose = 0.0
	arm_pose = 0.0
	hand_pose = 0.0

	base = 0.0
	elbow = 0.0
	#wrist = 0.0
	arm = 0.0
	hand = 0.0
	
	try:
		print msg 
		while(1):
			key = getKey()
			#print(key)
			if key in moveBindings.keys():

				if moveBindings[key][0]<0.0 and (base_pose-0.25) >= -3.10:
					base = moveBindings[key][0]
				elif moveBindings[key][0]>0.0 and (base_pose+0.25) <= 2.44:
					base = moveBindings[key][0]
				else:
					base = 0.0

				if moveBindings[key][1]<0.0 and (elbow_pose-0.1) >= -0.38:
					elbow = moveBindings[key][1]
				elif moveBindings[key][1]>0.0 and (elbow_pose+0.1) <= 1.107:
					elbow = moveBindings[key][1]
				else:
					elbow = 0.0

				if moveBindings[key][2]<0.0 and (arm_pose-0.25) >= -2.08:
					arm = moveBindings[key][2]
				elif moveBindings[key][2]>0.0 and (arm_pose+0.25) <= 0:
					arm = moveBindings[key][2]
				else:
					arm = 0.0

				if (key == 'l'):
					hand = 1.1
				elif (key == 'k'):
					hand = 0

			elif (key == '\x03'):
				break

			if key == 'a' or key == 'd':
				pub_base.publish(base_pose + base)
			elif key == 'w' or key == 's':
				pub_elbow.publish(elbow_pose + elbow)
			elif key == ',' or key == '.':
				pub_arm.publish(arm_pose + arm)
			elif key == 'l' or key == 'k':
				pub_hand.publish(hand)


	except:
		print e

	finally:
		pub_base.publish(0.0)
		pub_elbow.publish(0.0)
		pub_arm.publish(0.0)
		pub_hand.publish(0.0)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


