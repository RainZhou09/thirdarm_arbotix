#!/usr/bin/env python  
import rospy
import roslib
import math
import arbotix_msgs.srv
from random import gauss
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from hlpr_speech_msgs.msg import StampedString, SpeechCommand
from sensor_msgs.msg import JointState

def callbackPose(msg): 
    global base_pose, elbow_pose, wrist_pose, arm_pose, hand_pose
    elbow_pose = msg.position[0]
    base_pose = msg.position[1]
    wrist_pose = msg.position[2]
    arm_pose = msg.position[3]
    hand_pose = msg.position[4]

def callback(msg):
    global last_msg
    last_msg = msg.data

if __name__ == '__main__':

    # re-declare global variales and initialize them
    global base_pose, elbow_pose, wrist_pose, arm_pose, hand_pose
    global last_msg
    base_pose = 0.0
    elbow_pose = 0.0
    wrist_pose = 0.0
    arm_pose = 0.0
    hand_pose = 0.0
    last_msg = "OFF"

    rospy.init_node('command_listener')

    # sub to speech command
    rospy.Subscriber('command', String, callback)

    # sub to current servo positions
    rospy.Subscriber('/joint_states', JointState, callbackPose)

    # set up set_speed service
    elbow_speed = rospy.ServiceProxy('/elbow/set_speed', arbotix_msgs.srv.SetSpeed)
    elbow_speed(0.157)

    # pub to arm to move it
    # need to launch arbotix_driver, pub to servo/command, type Float64
    pub_base = rospy.Publisher('/base/command', Float64, queue_size=1)
    pub_elbow = rospy.Publisher('/elbow/command', Float64, queue_size=1)
    pub_arm = rospy.Publisher('/arm/command', Float64, queue_size=1)
    pub_wrist = rospy.Publisher('/wrist/command', Float64, queue_size=1)
    pub_hand = rospy.Publisher('/hand/command', Float64, queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # print (elbow_pose, base_pose, wrist_pose, arm_pose, hand_pose)

        print "Right now I'm doing", last_msg
        if last_msg == "LEFT":
            if (base_pose-0.775) >= -3.10:
                pub_base.publish(base_pose - 0.775)

            last_msg = "OFF"

        elif last_msg == "RIGHT":
            if (base_pose+0.775) <= 2.44:
                pub_base.publish(base_pose + 0.775)

            last_msg = "OFF"

        elif last_msg == "UP":
            if (elbow_pose-0.35) >= -0.38:
                pub_elbow.publish(elbow_pose - 0.35)

            last_msg = "OFF"

        elif last_msg == "DOWN":
            if (elbow_pose+0.35) <= 1.107:
                pub_elbow.publish(elbow_pose + 0.35)

            last_msg = "OFF"

        elif last_msg == "EXTEND":
            if (arm_pose-1.00) >= -2.08:
                pub_arm.publish(arm_pose - 1.00)

            last_msg = "OFF"

        elif last_msg == "RETRACT":
            pub_arm.publish(0.0)

            last_msg = "OFF"

        elif last_msg == "GRAB":
            pub_hand.publish(1.1)

            last_msg = "OFF"

        elif last_msg == "RELEASE":
            pub_hand.publish(0.0)

            last_msg = "OFF"

        elif last_msg == "HOME":
            pub_base.publish(0.0)
            pub_elbow.publish(0.0)
            pub_arm.publish(0.0)
            pub_wrist.publish(0.0)
            pub_hand.publish(0.0)

        elif last_msg == "OFF":
            # all motors stay at the same position
            pass

        rate.sleep()
