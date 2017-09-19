#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi

def main(args):

    rospy.init_node('GummiArm', anonymous=False)
    # anonymous=False because doesn't look like a good idea two GummiArm nodes...
    r = rospy.Rate(60)
    # Tried 100 and rostopic hz said it was working!
    # With 600 the elbow PID controller went crazy!

    gummi = Gummi()

    rospy.logwarn('Moving joints sequentially to equilibrium positions.')
    gummi.doGradualStartup()

    rospy.logwarn('Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)
    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()

    #for i in range(0,100):
    #    gummi.forearmRoll.servoTo(pi/2)
    #    r.sleep()

    gummi.setCollisionResponses(shoulder_yaw=False, shoulder_roll=False, shoulder_pitch=False, elbow=False)
    rospy.loginfo("GummiArm is live!")

    while not rospy.is_shutdown():
        try:
            rospy.has_param("/dynamixel_manager_arm/namespace")
            # rospy.has_param("/dynamixel_manager_arm/namespace") is a way to verify if the manager is running

            if gummi.teleop == 0 and gummi.velocity_control == 0:
                gummi.doUpdate()

            gummi.publishJointState()
            r.sleep()
        except:
            rospy.signal_shutdown("gummi_dynamixel manager seems to be dead... exiting!")


if __name__ == '__main__':
    main(sys.argv)
