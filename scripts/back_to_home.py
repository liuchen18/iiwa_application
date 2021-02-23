#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointPosition,JointVelocity
import math

if __name__ == '__main__':
    rospy.init_node('iiwa_pos',anonymous=True)
    pos_pub=rospy.Publisher('/iiwa/command/JointPosition',JointPosition,queue_size=10)

    rospy.loginfo('node initialized!,start position control')

    hz=100
    rate=rospy.Rate(hz)
    duration=1
    count=0

    while not rospy.is_shutdown():
        
        pos=JointPosition()
        pos.position.a1=-0.0
        pos.position.a2=-0.49668
        pos.position.a3=0
        pos.position.a4=-1.87076
        pos.position.a5=0
        pos.position.a6=-1.3763
        pos.position.a7=-0.0
        count+=1
        pos_pub.publish(pos)
        if count == hz*duration:
            exit(0)
        
        rate.sleep()    
