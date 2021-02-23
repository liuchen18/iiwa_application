#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointPositionVelocity
import math

def call_back(msg):
    with open('states.txt','a') as f:
        f.write(str(msg.position.a1)+' '+str(msg.velocity.a1)+' ' + \
            str(msg.position.a2)+' '+str(msg.velocity.a2)+' '+ \
            str(msg.position.a3)+' '+str(msg.velocity.a3)+' '+\
            str(msg.position.a4)+' '+str(msg.velocity.a4)+' ' + \
            str(msg.position.a5)+' '+str(msg.velocity.a5)+' '+ \
            str(msg.position.a6)+' '+str(msg.velocity.a6)+' '+ \
            str(msg.position.a7)+' '+str(msg.velocity.a7)+' ' + '\r\n')


if __name__ == '__main__':
    rospy.init_node('iiwa_sub',anonymous=True)
    sub=rospy.Subscriber('iiwa/state/JointPositionVelocity',JointPositionVelocity,call_back)

    with open('states.txt','w') as f:
        f.write('\r\n')

    hz=10
    rate=rospy.Rate(hz)
    duration=20
    count=0

    while not rospy.is_shutdown():
        count+=1     
        if count ==  duration*hz:
            exit(0)

        rate.sleep()    





