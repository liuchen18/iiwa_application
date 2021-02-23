#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointVelocity,JointPosition
import math

def call_back(msg):
    #print(msg.velocity.a1)
    with open('vel.txt','a') as f:
        f.write(str(msg.velocity.a4)+'\r\n')

def call_back_command(msg):
    got_msg=True
    with open('pos_command.txt','a') as f:
        #rospy.loginfo(msg)
        #print(msg.header.stamp.secs*1.0%100+msg.header.stamp.nsecs*1.0/1000000000)
        f.write(str(msg.header.stamp.secs*1.0%1000+msg.header.stamp.nsecs*1.0/1000000000)+' '+str(msg.position.a4)+'\r\n')


def call_back_state(msg):
    if got_msg == True:
            
        with open('pos_state.txt','a') as f:
            #rospy.loginfo(msg)
            #print(msg.header.stamp.secs*1.0%100+msg.header.stamp.nsecs*1.0/1000000000)
            f.write(str(msg.header.stamp.secs*1.0%1000+msg.header.stamp.nsecs*1.0/1000000000)+' '+str(msg.position.a4)+'\r\n')


if __name__ == '__main__':
    got_msg=False
    rospy.init_node('iiwa',anonymous=True)
    pos_c_sub=rospy.Subscriber('iiwa/command/JointPosition',JointPosition,call_back_command)
    pos_s_sub=rospy.Subscriber('iiwa/state/JointPosition',JointPosition,call_back_state)

    with open('pos_command.txt','w') as f:
        f.write('\r\n')
    with open('pos_state.txt','w') as f:
        f.write('\r\n')

    hz=10
    rate=rospy.Rate(hz)

    while not rospy.is_shutdown():
        rate.sleep()    
