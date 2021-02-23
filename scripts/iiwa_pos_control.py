#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointPosition,JointVelocity
import math

def call_back(msg):
    with open('vel.txt','a') as f:
        f.write(str(msg.velocity.a4)+'\r\n')

def call_back_pos(msg):
    with open('pos.txt','a') as f:
        f.write(str(msg.position.a4)+'\r\n')

if __name__ == '__main__':
    rospy.init_node('iiwa_pos',anonymous=True)
    pos_pub=rospy.Publisher('/iiwa/command/JointPosition',JointPosition,queue_size=10)
    vel_sub=rospy.Subscriber('iiwa/state/JointVelocity',JointVelocity,call_back)
    pos_sub=rospy.Subscriber('iiwa/state/JointPosition',JointPosition,call_back_pos)

    with open('vel.txt','w') as f:
        f.write('\r\n')
    with open('pos.txt','w') as f:
        f.write('\r\n')


    rospy.loginfo('node initialized!,start position control')

    hz=10000
    rate=rospy.Rate(hz)
    duration=10
    count=0

    while not rospy.is_shutdown():
        
        pos=JointPosition()
        count+=1
        
        if count < duration*hz:
            
            pos.position.a4=count*0.537835180759/100000
            #pos.position.a4=0.537835180759-count*0.537835180759/100000
        elif count ==  duration*hz:
            exit(0)
        
        pos_pub.publish(pos)
        
        rate.sleep()    
