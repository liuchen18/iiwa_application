#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointPositionVelocity
import math

def call_back(msg):
    with open('posvel.txt','a') as f:
        f.write(str(msg.position.a5)+' '+str(msg.velocity.a5)+' ' + \
            str(msg.position.a4)+' '+str(msg.velocity.a4)+' '+ \
            str(msg.position.a2)+' '+str(msg.velocity.a2)+' '+'\r\n')

def compute_pos_vel(count,total):
    pos_vel=JointPositionVelocity()
    t=count*1.0/total*2
    pos_vel.header.stamp=rospy.get_rostime()
    p=0.1*(t-2/math.pi*math.cos(math.pi/2*(t-1)))
    v=0.1*(1+math.sin(math.pi/2*(t-1)))/5
    pos_vel.position.a5=p
    pos_vel.velocity.a5=v
    pos_vel.position.a4=p
    pos_vel.velocity.a4=v
    pos_vel.position.a2=p
    pos_vel.velocity.a2=v
    #print(pos_vel.position.a4)
    return pos_vel

if __name__ == '__main__':
    rospy.init_node('iiwa_vel',anonymous=True)
    pos_pub=rospy.Publisher('/iiwa/command/JointPositionVelocity',JointPositionVelocity,queue_size=10)
    sub=rospy.Subscriber('iiwa/state/JointPositionVelocity',JointPositionVelocity,call_back)

    with open('posvel.txt','w') as f:
        f.write('\r\n')
    with open('posvel_d.txt','w') as f:
        f.write('\r\n')

    rospy.loginfo('node initialized!,start position control')

    hz=200
    rate=rospy.Rate(hz)
    duration=10
    count=0

    while not rospy.is_shutdown():
        count+=1
        posvel=compute_pos_vel(count,duration*hz)
        
        if count ==  duration*hz:
            exit(0)
        with open('posvel_d.txt','a') as f:
            f.write(str(posvel.position.a5)+' '+str(posvel.velocity.a5)+'\r\n')
        pos_pub.publish(posvel)

        rate.sleep()    





