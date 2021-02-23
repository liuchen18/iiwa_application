#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointVelocity,JointPosition
import math

def call_back(msg):
    #print(msg.velocity.a1)
    with open('vel.txt','a') as f:
        f.write(str(msg.velocity.a4)+'\r\n')

def call_back_pos(msg):
    with open('pos.txt','a') as f:
        f.write(str(msg.position.a4)+'\r\n')

def compute_vel(cur,goal,count,total_num):
    #sigmoid
    sx=cur+(goal-cur)/(1.0+math.pow(math.e,-5*(2*(count*1.0+1)-total_num)/total_num))
    return sx
'''
def compute_vel(count,total):
    vel=JointVelocity()
    t=count*1.0/total*2
    vel.header.stamp=rospy.get_rostime()
    vel.velocity.a4=0.1*(1+math.sin(math.pi/2*(t-1)))
    #print(pos_vel.position.a4)
    return vel
'''
if __name__ == '__main__':
    rospy.init_node('iiwa_vel',anonymous=True)
    vel_pub=rospy.Publisher('/iiwa/command/JointVelocity',JointVelocity,queue_size=10)
    vel_sub=rospy.Subscriber('iiwa/state/JointVelocity',JointVelocity,call_back)
    pos_sub=rospy.Subscriber('iiwa/state/JointPosition',JointPosition,call_back_pos)

    with open('vel.txt','w') as f:
        f.write('\r\n')

    with open('pos.txt','w') as f:
        f.write('\r\n')

    with open('vel_d.txt','w') as f:
        f.write('\r\n')

    rospy.loginfo('node initialized!,start velocity control')

    hz=10000
    rate=rospy.Rate(hz)

    max_vel=0.10
    duration=10
    count=0

    while not rospy.is_shutdown():
        count+=1
        
        vel=JointVelocity()

        count+=1
        if count < duration/2*hz:
            vel.velocity.a4=compute_vel(0.0,1*max_vel,count,hz*duration/2)
        elif count <  duration*hz:
            vel.velocity.a4=compute_vel(1*max_vel,0.0,count-hz*duration/2,hz*duration/2)
        elif count ==  duration*hz:
            rospy.loginfo('vel is set to zero')
            exit(0)
        '''
        vel=compute_vel(count,hz*duration)
        with open('vel_d.txt','a') as f:
            f.write(str(vel.velocity.a4)+'\r\n')
        vel_pub.publish(vel)
        if count == hz*duration:
            exit(0)
        '''
        vel_pub.publish(vel)
        rate.sleep()    
