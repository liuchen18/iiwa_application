#!/usr/bin/env python

import roslibpy
import time
import rospy
from iiwa_msgs.msg import JointPosition,JointVelocity
from geometry_msgs.msg import PoseStamped

class communication_class():
    def __init__(self):
        self.roslibpy_client=roslibpy.Ros(host='192.168.2.76',port=9099)
        self.roslibpy_client.run()
        self.pos_sender=roslibpy.Topic(self.roslibpy_client,'/iiwa_joint_position','iiwa_msgs/JointPosition')
        self.vel_sender=roslibpy.Topic(self.roslibpy_client,'/iiwa_joint_velocity','iiwa_msgs/JointVelocity')
        self.pose_receiver=roslibpy.Topic(self.roslibpy_client,'FX_msg','geometry_msgs/PoseStamped')
        self.pose_receiver.subscribe(self.callback_basepose)

        rospy.init_node('communication_node')
        self.vel_sub=rospy.Subscriber('iiwa/state/JointVelocity',JointVelocity,self.call_back_vel)
        self.pos_sub=rospy.Subscriber('iiwa/state/JointPosition',JointPosition,self.call_back_pos)
        self.base_pose_pub=rospy.Publisher('FX_msg_roslibpy',PoseStamped,queue_size=10)

        self.count=0
        self.base_pose=PoseStamped()

        self.joint_vel=dict()
        self.joint_vel['header']=dict()
        self.joint_vel['header']['seq']=0
        self.joint_vel['header']['frame_id']=''
        self.joint_vel['header']['stamp']=dict()
        self.joint_vel['header']['stamp']['secs']=0
        self.joint_vel['header']['stamp']['nsecs']=0
        self.joint_vel['velocity']=dict()
        self.joint_vel['velocity']['a1']=0
        self.joint_vel['velocity']['a2']=0
        self.joint_vel['velocity']['a3']=0
        self.joint_vel['velocity']['a4']=0
        self.joint_vel['velocity']['a5']=0
        self.joint_vel['velocity']['a6']=0
        self.joint_vel['velocity']['a7']=0

        self.joint_pos=dict()
        self.joint_pos['header']=dict()
        self.joint_pos['header']['seq']=0
        self.joint_pos['header']['frame_id']=''
        self.joint_pos['header']['stamp']=dict()
        self.joint_pos['header']['stamp']['secs']=0
        self.joint_pos['header']['stamp']['nsecs']=0
        self.joint_pos['position']=dict()
        self.joint_pos['position']['a1']=0
        self.joint_pos['position']['a2']=0
        self.joint_pos['position']['a3']=0
        self.joint_pos['position']['a4']=0
        self.joint_pos['position']['a5']=0
        self.joint_pos['position']['a6']=0
        self.joint_pos['position']['a7']=0



    def call_back_vel(self,msg):

        self.joint_vel['header']['seq']=msg.header.seq
        self.joint_vel['header']['frame_id']=msg.header.frame_id
        self.joint_vel['header']['stamp']['secs']=msg.header.stamp.secs
        self.joint_vel['header']['stamp']['nsecs']=msg.header.stamp.nsecs
        self.joint_vel['velocity']['a1']=msg.velocity.a1
        self.joint_vel['velocity']['a2']=msg.velocity.a2
        self.joint_vel['velocity']['a3']=msg.velocity.a3
        self.joint_vel['velocity']['a4']=msg.velocity.a4
        self.joint_vel['velocity']['a5']=msg.velocity.a5
        self.joint_vel['velocity']['a6']=msg.velocity.a6
        self.joint_vel['velocity']['a7']=msg.velocity.a7

    def call_back_pos(self,msg):
        
        self.joint_pos['header']['seq']=msg.header.seq
        self.joint_pos['header']['frame_id']=msg.header.frame_id
        self.joint_pos['header']['stamp']['secs']=msg.header.stamp.secs
        self.joint_pos['header']['stamp']['nsecs']=msg.header.stamp.nsecs
        self.joint_pos['position']['a1']=msg.position.a1
        self.joint_pos['position']['a2']=msg.position.a2
        self.joint_pos['position']['a3']=msg.position.a3
        self.joint_pos['position']['a4']=msg.position.a4
        self.joint_pos['position']['a5']=msg.position.a5
        self.joint_pos['position']['a6']=msg.position.a6
        self.joint_pos['position']['a7']=msg.position.a7
    
    
    def callback_basepose(self,msg):
        self.base_pose.header.frame_id=msg['header']['frame_id']
        self.base_pose.header.seq=msg['header']['seq']
        self.base_pose.header.stamp.secs=msg['header']['stamp']['secs']
        self.base_pose.header.stamp.nsecs=msg['header']['stamp']['nsecs']

        self.base_pose.pose.position.x=msg['pose']['position']['x']
        self.base_pose.pose.position.y=msg['pose']['position']['y']
        self.base_pose.pose.position.z=msg['pose']['position']['z']
        self.base_pose.pose.orientation.x=msg['pose']['orientation']['x']
        self.base_pose.pose.orientation.y=msg['pose']['orientation']['y']
        self.base_pose.pose.orientation.y=msg['pose']['orientation']['z']
        self.base_pose.pose.orientation.w=msg['pose']['orientation']['w']

        self.base_pose_pub.publish(self.base_pose)

    


def main():

    communication_node=communication_class()
    data_sender=roslibpy.Topic(communication_node.roslibpy_client,'/data_test','std_msgs/Float32')


    rate=rospy.Rate(100)
    while not rospy.is_shutdown():
        if communication_node.roslibpy_client.is_connected:
            communication_node.pos_sender.publish(roslibpy.Message(communication_node.joint_pos))
            communication_node.vel_sender.publish(roslibpy.Message(communication_node.joint_vel))
        else:
            rospy.logerr('connot communicate with the ros master')
        data_sender.publish(roslibpy.Message({'data':0.0}))
        rate.sleep()

    communication_node.pos_sender.unadvertise()
    communication_node.vel_sender.unadvertise()
    communication_node.roslibpy_client.terminate()
    

if __name__=='__main__':
    main()