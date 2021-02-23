#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include <iostream>
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/CartesianWrench.h"
#include "iiwa_msgs/CartesianPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "iiwa_msgs/CartesianVelocity.h"
#include "geometry_msgs/TwistStamped.h"

using namespace std;


double CurrentJointPosition[7];

void callback(const iiwa_msgs::JointPosition& Position){
    CurrentJointPosition[1]=Position.position.a1;
    CurrentJointPosition[2]=Position.position.a2;
    CurrentJointPosition[3]=Position.position.a3;
    CurrentJointPosition[4]=Position.position.a4;
    CurrentJointPosition[5]=Position.position.a5;
    CurrentJointPosition[6]=Position.position.a6;
    CurrentJointPosition[7]=Position.position.a7;
    std::cout<<"current joint position "<<Position.position.a1<<" "<<Position.position.a2<<" "
    <<Position.position.a3<<" "<<Position.position.a4<<" "<<Position.position.a5<<" "<<
    Position.position.a6<<" "<<Position.position.a7<<std::endl;
    
}

iiwa_msgs::CartesianWrench data;

void callback_2(const iiwa_msgs::CartesianWrench& msg){
    data.wrench=msg.wrench;
    
}


iiwa_msgs::CartesianPose current_pose;

void callback_3(const iiwa_msgs::CartesianPose& msg){
    current_pose.poseStamped = msg.poseStamped;

    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "JointPositionControlNode");
    ros::NodeHandle nh;
    ros::Subscriber JointPositionSub = nh.subscribe("/iiwa/state/CartesianPose", 1000, callback_3);
    //ros::Publisher poseCommandPub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPoseLin", 1000);
    //ros::Publisher poseCommandPub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
    ros::Publisher poseCommandPub = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/command/CartesianVelocity", 1000);


    ros::Rate loop_rate(10);

    int count=0;

    while(ros::ok()){
        count +=1;
        cout<<count<<endl;
        if(count >5){
            /*
            geometry_msgs::PoseStamped pose_command;
            cout<<'current_pose.poseStamped.pose'<<current_pose.poseStamped.pose<<endl;
            pose_command.pose.position=current_pose.poseStamped.pose.position;
            pose_command.pose.orientation = current_pose.poseStamped.pose.orientation;
            pose_command.pose.position.z+=0.01;
            */
            /*
            iiwa_msgs::CartesianPose pose_command;
            pose_command.poseStamped.pose.position=current_pose.poseStamped.pose.position;
            pose_command.poseStamped.pose.orientation=current_pose.poseStamped.pose.orientation;
            pose_command.poseStamped.pose.position.z +=0.05;
            pose_command.redundancy.e1=0;
            pose_command.redundancy.status=-1;
            pose_command.redundancy.turn = -1;
            */
            geometry_msgs::TwistStamped pose_command;
            pose_command.twist.linear.z=0.04;
            poseCommandPub.publish(pose_command);
            cout<<"publishing"<<endl;
        }

        //std::cout<<data.wrench.force.x<< " " <<data.wrench.force.y<<" "<<data.wrench.force.z<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
        if(count >20){
            return 0;
        }

    }
    return 0;
}
/*
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "JointPositionControlNode");
    ros::NodeHandle nh;

    #ifdef JOINTPOSITION
    ros::Publisher JointCommandPub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);
    #endif
    #ifdef JOINTVELOCITY
    ros::Publisher JointCommandPub = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);
    #endif
    ros::Subscriber JointPositionSub = nh.subscribe("/iiwa/state/JointPosition", 1000, callback);

    ros::Rate loop_rate(100);
    int count=0;

    while (ros::ok())
    {
        count+=1;
        #ifdef JOINTPOSITION
        iiwa_msgs::JointPosition JointCommand;
        JointCommand.position.a1=CurrentJointPosition[1];
        JointCommand.position.a2=CurrentJointPosition[2];
        JointCommand.position.a3=CurrentJointPosition[3];
        JointCommand.position.a4=CurrentJointPosition[4];
        JointCommand.position.a5=CurrentJointPosition[5];
        JointCommand.position.a6=CurrentJointPosition[6];
        JointCommand.position.a7=CurrentJointPosition[7];
        if(count<300){
            JointCommand.position.a7 = 0;
        }
        else if(count<600){
            JointCommand.position.a6=0;
        }
        else if(count<900){
            JointCommand.position.a5=0;
        }
        else if(count<1200){
            JointCommand.position.a4=0;
        }
        else if(count<1500){
            JointCommand.position.a3=0;
        }
        else if(count<1800){
            JointCommand.position.a2=0;
        }
        else if(count<2100){
            JointCommand.position.a1=0;
        }
        else{
            break;
        }
        JointCommandPub.publish(JointCommand);
        ros::spinOnce();
        loop_rate.sleep();
        #endif

        #ifdef JOINTVELOCITY
        iiwa_msgs::JointVelocity JointCommand;
        JointCommand.velocity.a1=0;
        JointCommand.velocity.a2=0;
        JointCommand.velocity.a3=0;
        JointCommand.velocity.a4=0;
        JointCommand.velocity.a5=0;
        JointCommand.velocity.a6=0;
        JointCommand.velocity.a7=0;

        if(count<500){
            JointCommand.velocity.a7 = 0.02;
        }
        else if(count<1000){
            JointCommand.velocity.a6=0.02;
        }
        else if(count<1500){
            JointCommand.velocity.a5=0.02;
        }
        else if(count<2000){
            JointCommand.velocity.a4=0.02;
        }
        else if(count<2500){
            JointCommand.velocity.a3=0.02;
        }
        else if(count<3000){
            JointCommand.velocity.a2=0.02;
        }
        else if(count<3500){
            JointCommand.velocity.a1=0.02;
        }
        
        JointCommandPub.publish(JointCommand);
        ros::spinOnce();
        loop_rate.sleep();
        if(count > 3510){
            break;
        }
        #endif
    }
    



    return 0;
}*/