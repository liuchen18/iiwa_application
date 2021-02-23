#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include <iostream>
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/CartesianWrench.h"
#include "7_DOF_inv.h"

//#define JOINTVELOCITY
#define JOINTPOSITION


double CurrentJointPosition[7];

void callback(const iiwa_msgs::JointPosition& Position){
    CurrentJointPosition[0]=Position.position.a1;
    CurrentJointPosition[1]=Position.position.a2;
    CurrentJointPosition[2]=Position.position.a3;
    CurrentJointPosition[3]=Position.position.a4;
    CurrentJointPosition[4]=Position.position.a5;
    CurrentJointPosition[5]=Position.position.a6;
    CurrentJointPosition[6]=Position.position.a7;
    //std::cout<<"current joint position "<<Position.position.a1<<" "<<Position.position.a2<<" "
    //<<Position.position.a3<<" "<<Position.position.a4<<" "<<Position.position.a5<<" "<<
    //Position.position.a6<<" "<<Position.position.a7<<std::endl;
    
}

iiwa_msgs::CartesianWrench data;

void callback_2(const iiwa_msgs::CartesianWrench& msg){
    data.wrench=msg.wrench;
    
}
/*int main(int argc, char *argv[])
{
    ros::init(argc, argv, "JointPositionControlNode");
    ros::NodeHandle nh;
    ros::Subscriber JointPositionSub = nh.subscribe("/iiwa/state/JointPosition", 1000, callback);

    ros::Rate loop_rate(100);


    while(ros::ok()){
        std::cout<<CurrentJointPosition[0]<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}*/

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
        if(count<=100){
            
        }
        else if(count<=600){
            /*
            JointCommand.position.a1 = CurrentJointPosition[1]+(PI/2-CurrentJointPosition[1])*(count-100)/500;
            JointCommand.position.a2 = CurrentJointPosition[2]+(PI/4.5-CurrentJointPosition[2])*(count-100)/500;
            JointCommand.position.a3 = CurrentJointPosition[3]+(0-CurrentJointPosition[3])*(count-100)/500;
            JointCommand.position.a4 = CurrentJointPosition[4]+(-PI/2.68657-CurrentJointPosition[4])*(count-100)/500;
            JointCommand.position.a5 = CurrentJointPosition[5]+(0-CurrentJointPosition[5])*(count-100)/500;
            JointCommand.position.a6 = CurrentJointPosition[6]+(PI/2.46575-CurrentJointPosition[6])*(count-100)/500;
            JointCommand.position.a7 = CurrentJointPosition[7]+(0-CurrentJointPosition[7])*(count-100)/500;*/
            
            /*
            JointCommand.position.a1 = PI/2;
            JointCommand.position.a2 = PI/4.5;
            JointCommand.position.a3 = 0;
            JointCommand.position.a4 = -PI/2.9032;
            JointCommand.position.a5 = 0;
            JointCommand.position.a6 = PI/2.3077;
            JointCommand.position.a7 = -PI/2;
            JointCommandPub.publish(JointCommand);
            */
            JointCommand.position.a1 = 0;
            JointCommand.position.a2 = 0;
            JointCommand.position.a3 = 0;
            JointCommand.position.a4 = 0;
            JointCommand.position.a5 = 0;
            JointCommand.position.a6 = 0;
            JointCommand.position.a7 = 0;
            JointCommandPub.publish(JointCommand);

        }
        else{
            break;
        }
        
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
}