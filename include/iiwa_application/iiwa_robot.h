#ifndef IIWA_ROBOT
#define IIWA_ROBOT

#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/CartesianPose.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include <vector>

class iiwa_robot{
private:
    iiwa_msgs::JointVelocity current_joint_velocity;
    iiwa_msgs::JointPosition current_joint_position;
    iiwa_msgs::CartesianPose current_cartesion_pose;
    bool got_vel=false,got_pos=false;

public:
    iiwa_robot(){}

    void velocity_callback(const iiwa_msgs::JointVelocity &msg){
        current_joint_velocity=msg;
    }

    void position_callback(const iiwa_msgs::JointPosition &msg){
        current_joint_position=msg;
    }

    void pose_callback(const iiwa_msgs::CartesianPose &msg){
        current_cartesion_pose=msg;
    }

    std::vector<double> get_position_vector(){
        std::vector<double> position(7,0);
        position[0]=current_joint_position.position.a1;
        position[1]=current_joint_position.position.a2;
        position[2]=current_joint_position.position.a3;
        position[3]=current_joint_position.position.a4;
        position[4]=current_joint_position.position.a5;
        position[5]=current_joint_position.position.a6;
        position[6]=current_joint_position.position.a7;

        return position;
    }

    std::vector<double> get_velocity_vector(){
        std::vector<double> velocity(7,0);
        velocity[0]=current_joint_velocity.velocity.a1;
        velocity[1]=current_joint_velocity.velocity.a2;
        velocity[2]=current_joint_velocity.velocity.a3;
        velocity[3]=current_joint_velocity.velocity.a4;
        velocity[4]=current_joint_velocity.velocity.a5;
        velocity[5]=current_joint_velocity.velocity.a6;
        velocity[6]=current_joint_velocity.velocity.a7;
        
        return velocity;
    }

    iiwa_msgs::CartesianPose get_cartesion_pose(){
        return current_cartesion_pose;
    }

    std::vector<double> inverse_kinematics(cv::Mat RPY_t,double Phi){
        if(!got_pos){
            ROS_ERROR("no valid joint position yet");
            return std::vector<double>();
            
        }
        std::vector<double> PreTheta=get_position_vector();

        std::vector<double> next_Theta(7);
        double next_t[7];
        double pre_t[7];
        for(int i=0;i<7;i++){
            pre_t[i]=PreTheta[i];
        }
        Inv_Kine(next_t,RPY_t,Phi,pre_t,7);
        for(int i=0;i<7;i++){
            next_Theta[i]=next_t[i];
        }
        return next_Theta;
        
    }

};

void inverse_kinematics(std::vector<double> &next_Theta,cv::Mat RPY_t,double Phi,std::vector<double> PreTheta){
    if(next_Theta.size()!=7 || PreTheta.size()!=7){
        ROS_ERROR("invalid input");
    }
    double next_t[7];
    double pre_t[7];
    for(int i=0;i<7;i++){
        pre_t[i]=PreTheta[i];
    }
    Inv_Kine(next_t,RPY_t,Phi,pre_t,7);
    for(int i=0;i<7;i++){
        next_Theta[i]=next_t[i];
    }
    
}

std::vector<double> inverse_kinematics(cv::Mat RPY_t,double Phi,std::vector<double> PreTheta){
    if(PreTheta.size()!=7){
        ROS_ERROR("invalid input");
    }
    std::vector<double> next_Theta(7);
    double next_t[7];
    double pre_t[7];
    for(int i=0;i<7;i++){
        pre_t[i]=PreTheta[i];
    }
    Inv_Kine(next_t,RPY_t,Phi,pre_t,7);
    for(int i=0;i<7;i++){
        next_Theta[i]=next_t[i];
    }
    return next_Theta;
    
}


#endif