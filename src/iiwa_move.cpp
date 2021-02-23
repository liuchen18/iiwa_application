#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/CartesianPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "iiwa_msgs/JointVelocity.h"
#include "7_DOF_inv.h"
#include <math.h>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include "std_msgs/Bool.h"

iiwa_msgs::JointPosition current_joint_position;
iiwa_msgs::JointVelocity current_joint_velocity;
iiwa_msgs::CartesianPose current_cartesian_pose;
bool got_position=false;
bool start_move=false;


std::vector<double> copy_joint_position(iiwa_msgs::JointPosition p){
    std::vector<double> PreTheta(7,0);
    PreTheta[0]=p.position.a1;
    PreTheta[1]=p.position.a2;
    PreTheta[2]=p.position.a3;
    PreTheta[3]=p.position.a4;
    PreTheta[4]=p.position.a5;
    PreTheta[5]=p.position.a6;
    PreTheta[6]=p.position.a7;
    return PreTheta;
}

void quaternion2rpy(geometry_msgs::Quaternion q,cv::Mat& RPY){
    double r=atan2(2*(q.w*q.x+q.y*q.z),1-2*(pow(q.x,2)+pow(q.y,2)));
    double p=asin(2*(q.w*q.y-q.x*q.z));
    double y=atan2(2*(q.w*q.z+q.x*q.y),1-2*(pow(q.y,2)+pow(q.z,2)));
    RPY.ptr<float>(0)[3] = r;
    RPY.ptr<float>(0)[4] = p;
    RPY.ptr<float>(0)[5] = y;
}

iiwa_msgs::JointPosition compute_joint_command(int count,
                                                std::vector<double> next_Theta, 
                                                std::vector<double> PreTheta){
    iiwa_msgs::JointPosition p;
    p.position.a1=PreTheta[0]+(next_Theta[0]-PreTheta[0])/400*(count%400);
    p.position.a2=PreTheta[1]+(next_Theta[1]-PreTheta[1])/400*(count%400);
    p.position.a3=PreTheta[2]+(next_Theta[2]-PreTheta[2])/400*(count%400);
    p.position.a4=PreTheta[3]+(next_Theta[3]-PreTheta[3])/400*(count%400);
    p.position.a5=PreTheta[4]+(next_Theta[4]-PreTheta[4])/400*(count%400);
    p.position.a6=PreTheta[5]+(next_Theta[5]-PreTheta[5])/400*(count%400);
    p.position.a7=PreTheta[6]+(next_Theta[6]-PreTheta[6])/400*(count%400);
    return p;
}

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

void generate_orientation(int count, Mat& RPY){
    geometry_msgs::Point q;
    
    int path_point=count/100;
    double fuzhi=6.0*PI/180;

    q.x=0;
    q.y=-fuzhi*cos(path_point*1.0/40*2*PI);
    q.z=-fuzhi*sin(path_point*1.0/40*2*PI);

    RPY.ptr<float>(0)[3] = q.x;
    RPY.ptr<float>(0)[4] = q.y;
    RPY.ptr<float>(0)[5] = q.z;
}

geometry_msgs::Point get_target_position(int count){
    double init_cartesian_pose[3]={183.19,11.22,1013};
    std::vector<double> init_joint_position(7,0);
    init_joint_position[0]=-0.05475;
    init_joint_position[1]=-0.46492;
    init_joint_position[2]=0;
    init_joint_position[3]=-1.85293;
    init_joint_position[4]=-0.00574;
    init_joint_position[5]=-1.49258;
    init_joint_position[6]=-0.054;
    int path_point=count/100;

    double fuzhi=100.0;
    double factor=1.0;
    if(path_point<120){
        factor=factor*(1-0.6*(path_point*1.0/120));
    }
    else{
        factor=0.4;
    }


    geometry_msgs::Point p;
    p.x=init_cartesian_pose[0];
    p.y=init_cartesian_pose[1] + factor*fuzhi*sin(path_point*1.0/40*2*PI);
    p.z=init_cartesian_pose[2] + factor*fuzhi*(1-cos(path_point*1.0/40*2*PI));
    /*
    p.position.y=init_cartesian_pose[1];
    if(path_point<11){
        p.position.z=init_cartesian_pose[2]+50;
    }
    else{
        p.position.z=init_cartesian_pose[2];
    }
    */
    /*
    if(path_point<11){
        p.position.z=init_cartesian_pose[2]+5*path_point;
    }
    else if(path_point<22){
        p.position.z=init_cartesian_pose[2]+50-5*(path_point-10);
    }
    else{
        exit(0);
    }
    */
    return p;
}

void callback(const iiwa_msgs::JointPosition& Position){
    current_joint_position=Position;
    if(Position.position.a1!=0){
        got_position=true;
    }
    
}

void callback_cartesian(const iiwa_msgs::CartesianPose& Pose){
    current_cartesian_pose=Pose;
    //std::cout<<"cartesion pose z: "<<Pose.poseStamped.pose.position.z<<std::endl;
}

void start_callback(const std_msgs::Bool& start_flag){
    start_move=start_flag.data;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_trajectory");
    ros::NodeHandle nh;


    ros::Publisher JointPositionCommandPub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);
    ros::Publisher CartesionCommandPub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
    ros::Publisher JointvelocityPub = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);
    ros::Publisher posvel_pub = nh.advertise<iiwa_msgs::JointPositionVelocity>("/iiwa/command/JointPositionVelocity", 1000);
    

    ros::Subscriber JointPositionSub = nh.subscribe("/iiwa/state/JointPosition", 1000, callback);
    ros::Subscriber CartesionPoseSub = nh.subscribe("/iiwa/state/CartesianPose", 1000, callback_cartesian);
    ros::Subscriber start_sub=nh.subscribe("/iiwa_start",1000,start_callback);

    ros::Rate loop_rate(100);
    int count=0;

    std::vector<double> next_Theta(7,0);
    std::vector<double> PreTheta(7,0);
    ROS_INFO("starting the main loop");

    while (ros::ok())
    {
        
        if(got_position==true && start_move==true){
            if(count%100==0){
                cv::Mat RPY_t=cv::Mat::zeros(6,1,CV_32FC1);
                geometry_msgs::Point target_position=get_target_position(count);
                generate_orientation(count,RPY_t);
                //quaternion2rpy(target_pose.orientation,RPY_t);
                RPY_t.ptr<float>(0)[0] = target_position.x;
                RPY_t.ptr<float>(0)[1] = target_position.y;
                RPY_t.ptr<float>(0)[2] = target_position.z;
                
                cout<<"time: "<<count/100<<endl;
                std::cout<<"TARGET POSE: ";
                for (int i = 0; i<6; i++){
                    std::cout<<RPY_t.ptr<float>(0)[i]<<" ";
                }
                std::cout<<endl;
                
                //cout<<"delta target pose: "<<target_pose.position.z-pre_target_pose.position.z<<endl;

                double Phi = 0;
                PreTheta=copy_joint_position(current_joint_position);
                //ROS_INFO("start inverse kinematics");
                inverse_kinematics(next_Theta,RPY_t,Phi,PreTheta);
                
                std::cout<<"TARGET theta: ";
                for (int i = 0; i<7; i++){
                    std::cout<<next_Theta[i]<<" ";
                }
                std::cout<<endl;
                
            }
            
            iiwa_msgs::JointPosition JointCommand;
            JointCommand.position.a1=next_Theta[0];
            JointCommand.position.a2=next_Theta[1];
            JointCommand.position.a3=next_Theta[2];
            JointCommand.position.a4=next_Theta[3];
            JointCommand.position.a5=next_Theta[4];
            JointCommand.position.a6=next_Theta[5];
            JointCommand.position.a7=next_Theta[6];
            
            JointPositionCommandPub.publish(JointCommand);
            
            /*
            if(PreTheta[0]!=0){
                iiwa_msgs::JointPositionVelocity pv_command;
                pv_command.position.a1=next_Theta[0];
                pv_command.position.a2=next_Theta[1];
                pv_command.position.a3=next_Theta[2];
                pv_command.position.a4=next_Theta[3];
                pv_command.position.a5=next_Theta[4];
                pv_command.position.a6=next_Theta[5];
                pv_command.position.a7=next_Theta[6];
                pv_command.velocity.a1=(next_Theta[0]-PreTheta[0])/10;
                pv_command.velocity.a2=(next_Theta[1]-PreTheta[1])/10;
                pv_command.velocity.a3=(next_Theta[2]-PreTheta[2])/10;
                pv_command.velocity.a4=(next_Theta[3]-PreTheta[3])/10;
                pv_command.velocity.a5=(next_Theta[4]-PreTheta[4])/10;
                pv_command.velocity.a6=(next_Theta[5]-PreTheta[5])/10;
                pv_command.velocity.a7=(next_Theta[6]-PreTheta[6])/10;
                //cout<<"a4 command: pos: "<<pv_command.position.a4<<" vel: "<<pv_command.velocity.a4<<endl;
                posvel_pub.publish(pv_command);
            }
            */
            

            /*
            iiwa_msgs::JointVelocity jointvel_command;
            jointvel_command.velocity.a1=next_Theta[0]-PreTheta[0];
            jointvel_command.velocity.a2=next_Theta[1]-PreTheta[1];
            jointvel_command.velocity.a3=next_Theta[2]-PreTheta[2];
            jointvel_command.velocity.a4=next_Theta[3]-PreTheta[3];
            jointvel_command.velocity.a5=next_Theta[4]-PreTheta[4];
            jointvel_command.velocity.a6=next_Theta[5]-PreTheta[5];
            jointvel_command.velocity.a7=next_Theta[6]-PreTheta[6];
            JointvelocityPub.publish(jointvel_command);
            */

            /*iiwa_msgs::JointPosition JointCommand
                =compute_joint_command(count,next_Theta,PreTheta);
            */
            /*
            iiwa_msgs::JointPosition JointCommand;
            JointCommand.position.a1=0.0612;
            JointCommand.position.a2=-0.4861;
            JointCommand.position.a3=0;
            JointCommand.position.a4=-1.7372;
            JointCommand.position.a5=0;
            JointCommand.position.a6=-1.2512;
            JointCommand.position.a7=-0.0612;
            */
            //JointPositionCommandPub.publish(JointCommand);



            
            count+=1;
        }    


        ros::spinOnce();
        loop_rate.sleep();
    }
    



    return 0;
}