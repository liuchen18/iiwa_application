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

double CurrentCartesianWrench[6];

void callback_wrench(const iiwa_msgs::CartesianWrench& Wrench){
    CurrentCartesianWrench[0]=Wrench.wrench.force.x;
    CurrentCartesianWrench[1]=Wrench.wrench.force.y;
    CurrentCartesianWrench[2]=Wrench.wrench.force.z;
    CurrentCartesianWrench[3]=Wrench.wrench.torque.x;//not sure
    CurrentCartesianWrench[4]=Wrench.wrench.torque.y;
    CurrentCartesianWrench[5]=Wrench.wrench.torque.z;

    //std::cout<<"current cartesian wrench "<<Wrench.wrench.force.x<<" "<<Wrench.wrench.force.y<<" "
    //<<Wrench.wrench.force.z<<" "<<Wrench.wrench.torque.x<<" "<<Wrench.wrench.torque.y<<" "<<
    //Wrench.wrench.torque.z<<std::endl;
    
}


int main(int argc, char *argv[])
{ 
    
    ros::init(argc, argv, "JointPositionControlNode");
    ros::NodeHandle nh;
    
    // define the Subscriber
    ros::Subscriber JointPositionSub = nh.subscribe("/iiwa/state/JointPosition", 1000, callback);
    ros::Subscriber CartesianWrenchSub = nh.subscribe("/iiwa/state/CartesianWrench", 1000, callback_wrench);
    
    //define the Publisher
    #ifdef JOINTPOSITION
    ros::Publisher JointCommandPub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);
    #endif
    #ifdef JOINTVELOCITY
    ros::Publisher JointCommandPub = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);
    #endif

    //wait for ready reading

    //cal initial postion and force
    double Final[4][4]={0};
    double Theta_o[7] = {0};
    for(int i=0;i<7;i++){
            Theta_o[0]=PI/2;
            Theta_o[1]=PI/4.5;
            Theta_o[2]=0;
            Theta_o[3]=-PI/2.68657;
            Theta_o[4]=0;
            Theta_o[5]=PI/2.46575;
            Theta_o[6]=-PI/2;
    }
    For_kine(Final,Theta_o);
    Mat R_o=Mat::zeros(3,3,CV_32FC1);
	for (int m=0;m<3;m++){
		for (int n=0;n<3;n++)
			R_o.ptr<float>(m)[n]=Final[m][n];
	}
	Mat RPY_o=Mat::zeros(3,1,CV_32FC1);
	RPY_o = r2RPY(R_o);
    double TCP_original[6] = {0}; 
    TCP_original[0] = Final[0][3];
    TCP_original[1] = Final[1][3];
    TCP_original[2] = Final[2][3];
    TCP_original[3] = RPY_o.ptr<float>(0)[0];
    TCP_original[4] = RPY_o.ptr<float>(0)[1];
    TCP_original[5] = RPY_o.ptr<float>(0)[2];
    double CurrentCartesianWrench_o[6]={0};
    CurrentCartesianWrench_o[0] = CurrentCartesianWrench[0];
    CurrentCartesianWrench_o[1] = CurrentCartesianWrench[1];
    CurrentCartesianWrench_o[2] = CurrentCartesianWrench[2];
    CurrentCartesianWrench_o[3] = CurrentCartesianWrench[3];
    CurrentCartesianWrench_o[4] = CurrentCartesianWrench[4];
    CurrentCartesianWrench_o[5] = CurrentCartesianWrench[5];
    double PreTheta[7]={0};
    for (int i = 0; i<7; i++){
        PreTheta[i] = Theta_o[i];
    }
    
    ofstream in;
    in.open("com.txt",ios::trunc); //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建

    //control rate
    int fre = 100;
    ros::Rate loop_rate(fre);

    int count=0;
    while(ros::ok()){

        //time in total
        if (count>=fre*20){
            break;
        }

        cout<<"count"<<count<<endl;
        cout<<"current joint position "<<CurrentJointPosition[0]<<" "<<CurrentJointPosition[1]<<" "
        <<CurrentJointPosition[2]<<" "<<CurrentJointPosition[3]<<" "<<CurrentJointPosition[4]<<" "<<
        CurrentJointPosition[5]<<" "<<CurrentJointPosition[6]<<endl;
        cout<<"CurrentCartesianWrench "<<CurrentCartesianWrench[0]<< " " <<CurrentCartesianWrench[1]<<" "
        <<CurrentCartesianWrench[2]<<CurrentCartesianWrench[3]<< " " <<CurrentCartesianWrench[4]<<" "
        <<CurrentCartesianWrench[5]<<endl;

        count+=1;      

        //last state
        for (int i = 0; i<7; i++){
            PreTheta[i] = CurrentJointPosition[i];
        }

        Mat TCP_t = (Mat_<float>(6,1)<<0,652.51,438.71,PI,0,0);

        double TCP_target[5] = {0};
        TCP_target[0] = TCP_original[0]+(TCP_t.ptr<float>(0)[0]-TCP_original[0])*count/fre/20;
        TCP_target[1] = TCP_original[1]+(TCP_t.ptr<float>(0)[1]-TCP_original[1])*count/fre/20;
        TCP_target[2] = TCP_original[2]+(TCP_t.ptr<float>(0)[2]-TCP_original[2])*count/fre/20;
        TCP_target[3] = TCP_original[3]+(TCP_t.ptr<float>(0)[3]-TCP_original[3])*count/fre/20;
        TCP_target[4] = TCP_original[4]+(TCP_t.ptr<float>(0)[4]-TCP_original[4])*count/fre/20;
        TCP_target[5] = TCP_original[5]+(TCP_t.ptr<float>(0)[5]-TCP_original[5])*count/fre/20;
        
        for (int i = 0; i<6; i++){
            cout<<TCP_original[i]<<"/";
        }
        cout<<endl;

        //inv_kine
        Mat RPY_t=Mat::zeros(6,1,CV_32FC1);
        cout<<"Target ";
        for (int i = 0; i<6; i++){
            RPY_t.ptr<float>(0)[i] = TCP_target[i];
            cout<<RPY_t.ptr<float>(0)[i]<<"/";
        }
        cout<<endl;
        double Theta[7]={0};
        double Phi = 0;
        Inv_Kine(Theta,RPY_t,Phi,PreTheta,7);

        cout<<"current theta "<<Theta[0]<<" "<<Theta[1]<<" "
        <<Theta[2]<<" "<<Theta[3]<<" "<<Theta[4]<<" "<<
        Theta[5]<<" "<<Theta[6]<<endl;

        //out file
        in<<count<<"\n";
        in<<"current joint position "<<CurrentJointPosition[0]<<" "<<CurrentJointPosition[1]<<" "
        <<CurrentJointPosition[2]<<" "<<CurrentJointPosition[3]<<" "<<CurrentJointPosition[4]<<" "<<
        CurrentJointPosition[5]<<" "<<CurrentJointPosition[6]<<"\n";
        in<<"CurrentCartesianWrench "<<CurrentCartesianWrench[0]<< " " 
        <<CurrentCartesianWrench[1]<<" "<<CurrentCartesianWrench[2]<<" "
        <<CurrentCartesianWrench[3]<< " " <<CurrentCartesianWrench[4]<<" "<<CurrentCartesianWrench[5]<<"\n";

        //input in robot on joint space
        #ifdef JOINTPOSITION
        iiwa_msgs::JointPosition JointCommand;
        JointCommand.position.a1=Theta[0];
        JointCommand.position.a2=Theta[1];
        JointCommand.position.a3=Theta[2];
        JointCommand.position.a4=Theta[3];
        JointCommand.position.a5=Theta[4];
        JointCommand.position.a6=Theta[5];
        JointCommand.position.a7=Theta[6];
        
        JointCommandPub.publish(JointCommand);
        ros::spinOnce();
        loop_rate.sleep();
        #endif

        #ifdef JOINTVELOCITY
        break;
        ros::spinOnce();
        loop_rate.sleep();    
        #endif

    }
    
    in.close();//关闭文件

    return 0;

}