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
    CurrentJointPosition[1]=Position.position.a1;
    CurrentJointPosition[2]=Position.position.a2;
    CurrentJointPosition[3]=Position.position.a3;
    CurrentJointPosition[4]=Position.position.a4;
    CurrentJointPosition[5]=Position.position.a5;
    CurrentJointPosition[6]=Position.position.a6;
    CurrentJointPosition[7]=Position.position.a7;
    //std::cout<<"current joint position "<<Position.position.a1<<" "<<Position.position.a2<<" "
    //<<Position.position.a3<<" "<<Position.position.a4<<" "<<Position.position.a5<<" "<<
    //Position.position.a6<<" "<<Position.position.a7<<std::endl;
    
}

double CurrentCartesianWrench[6];

void callback_wrench(const iiwa_msgs::CartesianWrench& Wrench){
    CurrentCartesianWrench[1]=Wrench.wrench.force.x;
    CurrentCartesianWrench[2]=Wrench.wrench.force.y;
    CurrentCartesianWrench[3]=Wrench.wrench.force.z;
    CurrentCartesianWrench[4]=Wrench.wrench.torque.x;//not sure
    CurrentCartesianWrench[5]=Wrench.wrench.torque.y;
    CurrentCartesianWrench[6]=Wrench.wrench.torque.z;

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

    //cal initial postion and force
    double Final[4][4]={0};
    double Theta_o[7] = {0};
    for(int i=0;i<7;i++){
        Theta_o[0]=PI/2;
        Theta_o[1]=PI/6;
        Theta_o[2]=0;
        Theta_o[3]=-PI/3;
        Theta_o[4]=0;
        Theta_o[5]=PI/2;
        Theta_o[6]=0;
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
    
    //control rate
    int fre = 100;
    ros::Rate loop_rate(fre);

    int count=0;
    while(ros::ok()){
        count+=1;


        //last state
        double PreTheta[7]={0};
        for (int i = 0; i<7; i++){
            PreTheta[i] = CurrentJointPosition[i];
        }


        //reference
        double Reference[6] = {0};
        Reference[2] = count/fre*1e-3; //speed related


        //read force
        double CurrentCartesianWrench_r[6]={0};
        cout<<CurrentCartesianWrench_r[0]<< " " <<CurrentCartesianWrench_r[1]<<" "<<CurrentCartesianWrench_r[2]<<endl;
        cout<<CurrentCartesianWrench_r[3]<< " " <<CurrentCartesianWrench_r[4]<<" "<<CurrentCartesianWrench_r[5]<<endl;
        for (int i = 0; i<5; i++){
            CurrentCartesianWrench_r[i] = CurrentCartesianWrench[i]-CurrentCartesianWrench_o[i];
        }


        //end judgement & safety range
        int fre6 = fre*6;
        if (count >= fre6||CurrentCartesianWrench_r[2]>=45){
            break;
            cout<<"end the assembly or force over limit"<<endl;
        }


        //state evauation through kinematic
        //double LH = 40e-3;//ingore the coupled influenced on dx and dy
        double l = Reference[2];


        //admittance controll adjustment value 
        double M[5] = {0};
        double D[5] = {0};
        double K[5] = {1e6,1e6,1e9,1e6,1e6};
        double A[5][5] = {0};
        A[0][0] = 1;
        A[1][1] = 1;
        A[2][2] = 1;
        A[3][3] = 1;
        A[4][4] = 1;
        double adjustment[5] = {0};
        for (int i = 0; i<5; i++){
            adjustment[i] = CurrentCartesianWrench_r[i]*A[i][i]/K[i];
        }
        


        //TCP position
        double TCP_adjustment[5] = {0};
        TCP_adjustment[0] = adjustment[0]+l*sin(adjustment[4]);
        TCP_adjustment[1] = adjustment[1]+l*sin(adjustment[3]);
        TCP_adjustment[2] = 0;
        TCP_adjustment[3] = adjustment[3];
        TCP_adjustment[4] = adjustment[4];

        if (TCP_adjustment[0] >= 1e-3||TCP_adjustment[1] >= 1e-3||TCP_adjustment[2] >= 1e-3||TCP_adjustment[4] >= 1||TCP_adjustment[5] >= 1){
            break;
            TCP_adjustment[0] >= 1e-3;
        }

        double TCP_target[5] = {0};
        for (int i = 0; i<5; i++){
            TCP_target[i] = TCP_original[i]+Reference[i]-TCP_adjustment[i];
        }

        //inv_kine
        Mat RPY_t=Mat::zeros(3,1,CV_32FC1);
        for (int i = 0; i<5; i++){
            RPY_t.ptr<float>(0)[1] = TCP_target[i];
        }
        double Theta[7]={0};
        double Phi = 0;
        Inv_Kine(Theta,RPY_t,Phi,PreTheta,7);


        //out file
        ofstream outfile("Desktop/record.txt");
        for (int i=0;i<7;i++)
        {
            outfile<<Theta[i]<<"  ";
        }
        outfile<<endl;
        for (int i=0;i<6;i++)
        {
            outfile<<CurrentCartesianWrench_r[i]<<"  ";
        }
        outfile<<endl;
	


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

    return 0;

}