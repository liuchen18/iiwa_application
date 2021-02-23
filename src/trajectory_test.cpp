#include "ros/ros.h"
#include "7_DOF_inv.h"
#include "cubicspline.h"
#include "iiwa_robot.h"
#include <fstream>
#include "iiwa_msgs/JointPositionVelocity.h"

std::vector<cv::Mat> generate_desired_pose(std::vector<double> &time_stamp){
    double init_cartesian_pose[3]={191.7,0,1013};

    std::vector<cv::Mat> desired_pose;
    for(int i=0;i<200;i++){
        cv::Mat RPY_t=cv::Mat::zeros(6,1,CV_32FC1);
        RPY_t.ptr<float>(0)[0]=init_cartesian_pose[0];
        RPY_t.ptr<float>(0)[1]=init_cartesian_pose[1] + 50.0*sin(i*1.0/200*2*PI);
        RPY_t.ptr<float>(0)[2]=init_cartesian_pose[2] + 50.0*(1-cos(i*1.0/200*2*PI));
        RPY_t.ptr<float>(0)[3]=0;
        RPY_t.ptr<float>(0)[4]=0;
        RPY_t.ptr<float>(0)[5]=0;
        desired_pose.push_back(RPY_t);
        double time=i*0.1;
        time_stamp.push_back(time);
    }
    return desired_pose;
}

std::vector<std::vector<double>> inverse_kinematics_vector(std::vector<cv::Mat> pose_list){
    std::vector<std::vector<double>> joint_value_list;

    std::vector<double> last_joint_value(7,0),current_joint_value(7,0);
    last_joint_value[0]=-0.0;
    last_joint_value[1]=-0.49668;
    last_joint_value[2]=0;
    last_joint_value[3]=-1.87076;
    last_joint_value[4]=0;
    last_joint_value[5]=-1.37629;
    last_joint_value[6]=-0.0;

    for(auto pose:pose_list){
        std::vector<double> next=inverse_kinematics(pose,0.0,last_joint_value);
        joint_value_list.push_back(next);
        for(int i=0;i<last_joint_value.size();i++){
            last_joint_value[i]=next[i];
        }

    }

    return joint_value_list;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iiwa_traj");
    iiwa_robot iiwa;
    ros::NodeHandle nh;
    //ros::Subscriber vel_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, subCallback);
    //ros::Subscriber pos_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, subCallback);
    //ros::Subscriber pose_sub = nh.subscribe("/iiwa/state/CartesionPose", 1000, subCallback);
    ros::Publisher posvel_pub = 
        nh.advertise<iiwa_msgs::JointPositionVelocity>("/iiwa/command/JointPositionVelocity", 1000);

    //compute the desired pose and inverse kinematics
    std::vector<double> time_stamp;
    std::vector<cv::Mat> desired_pose=generate_desired_pose(time_stamp);
    std::vector<std::vector<double>> joint_value_list=inverse_kinematics_vector(desired_pose);
    ROS_INFO("inverse kinematics completed!");

    //write the result of the inverse kinematics to the txt file 
    std::ofstream out("desired_joint_values.txt");
    for (auto joint_value:joint_value_list){
        for(auto joint: joint_value){
            out<<joint<< " ";
        }
        out<<std::endl;
    }
    out.close();

    //convert the joint value list to its transpose
    std::vector<std::vector<double>> joint_values;
    for(int i=0;i<7;i++){
        std::vector<double> current_joint;
        for(int j=0;j<joint_value_list.size();j++){
            current_joint.push_back(joint_value_list[j][i]);
        }
        joint_values.push_back(current_joint);
    }


    //use cubic spline to interpolate the results
    ROS_INFO("the length of time stamp is %d, the length of the joints values is %d",
        int(time_stamp.size()),int(joint_values[0].size()));
    tk::spline s1,s2,s3,s4,s5,s6,s7;
    s1.set_points(time_stamp,joint_values[0]);
    s2.set_points(time_stamp,joint_values[1]);
    s3.set_points(time_stamp,joint_values[2]);
    s4.set_points(time_stamp,joint_values[3]);
    s5.set_points(time_stamp,joint_values[4]);
    s6.set_points(time_stamp,joint_values[5]);
    s7.set_points(time_stamp,joint_values[6]);
    ROS_INFO("interpolation completed!");

    //write the interpolated points to the txt file
    std::ofstream myout("interpolated_values.txt");
    for(int i=0;i<10*time_stamp.size();i++){
        myout<<s1(i*0.01)<<" "<<s1.deriv(1,i*0.01)<<" "<<s2(i*0.01)<<" "<<s2.deriv(1,i*0.01)<<" "
            <<s3(i*0.01)<<" "<<s3.deriv(1,i*0.01)<<" "<<s4(i*0.01)<<" "<<s4.deriv(1,i*0.01)<<" "
            <<s5(i*0.01)<<" "<<s5.deriv(1,i*0.01)<<" "<<s6(i*0.01)<<" "<<s6.deriv(1,i*0.01)<<" "
            <<s7(i*0.01)<<" "<<s7.deriv(1,i*0.01)<<" "<<s1.deriv(2,i*0.01)<<std::endl;
    }
    myout.close();

    //start the main loop to control the robot
    int hz=100;
    ros::Rate loop_rate(hz);
    int count=0;
    ROS_INFO("start the main loop!");

    while(ros::ok()){
        iiwa_msgs::JointPositionVelocity posvel;

        //initial position
        posvel.position.a1=-0.0;
        posvel.position.a2=-0.49668;
        posvel.position.a3=0;
        posvel.position.a4=-1.87076;
        posvel.position.a5=0;
        posvel.position.a6=-1.37629;
        posvel.position.a7=-0.0;

        
        posvel.position.a1=s1(count*1.0/hz);
        
        posvel.position.a2=s2(count*1.0/hz);
        
        posvel.position.a3=s3(count*1.0/hz);
        
        posvel.position.a4=s4(count*1.0/hz);
        
        posvel.position.a5=s5(count*1.0/hz);
        posvel.position.a6=s6(count*1.0/hz);
        posvel.position.a7=s7(count*1.0/hz);
        
        
        posvel.velocity.a1=s1.deriv(1,count*1.0/hz);
        
        posvel.velocity.a2=s2.deriv(1,count*1.0/hz);
        
        posvel.velocity.a3=s3.deriv(1,count*1.0/hz);
        
        posvel.velocity.a4=s4.deriv(1,count*1.0/hz);
        
        posvel.velocity.a5=s5.deriv(1,count*1.0/hz);
        posvel.velocity.a6=s6.deriv(1,count*1.0/hz);
        posvel.velocity.a7=s7.deriv(1,count*1.0/hz);
        
        

        posvel_pub.publish(posvel);
        count+=1;
        if(count == 20*hz){
            exit(0);
        }

        loop_rate.sleep();
    }


    return 0;
}






