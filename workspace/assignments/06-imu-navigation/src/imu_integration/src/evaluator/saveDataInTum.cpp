#include <fstream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include "stdio.h"

static std::ofstream ground_truth_str;

bool CreateDirectory(std::string directory_path){
    if(!boost::filesystem::is_directory(directory_path))
    boost::filesystem::create_directory(directory_path);

    if(!boost::filesystem::is_directory(directory_path)){
        std::cout<< "cannot create directory" << std::endl;
        return false;
    }

    std::string filename{"/workspace/assignments/06-imu-navigation/src/imu_integration/result/sim/gt.txt"};
    remove(filename.c_str());
    return true;
}

bool CreateFile(std::ofstream& ofs, std::string file_path){
    ofs.setf(std::ios::fixed, std::ios::floatfield);
    ofs.open(file_path.c_str(), std::ios::out|std::ios::app);
    if(!ofs){
        std::cout << "cannot open file" << std::endl;
        return false;
    }
    return true;
}

void callback_truth(const nav_msgs::OdometryConstPtr& ground_truth){
    
    static bool is_file_created = false;

    if(!is_file_created){
        if(!CreateDirectory("/workspace/assignments/06-imu-navigation/src/imu_integration/result/sim"))
        return;
        if(!CreateFile(ground_truth_str,"/workspace/assignments/06-imu-navigation/src/imu_integration/result/sim/gt.txt"))
        return;
        is_file_created = true;
    }

    Eigen::Vector3d t;
    t(0) = ground_truth->pose.pose.position.x;
    t(1) = ground_truth->pose.pose.position.y;
    t(2) = ground_truth->pose.pose.position.z;
    Eigen::Quaterniond q;
    q.w() = ground_truth->pose.pose.orientation.w;
    q.x() = ground_truth->pose.pose.orientation.x;
    q.y() = ground_truth->pose.pose.orientation.y;
    q.z() = ground_truth->pose.pose.orientation.z;

    ros::Time timestamp_ = ground_truth->header.stamp;
    double timestamp_in_sec = timestamp_.toSec();

    ground_truth_str.precision(9);
    ground_truth_str <<timestamp_in_sec<<" ";
    ground_truth_str.precision(5);
    ground_truth_str <<t(0)<<" "
                <<t(1)<<" "
                <<t(2)<<" "
                <<q.x()<<" "
                <<q.y()<<" "
                <<q.z()<<" "
                <<q.w() <<std::endl;

}

int main(int argc, char** argv){
    std::string nodename{"saveDataInTum"};
    ros::init(argc, argv, nodename);

    ros::NodeHandle nh;
    ros::Subscriber gt_sub = nh.subscribe("/sim/gt", 10, &callback_truth);

    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    ground_truth_str.close();
    return 0;
}