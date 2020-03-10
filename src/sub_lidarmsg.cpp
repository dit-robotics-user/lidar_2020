#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"

#include "sensor_msgs/LaserScan.h"
#include "lidar_2020/alert_range.h"

#define BEAM_NUM 8


void callback(const lidar_2020::alert_range::ConstPtr& msg ){
    
    for(int i=0 ; i<BEAM_NUM ; i++){
        ROS_INFO("alert[%d] = %d" , i ,msg->alert[i]);
    }
    
    
}

int main(int argc, char **argv){

    ros::init(argc,argv,"sub_lidarmsg");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<lidar_2020::alert_range>("ranging_alert", 1000, callback);
    ros::Publisher pub = nh.advertise<lidar_2020::alert_range>("agent",1000);
    ros::spin();
    return 0;
}