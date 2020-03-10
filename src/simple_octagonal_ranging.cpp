#include "ros/ros.h"
#include <iostream>


#include "sensor_msgs/LaserScan.h"
#include "lidar_2020/alert_range.h"

#define BEAM_NUM 8

class Ranging
{
	public:
		Ranging(float alert_range_=0.3,float forget_rate_=0.9, int num_point_for_a_beam=40);
		~Ranging(){};
    	//public function
	private:
		//field
		ros::NodeHandle nh;
		ros::Publisher ranging_pub;
		ros::Subscriber scan_sub;
        float range_memory[BEAM_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float forget_rate;
        float rememb_rate;
        float alert_range;
        int half_num_point_for_a_beam;
		//private function
		void scan_sub_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        float average_ranging(const sensor_msgs::LaserScan::ConstPtr& msg, int start1,int end1,int start2=0,int end2=0);
};

Ranging::Ranging(float alert_range_,float forget_rate_, int num_point_for_a_beam){
    alert_range = alert_range_;
    if(forget_rate_>0.99999999999 || forget_rate_<0.0000001)
        forget_rate_ = 0.9;
    forget_rate = forget_rate_;
    rememb_rate = 1-forget_rate_;
    half_num_point_for_a_beam = num_point_for_a_beam/2;
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &Ranging::scan_sub_Callback, this);
    ranging_pub = nh.advertise<lidar_2020::alert_range>("ranging_alert", 1);
}

float Ranging::average_ranging(const sensor_msgs::LaserScan::ConstPtr& msg, int start1,int end1,int start2,int end2){
    int count = 0;
    float current_range = 0.0;
    for(int i=start1;i<end1;i++){
        if(msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min ){
            current_range += msg->ranges[i];
            count += 1;
        }
    }
    for(int i=start2;i<end2;i++){
        if(msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min ){
            current_range += msg->ranges[i];
            count += 1;
        }
    }
    if(count==0){
        count += 1;
    }
    current_range = current_range/count;
    return current_range;
}

void Ranging::scan_sub_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //-180 degree
    // 0~20, 340~360
    // memory_index 0
    range_memory[0] = rememb_rate*range_memory[0] + forget_rate*average_ranging(msg, 0, half_num_point_for_a_beam,\
                                                                                360-half_num_point_for_a_beam,360);
    //-135 degree
    // 25~65
    // memory_index 1
    // -90 degree
    // 70~110
    // memory_index 2
    // -45 degree
    // 115~155
    // memory_index 3
    //   0 degree
    // 160~200
    // memory_index 4
    //  45 degree
    // 205~245
    // memory_index 5
    //  90 degree
    // 250~290
    // memory_index 6
    // 135 degree
    // 295~335
    // memory_index 7
    for(int beam_i=1;beam_i<BEAM_NUM;beam_i++){
        range_memory[beam_i] = rememb_rate*range_memory[beam_i] + forget_rate*average_ranging(msg, beam_i*45 - half_num_point_for_a_beam,\
                                                                                                beam_i*45 + half_num_point_for_a_beam);
    }
    
    lidar_2020::alert_range ran_ale;
    ran_ale.header = msg->header;
    ran_ale.num = BEAM_NUM;
    for(int i=0;i<BEAM_NUM;i++){
        ran_ale.ranging_value.push_back(range_memory[i]);
        if(range_memory[i]<alert_range)
            ran_ale.alert.push_back(true);
        else
            ran_ale.alert.push_back(false);
    }
    
    ranging_pub.publish(ran_ale);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "simple_octagonal_ranging");

    float alert_range_;
    float forget_rate_;
    int num_point; //num point for_a_beam
    ros::param::param<float>("~alert_range"  , alert_range_ , 0.3);
    ros::param::param<float>("~forget_rate"  , forget_rate_ , 0.9);
    ros::param::param<int>("~num_point"  , num_point , 40);

    Ranging ranging(alert_range_,forget_rate_,num_point);

    ros::spin();
    return 0;
};