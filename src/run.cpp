#include <iostream>
#include "ros/ros.h"
#include "ros_wrapper.h"
#include "base_type.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "eskf");

    ros::NodeHandle n;

    ROSWrapper eskf(n, 47.5115140833, 6.79310693333);

    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}