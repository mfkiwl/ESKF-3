#ifndef __ROSWrapper__
#define __ROSWrapper__

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geography.h"
#include "base_type.h"
#include "eskf.h"
#include <boost/thread/thread.hpp>

using namespace std;

class ROSWrapper{
public:
    ROSWrapper(ros::NodeHandle &n, double lat, double lon);
    void gps_callback(const sensor_msgs::NavSatFixConstPtr&);
    void imu_callback(const sensor_msgs::ImuConstPtr&);
    void assign_imu(const sensor_msgs::ImuConstPtr&, IMUData&);
    void assign_gps(const sensor_msgs::NavSatFixConstPtr&, GPSData&);

private:
    ros::NodeHandle nh;
    nav_msgs::Path gps_path;
    nav_msgs::Path fusion_path;

    ros::Publisher gps_pub;
    ros::Publisher fusion_pub;

    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;

    State state;
    IMUData imu_data;
    GPSData gps_data;
    map_projection_reference_s map_ref;
    ESKF eskf;
    bool init;
};

ROSWrapper::ROSWrapper(ros::NodeHandle &n, const double lat, const double lon): nh(n), init(false){
    gps_pub = nh.advertise<nav_msgs::Path>("/gps_path", 10);

    fusion_pub = nh.advertise<nav_msgs::Path>("/fusion_path", 10);

    gps_sub = nh.subscribe("/fix", 10, &ROSWrapper::gps_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &ROSWrapper::imu_callback, this);

    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;

    fusion_path.header.frame_id = "map";
    fusion_path.header.stamp = ros::Time::now();
    fusion_path.header.seq = 0;

    state.position = Eigen::Vector3d::Zero();
    state.velocity = Eigen::Vector3d::Zero();
    state.quaternion = Eigen::Quaterniond(0., 0., 0., 0.);
    state.acc_bias = Eigen::Vector3d::Zero();
    state.gyro_bias = Eigen::Vector3d::Zero();
    state.gravity = Eigen::Vector3d(0., 0., 9.81007);
    state.cov = Eigen::Matrix<double, 18, 18>::Zero();
    state.error = Eigen::Matrix<double, 18, 1>::Zero();

    map_projection_init(&map_ref, lat, lon);
    ROS_INFO("ESKF Init");
}

void ROSWrapper::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr){
    assign_imu(imu_msg_ptr, imu_data);

    if(!init) return;

    eskf.Predict(imu_data, state);
}

void ROSWrapper::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr){
    assign_gps(gps_msg_ptr, gps_data);

    if(!init){
        eskf.Init(gps_data, state);
        init = true;
        return;
    }

    eskf.Update(gps_data, state);
    eskf.Injection(state);
    eskf.Reset(state);

    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.pose.position.x = gps_data.ned[0];
    point.pose.position.y = gps_data.ned[1];
    point.pose.position.z = 0;
    point.pose.orientation.w = 0;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 0;
    gps_path.poses.push_back(point);
    gps_pub.publish(gps_path);

    geometry_msgs::PoseStamped point_fused;
    point_fused.header.frame_id = "map";
    point_fused.header.stamp = ros::Time::now();
    point_fused.pose.position.x = state.position[0];
    point_fused.pose.position.y = state.position[1];
    point_fused.pose.position.z = 0;
    point_fused.pose.orientation.w = state.quaternion.w();
    point_fused.pose.orientation.x = state.quaternion.x();
    point_fused.pose.orientation.y = state.quaternion.y();
    point_fused.pose.orientation.z = state.quaternion.z();

    fusion_path.poses.push_back(point_fused);
    fusion_pub.publish(fusion_path);
}

void ROSWrapper::assign_imu(const sensor_msgs::ImuConstPtr& imu_msg_ptr, IMUData& imu_data){
    imu_data.timestamp = imu_msg_ptr->header.stamp.toSec();

    imu_data.acc = Eigen::Vector3d(imu_msg_ptr->linear_acceleration.x, 
                                    imu_msg_ptr->linear_acceleration.y,
                                    imu_msg_ptr->linear_acceleration.z);

    imu_data.gyro = Eigen::Vector3d(imu_msg_ptr->angular_velocity.x,
                                    imu_msg_ptr->angular_velocity.y,
                                    imu_msg_ptr->angular_velocity.z);
}

void ROSWrapper::assign_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr, GPSData& gps_data){
    gps_data.timestamp = gps_msg_ptr->header.stamp.toSec();

    gps_data.lla = Eigen::Vector3d(gps_msg_ptr->latitude,
                                    gps_msg_ptr->longitude,
                                    gps_msg_ptr->altitude);

    float x, y;
    map_projection_project(&map_ref, gps_msg_ptr->latitude, gps_msg_ptr->longitude, &x, &y);
    gps_data.ned << x, y, -gps_msg_ptr->altitude;
}

#endif