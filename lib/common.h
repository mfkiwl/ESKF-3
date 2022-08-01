/**************************************************************
 * Copyright (C) 2020 Vtol Develop Team - All Rights Reserved
 *************************************************************/

/**
 * @file the common lib function
 * @author zhongzhw zhongzhwin@163.com
 */

#ifndef COMMON_H
#define COMMON_H

#include <cmath>
#include <Eigen/Dense>

/**
 * @function trans degree to rad
 * @param degree
 * @return rad
 */
template<class T>
float deg2rad(T degree) {
    float deg = static_cast<double>(degree);
    float rad = deg * M_PI / 180;
    return rad;
}

/**
 * @function trans rad to degree
 * @param rad
 * @return degree
 */
template<class T>
float rad2deg(T radi) {
    float rad = static_cast<double>(radi);
    float deg = rad * 180 / M_PI;
    return deg;
}

template<typename T>
constexpr T radians(T degrees)
{
	return degrees * (static_cast<T>(M_PI) / static_cast<T>(180));
}

template<typename T>
constexpr T degrees(T radians)
{
	return radians * (static_cast<T>(180) / static_cast<T>(M_PI));
}

// /**
//  * @function: 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
//    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
//  * @param in: q0 q1 q2 q3  w x y z
//  * @return: (roll, pitch, yaw)
// **/
// Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q);

// /**
//  * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
//  * @param in: vector3d
//  * @return:   q(w x y z) 
//  */
// Eigen::Quaterniond euler_to_quatertion(const Eigen::Vector3d euler);

// /**
//  * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
//  * @param in: roll, pitch, yaw
//  * @return:   w x y z 
//  */
// Eigen::Quaterniond euler_to_quatertion(const double roll, const double pitch, const double yaw);

uint64_t hrt_absolute_time();

#endif