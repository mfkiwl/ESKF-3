#include <sys/time.h>
#include "common.h"

// /**
//  *@function: 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
//    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
//  * @param in: q0 q1 q2 q3  w x y z
//  * @return: (roll, pitch, yaw)弧度
// **/
// Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q){
//     float quat[4];
//     quat[0] = q.w();
//     quat[1] = q.x();
//     quat[2] = q.y();
//     quat[3] = q.z();

//     Eigen::Vector3d ans;
//     ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
//     ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
//     ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
//     return ans;
// }

// /**
//  * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
//  * @param in: vector3d
//  * @return:   q(w x y z) 
//  */
// Eigen::Quaterniond euler_to_quatertion(const Eigen::Vector3d euler){
//     double roll = euler[0];
//     double pitch = euler[1];
//     double yaw = euler[2];

//     return euler_to_quatertion(roll, pitch, yaw);
// }

// /**
//  * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
//  * @param in: roll, pitch, yaw
//  * @return:   w x y z 
//  */
// Eigen::Quaterniond euler_to_quatertion(const double roll, const double pitch, const double yaw){
//     // Abbreviations for the various angular functions
//     double cy = cos(yaw * 0.5);
//     double sy = sin(yaw * 0.5);
//     double cp = cos(pitch * 0.5);
//     double sp = sin(pitch * 0.5);
//     double cr = cos(roll * 0.5);
//     double sr = sin(roll * 0.5);

//     Eigen::Vector4d vq;
    
//     vq[0] = cy * cp * cr + sy * sp * sr;
//     vq[1] = cy * cp * sr - sy * sp * cr;
//     vq[2] = sy * cp * sr + cy * sp * cr;
//     vq[3] = sy * cp * cr - cy * sp * sr;

//     Eigen::Quaterniond q(vq);

//     return q;
// }

uint64_t hrt_absolute_time() {
    //get time ms and us
    struct timeval tv;
    struct timezone tz;
    
    gettimeofday(&tv,&tz);

    return tv.tv_sec * 1000000 + tv.tv_usec;
}