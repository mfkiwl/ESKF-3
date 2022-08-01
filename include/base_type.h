#ifndef __BASETYPE__
#define __BASETYPE__

#include <iostream>
#include <Eigen/Core>

struct IMUData{
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct GPSData{
    double timestamp;

    Eigen::Vector3d lla;
    Eigen::Vector3d ned;
};

struct State{
    double timestamp;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d gravity;
    Eigen::Matrix<double, 18, 18> cov;
    Eigen::Matrix<double, 18, 1> error;
};

#endif