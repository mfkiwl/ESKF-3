#ifndef __ESKF__
#define __ESKF__

#include <iostream>
#include "base_type.h"

using namespace std;

constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;

class ESKF{
public:
    ESKF(){};
    void Init(const GPSData&, State&);
    void Predict(const IMUData&, State&);
    void Update(const GPSData&, State&);
    void Injection(State&);
    void Reset(State&);
    Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d&);
    Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond&, const Eigen::Quaterniond&);
private:
    double acc_noise = 5e-4;
    double gyro_noise = 5e-4;
    double acc_bias_noise = 5e-4;
    double gyro_bias_noise = 5e-4;
    double pose_noise = 0.8;
};

void ESKF::Init(const GPSData& gps_data, State& state){
    state.timestamp = gps_data.timestamp;
    state.position = gps_data.ned;
    state.cov.block<3, 3>(0, 0) = pose_noise * Eigen::Matrix3d::Identity(); // position cov
    state.cov.block<3, 3>(3, 3) = 10. * Eigen::Matrix3d::Identity(); // velocity cov
    state.cov.block<3, 3>(6, 6) = 1. * Eigen::Matrix3d::Identity(); // posture cov
}

void ESKF::Predict(const IMUData& imu_data, State& state){
    const double delta_t = imu_data.timestamp - state.timestamp;
    state.timestamp = imu_data.timestamp;
    
    // rotation matrix from IMU to NED
    Eigen::Matrix3d R = state.quaternion.toRotationMatrix();

    state.position = state.position + state.velocity * delta_t + 0.5 * (R * (imu_data.acc - state.acc_bias) + state.gravity) * delta_t * delta_t;
    state.velocity = state.velocity + (R * (imu_data.acc - state.acc_bias) + state.gravity) * delta_t;
    state.quaternion = kronecker_product(state.quaternion, euler_to_quatertion((imu_data.gyro - state.gyro_bias) * delta_t));

    Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6) = - skewsym_matrix(R * (imu_data.acc - state.acc_bias)) * delta_t;
    Fx.block<3, 3>(3, 9) = - R * delta_t;
    Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(6, 12) = - R * delta_t;

    Eigen::Matrix<double, 18, 12> Fi = Eigen::Matrix<double, 18, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t * delta_t * acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t * delta_t * gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise * Eigen::Matrix3d::Identity();

    // state.error = Fx * state.error;
    state.cov = Fx * state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
};

void ESKF::Update(const GPSData& gps_data, State& state){
    Eigen::Vector3d Y(gps_data.ned[0], gps_data.ned[1], gps_data.ned[2]);

    Eigen::Vector3d X(state.position[0], state.position[1], state.position[2]);

    Eigen::Matrix3d V = pose_noise * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 19> Hx = Eigen::Matrix<double, 3, 19>::Zero();
    Hx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 19, 18> Xx = Eigen::Matrix<double, 19, 18>::Identity();
    Eigen::Matrix<double, 4, 3> Q_theta;
    Q_theta << - state.quaternion.x(), - state.quaternion.y(), - state.quaternion.z(), 
                state.quaternion.w(), state.quaternion.z(), - state.quaternion.y(),
                - state.quaternion.z(), state.quaternion.w(), state.quaternion.x(),
                state.quaternion.y(), - state.quaternion.x(), state.quaternion.w();
    Q_theta *= 0.5;
    Xx.block<4, 3>(6, 6) = Q_theta;
    
    Eigen::Matrix<double, 3, 18> H = Hx * Xx;

    Eigen::MatrixXd K = state.cov * H.transpose() * (H * state.cov * H.transpose() + V).inverse();
    state.error = K * (Y - X);
    state.cov = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * state.cov;
};

void ESKF::Injection(State& state){
    Eigen::Vector3d error_pos = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(0, 0).transpose().data());
    Eigen::Vector3d error_vel = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(3, 0).transpose().data());
    
    Eigen::Vector3d error_ori = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(6, 0).transpose().data());
    Eigen::Quaterniond error_quat = euler_to_quatertion(error_ori);

    Eigen::Vector3d error_acc_bias = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(9, 0).transpose().data());
    Eigen::Vector3d error_gyr_bias = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(12, 0).transpose().data());
    Eigen::Vector3d error_gra = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(15, 0).transpose().data());

    state.position = state.position + error_pos;
    state.velocity = state.velocity + error_vel;
    state.quaternion = kronecker_product(error_quat, state.quaternion);
    state.acc_bias = state.acc_bias + error_acc_bias;
    state.gyro_bias = state.gyro_bias + error_gyr_bias;
    // state.gravity = state.gravity + error_gra;
}

void ESKF::Reset(State& state){
    state.error.setZero();
}

Eigen::Quaterniond ESKF::kronecker_product(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q){
    Eigen::Quaterniond res;
    res.w() = p.w() * q.w() - p.x() * q.x() - p.y() * q.y() - p.z() * q.z();
    res.x() = p.w() * q.x() + p.x() * q.w() + p.y() * q.z() - p.z() * q.y();
    res.y() = p.w() * q.y() - p.x() * q.z() + p.y() * q.w() + p.z() * q.x();
    res.z() = p.w() * q.z() + p.x() * q.y() - p.y() * q.x() + p.z() * q.w();
    return res;
}

Eigen::Matrix3d ESKF::skewsym_matrix(const Eigen::Vector3d& vec){
    Eigen::Matrix3d mat;
    mat <<  0.,   -vec(2),  vec(1),
          vec(2),  0.,   -vec(0),
         -vec(1),  vec(0),  0.;
    return mat;
}

#endif