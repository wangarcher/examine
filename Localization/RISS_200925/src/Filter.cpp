#include <iostream>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../include/Filter.h"

#include "../include/Robot.h"
#include "../include/Odom.h"
#include "../include/Imu.h"
#include "../include/Gps.h"

WeightingFilter weightingFilter;
WeightingFilter* pweightingFilter;


KalmanFilter kalmanFilter;
KalmanFilter *pkalmanFilter;

// Constructor
KalmanFilter::KalmanFilter()
{
    is_initialized_ = false;
    lastTimeStamp_ = 0;
    nowTimeStamp_ = 0;
    deltaTime_ = 0;
}

//Destructor
KalmanFilter::~KalmanFilter()
{
    delete pkalmanFilter;
}

void KalmanFilter::Initialization()
{
    lastTimeStamp_ = nowTimeStamp_ = getSysTime();
    
    Eigen::VectorXf x_in(5,1);
    x_in << 0, 
            0,
            0, 
            0, 
            0;
    SetX(x_in);

    // state covariance matrix the prediction error
    Eigen::MatrixXf P_in(5,5);
    P_in << 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1;
    SetP(P_in);


    //process covariance matrix
    Eigen::MatrixXf Q_in(5,5);
    Q_in << 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1;
    SetQ(Q_in);

    //Observation
    Eigen::VectorXf z_in(3,1);
    z_in << 0.0,
            0.0, 
            0.0;


    //measurement matrix
    Eigen::MatrixXf H_in(3,5);
    H_in << 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 1.0;
    SetH(H_in);
    
    
    //measurement covariance matrix
    // R is provided by Sensor supplier
    Eigen::MatrixXf R_in(3,3);
    R_in << 0.1, 0.0,0.0,
            0.0, 0.1,0.0,
            0.0, 0.0,0.1;
    SetR(R_in);


    is_initialized_ = true;

}

void KalmanFilter::SetX(Eigen::VectorXf x_in)
{
    x_ = x_in;
}

bool KalmanFilter::GetIsInitialized()
{
    return is_initialized_;
}

void KalmanFilter::SetF(Eigen::MatrixXf F_in)
{
    F_ = F_in;
}

void KalmanFilter::SetP(Eigen::MatrixXf P_in)
{
    P_ = P_in;
}

void KalmanFilter::SetQ(Eigen::MatrixXf Q_in)
{
    Q_ = Q_in;
}

void KalmanFilter::SetH(Eigen::MatrixXf H_in)
{
    H_ = H_in;
}

void KalmanFilter::SetR(Eigen::MatrixXf R_in)
{
    R_ = R_in;
}

void KalmanFilter::Prediction(float d_t) //预测
{
    float x_k, y_k, theta_k, v_k, w_k;
    x_k = x_[0];
    y_k = x_[1];
    theta_k = x_[2];
    v_k = x_[3];
    w_k = x_[4];

    x_ << x_k + 2.2*v_k*d_t*cos(theta_k),y_k + 2.2*v_k*d_t*sin(theta_k), theta_k + 3.8*w_k*d_t, v_k,  w_k;
    Eigen::MatrixXf F_in(5,5);
    F_in << 1, 0, 0, d_t*cos(x_[2]), 0,
            0, 1, 0, d_t*sin(x_[2]), 0,
            0, 0, 1, 0, d_t,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    kalmanFilter.SetF(F_in);
    Eigen::MatrixXf Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}
/*
void KalmanFilter::CalculateJacobianMatrix()
{
    Eigen::MatrixXf Hj(4,4);

    //get state paraeters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    //pre-compute a set of terms to avoid repeated calculation
    Hj << 1.0, 0.0, 0.0, 0.0,   //GPS 与 ODOM与IMU加权平均 后的结果进行再次融合
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;

    SetH(Hj);
}
*/
void KalmanFilter::KFUpdate(const Eigen::VectorXf &z)
{
    int size = x_.size();
    std::cout << size << std::endl;
    Eigen::VectorXf y = z - H_ * x_;
    Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K * y);
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(size, size);
    P_ = (I - K * H_) * P_;
    //std::cout << "x_:" << std::endl << x_ << std::endl;

}
/*
void KalmanFilter::EKFUpdate(const Eigen::VectorXf &z)
{   
   
    // Eigen::VectorXf h = Eigen::VectorXf(4);
    CalculateJacobianMatrix(); 

    Eigen::VectorXf y = z - H_ * x_;
    Eigen::MatrixXf Ht = H_.transpose();
    Eigen::MatrixXf S = H_ * P_ * Ht + R_;
    Eigen::MatrixXf Si = S.inverse();
    Eigen::MatrixXf K = P_ * Ht * Si;

    x_ = x_ + (K * y);

    int x_size = x_.size();
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
	std::cout << "x_:" << std::endl << x_ <<std::endl;

}
*/
Eigen::VectorXf KalmanFilter::GetX()
{
    return x_;
}

void KalmanFilter::setLastTimeStamp(const long int lastTimeStamp)
{
    this->lastTimeStamp_ = lastTimeStamp;
}

void KalmanFilter::setNowTimeStamp(const long int nowTimeStamp )
{
    this->nowTimeStamp_ = nowTimeStamp;
}
void KalmanFilter::setDeltaTime(const long int deltaTime)
{
    this->deltaTime_ = deltaTime;
}

void KalmanFilter::getLastTimeStamp(long int& lastTimeStamp)
{
    lastTimeStamp = this->lastTimeStamp_;
}
void KalmanFilter::getNowTimeStamp(long int& nowTimeStamp)
{
    nowTimeStamp = this->nowTimeStamp_;
}
void KalmanFilter::getDeltaTime(long int& deltaTime)
{
    deltaTime = this->deltaTime_;
}


WeightingFilter::WeightingFilter()
{

}

WeightingFilter::~WeightingFilter()
{
    delete pweightingFilter;
}

void WeightingFilter::WeightingFilterUpdate()  //只是进行位置融合
{
    float odom_x, odom_y, odom_theta;
    float imu_x, imu_y, imu_z, imu_theta;
    float Roll; //翻滚角
    float Pitch; //俯仰角
    float Yaw; //偏航角

    odom.GetPos(odom_x, odom_y, odom_theta);
    imu.GetPosition( imu_x, imu_y, imu_theta);

    float robot_x, robot_y, robot_theta;

    robot_x = WEIGHT * odom_x + (1- WEIGHT) * imu_x;
    robot_y = WEIGHT * odom_y + (1- WEIGHT) * imu_y;
    imu.GetPostureYPR(Roll, Pitch, Yaw);
    robot_theta = Yaw/57.3;
    
    robot.SetRobotPosition(robot_x, robot_y, 0); 
    robot.SetRobotRotation(Roll, Pitch, Yaw);
    odom.SetPos(robot_x,robot_y,robot_theta);
    imu.SetPosition(robot_x,robot_y,robot_theta);
}





