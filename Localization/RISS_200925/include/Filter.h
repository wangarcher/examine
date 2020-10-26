#ifndef _FILTER_H
#define _FILTER_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>

#include "../include/Robot.h"
#include "../include/Mtime.h"

#define WEIGHT 1.0

class  WeightingFilter
{
public:
    /// constructor
    WeightingFilter();
    
    /// destructor
    ~WeightingFilter();

    void initialize();

    bool isInitialized();

    void WeightingFilterUpdate();

private:
    bool odom_active_, imu_active_, vo_active_, gps_active_;
    bool odom_used_, imu_used_, v0_used_, gps_used_;
    bool odom_initializing_, imu_initializing_, vo_initializing_, gps_initializing_;
};

class KalmanFilter
{
public:
    // Constructor
    KalmanFilter();

    //Destructor
    ~KalmanFilter();

    void Initialization();
 
    void SetX(Eigen::VectorXf x_in);

    bool GetIsInitialized();

    void SetF(Eigen::MatrixXf F_in);

    void SetP(Eigen::MatrixXf P_in);

    void SetQ(Eigen::MatrixXf Q_in);

    void SetH(Eigen::MatrixXf H_in);

    void SetR(Eigen::MatrixXf R_in);

    void Prediction(float d_t); //预测

    void CalculateJacobianMatrix();

    void KFUpdate(const Eigen::VectorXf &z);

    void EKFUpdate(const Eigen::VectorXf &z);

    Eigen::VectorXf GetX();

    void setLastTimeStamp(const long int lastTimeStamp);
    void setNowTimeStamp(const long int nowTimeStamp );
    void setDeltaTime(const long int deltaTime);

    void getLastTimeStamp(long int& lastTimeStamp);
    void getNowTimeStamp(long int& nowTimeStamp);
    void getDeltaTime(long int& deltaTime);

private:
    //flag of initialization
    bool is_initialized_;

    //state vector
    Eigen::VectorXf x_;

    //state transistion matrix
    Eigen::MatrixXf F_;

    //state covariance matrix
    Eigen::MatrixXf P_;

    //process covariance matrix
    Eigen::MatrixXf Q_;

    //measurement matrix
    Eigen::MatrixXf H_;

    //measurement covariance matrix
    Eigen::MatrixXf R_;

    //TimeStamp
    long int lastTimeStamp_;
    long int nowTimeStamp_;
    long int deltaTime_;

};

extern WeightingFilter weightingFilter;
extern WeightingFilter* pweightingFilter;

extern KalmanFilter kalmanFilter;
extern KalmanFilter* pkalmanFilter;

#endif
