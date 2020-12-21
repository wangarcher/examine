#ifndef _FILTER_H
#define _FILTER_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>

#include "../include/Robot.h"
#include "../include/Mtime.h"


const double W = 0.5 / 3;
const double W0 = -4 / double(3);
const double WEIGHTS[15] = {W0, W, W, W, W, W, W, W, W, W, W, W, W, W, W};

class UnscentedKalmanFilter
{
public:
    // Constructor
    UnscentedKalmanFilter();

    //Destructor
    ~UnscentedKalmanFilter();

    void Initialization();
    
    bool GetIsInitialized();

    void SetX(Eigen::VectorXf x_in);

    void SetP(Eigen::MatrixXf P_in);

    void SetPreSigmaX(Eigen::MatrixXf PreSigmaX_in);

    void SetPreSigmaZ(Eigen::MatrixXf PreSigmaZ_in);

    void SetR(Eigen::MatrixXf R_in);

    void SetZ(Eigen::VectorXf z_in);
    
    void SetS(Eigen::MatrixXf S_in);

    void StatePrediction(float d_t);//state prediction

    void MeasurementPrediction();//measurement prediction

    void StateUpdate(Eigen::VectorXf z_new);//state update


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

    //state covariance matrix
    Eigen::MatrixXf P_;

    //predicted state sigma
    Eigen::MatrixXf PreSigmaX_;

    //predicted measurement sigma
    Eigen::MatrixXf PreSigmaZ_;

    //
    Eigen::VectorXf z_;

    //measurement covariance matrix
    Eigen::MatrixXf R_;


    Eigen::MatrixXf S_;

    //TimeStamp
    long int lastTimeStamp_;
    long int nowTimeStamp_;
    long int deltaTime_;

};

extern UnscentedKalmanFilter unscentedKalmanFilter;
extern UnscentedKalmanFilter* punscentedKalmanFilter;

#endif
