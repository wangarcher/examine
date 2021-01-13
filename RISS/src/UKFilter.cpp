#include <iostream>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../include/UKFilter.h"
#include "../include/Robot.h"
#include "../include/Odom.h"
#include "../include/Imu.h"
#include "../include/Gps.h"

UnscentedKalmanFilter unscentedKalmanFilter;
UnscentedKalmanFilter *punscentedKalmanFilter;

// Constructor
UnscentedKalmanFilter::UnscentedKalmanFilter()
{
    is_initialized_ = false;
    lastTimeStamp_ = 0;
    nowTimeStamp_ = 0;
    deltaTime_ = 0;
}

//Destructor
UnscentedKalmanFilter::~UnscentedKalmanFilter()
{
    delete punscentedKalmanFilter;
}

void UnscentedKalmanFilter::Initialization()
{
    lastTimeStamp_ = nowTimeStamp_ = getSysTime();
    
    Eigen::VectorXf x_in = Eigen::VectorXf::Zero(5,1);
    SetX(x_in);

    // state covariance matrix the prediction error
    Eigen::MatrixXf P_in(5,5);
    P_in << 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1;
    SetP(P_in);
    
    //predicted sigma
    Eigen::MatrixXf PreSigmaX_in = Eigen::MatrixXf::Zero(5, 15);
    SetPreSigmaX(PreSigmaX_in);

    //measurement covariance matrix
    // R is provided by Sensor supplier
    Eigen::MatrixXf R_in(3,3);
    R_in << 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1;
    SetR(R_in);

    //Observation
    Eigen::VectorXf z_in(3,1);
    z_in << 0.0,
            0.0, 
            0.0;
    SetZ(z_in);


    is_initialized_ = true;

}

//state prediction
//state defination: x, y, v, yaw, w
void UnscentedKalmanFilter::StatePrediction(float d_t)
{
    Eigen::MatrixXf augmented_sigma = Eigen::MatrixXf::Zero(7, 15);
    Eigen::VectorXf augmented_x = Eigen::VectorXf::Zero(7);
    Eigen::MatrixXf augmented_P = Eigen::MatrixXf::Zero(7, 7);
    augmented_x.head(5) = x_;
    augmented_P.topLeftCorner(5, 5) = P_;
    augmented_P(5, 5) = 0.1;
    augmented_P(6, 6) = 0.1;
    const Eigen::MatrixXf L = augmented_P.llt().matrixL();
    augmented_sigma.col(0) = augmented_x;
    for(int c = 0; c < 7; c++)
    {
        const int i = c + 1;
        augmented_sigma.col(i)  = augmented_x + 1.732 * L.col(c);
        augmented_sigma.col(i + 7)  = augmented_x - 1.732 * L.col(c);
    }

    Eigen::MatrixXf predicted_sigma = Eigen::MatrixXf(5, 15);
    for(int c = 0; c < 15; ++c)
    {
        /*************************************
        * Get the current state
        *************************************/
        const float px = augmented_sigma(0, c);
        const float py = augmented_sigma(1, c);
        const float speed = augmented_sigma(2, c);
        const float yaw = augmented_sigma(3, c);
        const float yawrate = augmented_sigma(4, c);
        const float speed_noise = augmented_sigma(5, c);
        const float yawrate_noise = augmented_sigma(6, c);
 
        /*************************************
        * predict the next state with noise
        * USING THE CTRV MODEL
        *************************************/
        const float cos_yaw = cos(yaw);
        const float sin_yaw = sin(yaw);
        const float d_t2 = d_t * d_t;
    
        // predicted position noise
        const float p_noise = 0.5 * speed_noise * d_t2;
        // predicted yaw noise
        const float y_noise = 0.5 * yawrate_noise * d_t2;
        const float dyaw = yawrate * d_t; //change in yaw
        const float dspeed = speed * d_t; //change in speed
  
        // predicted speed = assumed constant speed + noise
        const float p_speed = speed + speed_noise * d_t;
    
        // predicted yaw
        const float p_yaw = yaw + dyaw + y_noise;
        // predicted yaw rate = assumed constant yawrate + noise
        const float p_yawrate = yawrate + yawrate_noise * d_t;
        // where predicted positions will be stored
        float p_px, p_py;

        if(fabs(yawrate) <= 0.001) 
        {
            // moving straight
            p_px = px + dspeed * cos_yaw + p_noise * cos_yaw;
            p_py = py + dspeed * sin_yaw + p_noise * sin_yaw;

        }
        else
        {
            const float k = speed / yawrate;
            const float theta = yaw + dyaw;
            p_px = px + k * (sin(theta) - sin_yaw) + p_noise * cos_yaw;
            p_py = py + k * (cos_yaw - cos(theta)) + p_noise * sin_yaw ;
        }

        /*************************************
        * Write the prediction to the appropriate column
        *************************************/
        predicted_sigma(0, c) = p_px;
        predicted_sigma(1, c) = p_py;
        predicted_sigma(2, c) = p_speed;
        predicted_sigma(3, c) = p_yaw;
        predicted_sigma(4, c) = p_yawrate;
    }

    Eigen::VectorXf predicted_x = Eigen::VectorXf::Zero(5);
    for(int c = 0; c < 15; c++)
    {
        predicted_x += WEIGHTS[c] * predicted_sigma.col(c);
    }
    Eigen::MatrixXf predicted_P = Eigen::MatrixXf::Zero(5, 5);
    Eigen::VectorXf dx = Eigen::VectorXf(5);
    for(int c = 0; c < 15; c++)
    {
        dx = predicted_sigma.col(c) - predicted_x;
        dx(3) = (fabs(dx(3)) > M_PI) ? remainder(dx(3), 2. * M_PI) : dx(3);
        predicted_P += WEIGHTS[c] * dx * dx.transpose();
    }
    SetPreSigmaX(predicted_sigma);
    SetX(predicted_x);
    SetP(predicted_P);
}

void UnscentedKalmanFilter::MeasurementPrediction()//measurement prediction
{
    Eigen::MatrixXf sigma = Eigen::MatrixXf::Zero(3, 15);
    for(int c = 0; c < 15; c++)
    {
        const float px = PreSigmaX_(0, c);
        const float py = PreSigmaX_(1, c);
        const float v = PreSigmaX_(2, c);
        const float yaw = PreSigmaX_(3, c);

        const float vx = cos(yaw) * v;
        const float vy = sin(yaw) * v;

        const float rho = sqrt(px * px + py * py);
        const float phi = atan2(py, px);
        const float rhodot = (rho > 0.0001) ? ((px * vx + py * vy) / rho) : 0.0; 

        // avoid division by zero
        sigma(0, c) = rho;
        sigma(1, c) = phi;
        sigma(2, c) = rhodot;
    }

    Eigen::VectorXf z = Eigen::VectorXf::Zero(3);
    Eigen::VectorXf dz;
    Eigen::MatrixXf S = Eigen::MatrixXf::Zero(3, 3);
    for(int c = 0; c < 15; c++){
        z += WEIGHTS[c] * sigma.col(c);
    }
    for(int c = 0; c < 15; c++)
    {
        dz = sigma.col(c) - z;
        dz(1) =  (fabs(dz(1)) > M_PI) ? remainder(dz(1), 2. * M_PI) : dz(1);
        S += WEIGHTS[c] * dz * dz.transpose();
    }
    S += R_;

    SetPreSigmaZ(sigma);
    SetZ(z);
    SetS(S);
}


void UnscentedKalmanFilter::StateUpdate(Eigen::VectorXf z_new)
{
    Eigen::VectorXf dz;
    Eigen::VectorXf dx;
    Eigen::MatrixXf Tc = Eigen::MatrixXf::Zero(5, 3);

    for(int c = 0; c < 15; c++)
    {
        dx = PreSigmaX_.col(c) - x_;
        dx(3) = (fabs(dx(3)) > M_PI) ? remainder(dx(3), 2. * M_PI) : dx(3);
        dz = PreSigmaZ_.col(c) - z_;
        dz(1) = (fabs(dz(1)) > M_PI) ? remainder(dz(1), 2. * M_PI) : dz(1);
        Tc += WEIGHTS[c] * dx * dz.transpose();
    }
    Eigen::MatrixXf Si = S_.inverse();
    Eigen::MatrixXf K = Tc * Si;
    Eigen::VectorXf dz = z_new - z_;
    dz(1) = (fabs(dz(1)) > M_PI) ? remainder(dz(1), 2. * M_PI) : dz(1);

    SetX(x_ + K * dz);
    SetP(P_ - K * S_ * K.transpose());
}

bool UnscentedKalmanFilter::GetIsInitialized()
{
    return is_initialized_;
}

void UnscentedKalmanFilter::SetX(Eigen::VectorXf x_in)
{
    x_ = x_in;
}

void UnscentedKalmanFilter::SetP(Eigen::MatrixXf P_in)
{
    P_ = P_in;
}

void UnscentedKalmanFilter::SetPreSigmaX(Eigen::MatrixXf PreSigmaX_in)
{
    PreSigmaX_ = PreSigmaX_in;
}

void UnscentedKalmanFilter::SetPreSigmaZ(Eigen::MatrixXf PreSigmaZ_in)
{
    PreSigmaZ_ = PreSigmaZ_in;
}

void UnscentedKalmanFilter::SetZ(Eigen::VectorXf z_in)
{
    z_ = z_in;
}

void UnscentedKalmanFilter::SetR(Eigen::MatrixXf S_in)
{
    S_ = S_in;
}

void UnscentedKalmanFilter::SetR(Eigen::MatrixXf R_in)
{
    R_ = R_in;
}

void UnscentedKalmanFilter::setLastTimeStamp(const long int lastTimeStamp)
{
    this->lastTimeStamp_ = lastTimeStamp;
}

void UnscentedKalmanFilter::setNowTimeStamp(const long int nowTimeStamp )
{
    this->nowTimeStamp_ = nowTimeStamp;
}
void UnscentedKalmanFilter::setDeltaTime(const long int deltaTime)
{
    this->deltaTime_ = deltaTime;
}

void UnscentedKalmanFilter::getLastTimeStamp(long int& lastTimeStamp)
{
    lastTimeStamp = this->lastTimeStamp_;
}
void UnscentedKalmanFilter::getNowTimeStamp(long int& nowTimeStamp)
{
    nowTimeStamp = this->nowTimeStamp_;
}
void UnscentedKalmanFilter::getDeltaTime(long int& deltaTime)
{
    deltaTime = this->deltaTime_;
}