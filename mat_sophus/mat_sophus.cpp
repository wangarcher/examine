/*

Project: From rotation matrix to sophus
Date: 20/08/03
@Author: Wang
Detail: To solve the problem in rotation matrix criteria. When: RtR != I ,and det(R) != 1
Scenario: Graph optimization

*/


#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;


typedef Eigen::Matrix<double, 6, 1> Vector6d;
//typedef Eigen::Matrix<float, 3, 3> Matrix3f;
//typedef Eigen::Matrix<float, 3, 1> Vector3f;

void mat_sophus(const Eigen::Matrix4f&raw_transform, Vector6d&se3)
{

    Matrix4d transform = raw_transform.cast <double> ();   // Matrix of floats.
    Matrix3d R;
    Vector3d t; 
    R = transform.block<3,3>(0,0);
    t = transform.block<3,1>(0,3);
    Quaterniond q(R);                //turn it in to quaterniond
    Sophus::SE3d SE3_qt(q, t);       //create the quaterniond from q and t  
    se3 = SE3_qt.log();     //func log to make it in to se3
    std::cout << "SE3 from q,t= \n" << SE3_qt.matrix() << std::endl; //test
    std::cout << "se3 = " << se3.transpose() << std::endl;  //test
}

//awful try
void mat_sophus2(const Eigen::Matrix4f&raw_transform, Vector6d&se3)
{

    Matrix4d transform = raw_transform.cast <double> ();   // Matrix of floats.
    Matrix3d R;
    Vector3d t; 
    R = transform.block<3,3>(0,0);
    t = transform.block<3,1>(0,3);
    //Quaterniond q(R);                //turn it in to quaterniond
    //cout << q(R) << endl;
    //double trace = R.trace();
    //Vector4d q(0,0,0,0);
    //q(0) = (1+trace)/2;
/*
    double max_q_val = q(0);
    int index = 0;
    for(size_t i = 1; i < 3; i++)
    {
        q(i) = (1 - trace + R(i-1,i-1))/4;
        if(q(i) > max_q_val)
        {
            max_q_val = q(i);
	    index = i;
        }
    }
    switch(index) 
    {
        case 0:
        q(1) = (R(2,1)-R(1,2))/4*q(0);
        q(2) = (R(0,2)-R(2,0))/4*q(0);
        q(3) = (R(1,0)-R(0,1))/4*q(0);
        break;
        case 1:
        q(0) = (R(2,1)-R(1,2))/4*q(1);
        q(2) = (R(1,0)+R(0,1))/4*q(1);
        q(3) = (R(0,2)+R(2,0))/4*q(1);
        break;
        case 2:
        q(0) = (R(0,2)-R(2,0))/4*q(2);
        q(1) = (R(0,1)+R(1,0))/4*q(2);
        q(3) = (R(2,1)+R(1,2))/4*q(2);
        break;
        case 3:
        q(0) = (R(1,0)-R(0,1))/4*q(3);
        q(1) = (R(0,2)+R(2,0))/4*q(3);
        q(2) = (R(2,1)+R(1,2))/4*q(3);
        break;
    }
*/
    Sophus::SE3d SE3_Rt(R, t);          //create the quaterniond from q and t  
    se3 = SE3_Rt.log();                 //func log to make it in to se3
    std::cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << std::endl; //test
    std::cout << "se3 = " << se3.transpose() << std::endl;  //test
}


int main(int argc, char **argv) {
    Vector6d final_se3;  //func log to make it in to se3
    Matrix4f transform;
    transform << 0.999711, -0.0241033, 0.000327854, 0.00065927, 0.0241039, 0.999712, -0.000496294,0.0522064 , -0.000315792, -0.000504087, 1, 0.000850175, 0, 0, 0, 1;

    mat_sophus(transform, final_se3);
    std::cout << transform << std::endl;  //test
    return 0;
}
