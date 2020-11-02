#include <iostream>
#include <cmath>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "../include/Robot.h"
#include "../include/Gps.h"
#include "../include/Imu.h"
#include "../include/Odom.h"

Robot robot;
Robot* probot = new Robot();

Robot::Robot()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->Roll = 0;
    this->Pitch = 0;
    this->Yaw = 0;
}

Robot::~Robot()
{
    delete probot;
}

void Robot::SetRobotPosition(float x_, float y_, float z_)
{
    this->x = x_;
    this->y = y_;
    this->z = z_;
}

void Robot::SetRobotVelocity(float vx_, float vy_, float vz_)
{
    this->vx = vx_;
    this->vy = vy_;
    this->vz = vz_;
}

void Robot::SetRobotRotation(float Roll_, float Pitch_, float Yaw_)
{
    this->Roll = Roll_;
    this->Pitch = Pitch_;
    this->Yaw = Yaw_;
}

void Robot::GetRobotPosition(float &x_, float &y_, float &z_)
{
    x_ = this->x;
    y_ = this->y;
    z_ = this->z;
}

void Robot::GetRobotVelocity(float &vx_, float &vy_, float &vz_)
{
    vx_ = this->vx;
    vy_ = this->vy;
    vz_ = this->vz;
}

void Robot::GetRobotRotation(float &Roll_, float &Pitch_, float &Yaw_)
{
    Roll_ = this->Roll;
    Pitch_ = this->Pitch;
    Yaw_ = this->Yaw;
}



//TODO 新的数据结构  重构代码
// void Robot::SetRobotPosition(const Eigen::Vector3f& position_in)
// {
//     position = position_in;
// }

// Eigen::Vector3f Robot::GetRobotPosition()
// {
//     return position;
// }

// void Robot::SetRobotVelocity(const Eigen::Vector3f& velocity_in)
// {
//     velocity = velocity_in;
// }

// Eigen::Vector3f Robot::GetRobotVelocity()
// {
//     return velocity;
// } 

// void Robot::SetRobotRotation(const Eigen::Vector3f& rotation_in)
// {
//     rotation = rotation_in;
// }

// Eigen::Vector3f Robot::GetRobotRotation()
// {
//     return rotation;
// }




