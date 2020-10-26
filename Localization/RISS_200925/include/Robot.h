#ifndef _ROBOT_H
#define _ROBOT_H

#include <Eigen/Eigen>
#include <Eigen/Dense>

typedef struct 
{
	float x;
	float y;
	float z;
    float roll; //翻滚角
    float pitch; //俯仰角
    float yaw; //偏航角
}RobotPos; 

class Robot
{
private:

    float x; //位置
    float y;
    float z;

    float vx; //速度
    float vy;
    float vz;
    
    float Roll; //翻滚角
    float Pitch; //俯仰角
    float Yaw; //偏航角

    //Eigen::Vector3f position; //位置
    //Eigen::Vector3f velocity; //速度
    //Eigen::Vector3f rotation; //旋转

public:
    Robot();
    ~Robot();
    
    //void SetRobotPosition(const Eigen::Vector3f &position_in);
    //Eigen::Vector3f GetRobotPosition();

    //void SetRobotVelocity(const Eigen::Vector3f &velocity_in);
    //Eigen::Vector3f GetRobotVelocity();

    //void SetRobotRotation(const Eigen::Vector3f &rotation_in);
    //Eigen::Vector3f GetRobotRotation();

    void SetRobotPosition(float x_, float y_, float z_);
    void SetRobotVelocity(float vx_, float vy_, float vz_);
    void SetRobotRotation(float Roll_, float Pitch_, float Yaw_);

    void GetRobotPosition(float &x_, float &y_, float &z_);
    void GetRobotVelocity(float &vx_, float &vy_, float &vz_);
    void GetRobotRotation(float &Roll_, float &Pitch_, float &Yaw_);
};

extern Robot robot;
extern Robot* probot;

#endif
