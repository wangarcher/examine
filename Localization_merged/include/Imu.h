#pragma once
#ifndef _IMU_H
#define _IMU_H

//----------------------------------------------------------------------------------------------------
// Definitions
#define Kp 10.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f		// integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.0025f		// half the sample period
#define halfT 0.025f		// half the sample period

#define ImuDT 0.01f

typedef struct 
{
    float gravity_x;    //陀螺仪三轴分量   角加速度
    float gravity_y;
    float gravity_z;
}Gravity;

typedef struct 
{
    float accelerate_x;  //速度计  加速度
    float accelerate_y;
    float accelerate_z;
}Accelerate;

typedef struct 
{

}Magnetometer;

typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;
}Quaternion;

typedef struct 
{
    float x;
    float y;
    float theta;
}ImuPosition;

typedef struct 
{
    float Roll; //翻滚角
    float Pitch; //俯仰角
    float Yaw; //偏航角
}ImuPosture;

typedef struct speed
{
    float vx;
    float vy;
    float w;
}ImuSpeed;


class ImuData
{
private:
    Gravity     Gyro;
    Accelerate  Accelerometer;
    Quaternion  Quaternionelement;
    ImuPosture  PostureYPR;
    ImuPosition Position;     
    ImuSpeed    Speed;
    Accelerate  LastAccelerometer;
	ImuPosture  UpDateBenchmarkYPR;
public:
    ImuData();
    ~ImuData();

    static void* ReadImuData(void *arg);

    int  InitImu();

    int  StartReadImuDataThread();

    void MahonyAHRSUpdate(); //使用磁力计扩展

    void ImuUpdate(float _gravity_x, float _gravity_y, float _gravity_z, float _accelerate_x, float _accelerate_y, float _accelerate_z); //得到角速度 加速度
    
    void ImuTrackDeduction(); //IMU航迹推演

    void ImuUpdateYaw(float yaw);

    void ClearQuaternion(void);

    void SetImuData(float _accelerate_x, float _accelerate_y, float _accelerate_z, float _gravity_x, float _gravity_y, float _gravity_z);

    void SetAccelerateXYZ(float _accelerate_x, float _accelerate_y, float _accelerate_z);

    void SetGravityXYZ(float _gravity_x, float _gravity_y, float _gravity_z);

    void SetPostureYPR(float _Roll, float _Pitch, float _Yaw);

    void SetPosition(float _x, float _y, float _theta);

    void SetSpeed(float _vx, float _vy, float _w);

    void SetLastAccelerometer(float _accelerate_x, float _accelerate_y, float _accelerate_z);
    
    void GetImuData(float &_accelerate_x, float &_accelerate_y, float &_accelerate_z, float &_gravity_x, float &_gravity_y, float &_gravity_z);

    void GetAccelerateXYZ(float &_accelerate_x, float &_accelerate_y, float &_accelerate_z);

    void GetGravityXYZ(float &_gravity_x, float &_gravity_y, float &_gravity_z);

    void GetPostureYPR(float &_Roll, float &_Pitch, float &_Yaw);

    void GetPosition(float &_x, float &_y, float &_theta);

    void GetSpeed(float &_vx, float &_vy, float &_w);

    void GetLastAccelerometer(float &_accelerate_x, float &_accelerate_y, float &_accelerate_z);
};

extern ImuData imu;
extern ImuData* pimu;

#endif
