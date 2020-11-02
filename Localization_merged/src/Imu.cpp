#include <iostream>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <cmath>
#include <math.h>

#include "../include/Imu.h"
#include "../include/Mtime.h"
#include "../include/UartCom.h"

ImuData imu;
ImuData* pimu = new ImuData();

int Imufd;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


ImuData::ImuData()
{
	float x_,y_,theta_;
    //陀螺仪初始化
    this->Gyro.gravity_x = 0;
    this->Gyro.gravity_y = 0;
    this->Gyro.gravity_z = 0;

    //加速度计初始化
    this->Accelerometer.accelerate_x = 0;
    this->Accelerometer.accelerate_y = 0;
    this->Accelerometer.accelerate_z = 0;
	
	this->UpDateBenchmarkYPR.Roll = 0;
	this->UpDateBenchmarkYPR.Pitch = 0;
	this->UpDateBenchmarkYPR.Yaw = 0;
}

ImuData::~ImuData()
{
    
}

int ImuData::InitImu()
{
    int ret = 0;

	//******************UDP通信初始化**************************//
    Imufd = open("/dev/ttyTHS4",O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(Imufd <= 0)
    {
        std::cout << "Fail to open /dev/ttyTHS4!" << std::endl;
        return -1;
    }
    ret = UART_Init_Baud(Imufd,230400);
    //******************UDP通信初始化**************************//
    //******************IMU线程初始化**************************//
    imu.StartReadImuDataThread();
    //******************IMU线程初始化**************************//
}

void* ImuData::ReadImuData(void *arg)
{
    unsigned char imuDataBuf[120];

    union chartofloat cti;

	float X_GYRO_OUT,Y_GYRO_OUT,Z_GYRO_OUT,X_ACCL_OUT,Y_ACCL_OUT,Z_ACCL_OUT;//惯导数据
	//float Frist_X_GYRO_OUT=0.0,Frist_Y_GYRO_OUT=0.0,Frist_Z_GYRO_OUT=0.0,Frist_X_ACCL_OUT=0.0,Frist_Y_ACCL_OUT=0.0,Frist_Z_ACCL_OUT=0.0;//惯导数据
    
    int imuDataLength = 0;

    int imuDataStartFlagIdx = 0;

    while(1)
    {
        usleep(5000);
        imuDataLength = read(Imufd,imuDataBuf,120);
  
        //读取数据长度小于58字节
        if(imuDataLength < 58)
        {
            continue;
        }

	    imuDataStartFlagIdx = 0;
    
        while((imuDataBuf[imuDataStartFlagIdx]!=0x5a)||(imuDataBuf[ imuDataStartFlagIdx + 1]!=0x5a))
        {
            imuDataStartFlagIdx++;

            if( imuDataLength < imuDataStartFlagIdx + 58) //没有读到完整的一包
            {
                continue;
            }
        }

        if(GET_IMU_CRC(&imuDataBuf[imuDataStartFlagIdx+2],56) == imuDataBuf[imuDataStartFlagIdx +58])
        {
            for(int idx = 0; idx < 4; idx++)
            cti.ch[idx] = imuDataBuf[imuDataStartFlagIdx+2+idx];

            X_GYRO_OUT = cti.fl * 3.1415 / 360;

            for(int idx = 0; idx < 4; idx++)
            cti.ch[idx] = imuDataBuf[imuDataStartFlagIdx + 6 + idx];

            Y_GYRO_OUT = cti.fl * 3.1415 / 360;

            for(int idx = 0; idx < 4; idx++)
            cti.ch[idx] = imuDataBuf[imuDataStartFlagIdx + 10 + idx];

            Z_GYRO_OUT = cti.fl * 3.1415 / 360;

            for(int idx = 0; idx < 4; idx++)
            cti.ch[idx] = imuDataBuf[imuDataStartFlagIdx + 14 + idx];
	    
            X_ACCL_OUT = cti.fl * 9.80665;

            for(int idx = 0; idx < 4; idx++)
            {
                    cti.ch[idx] = imuDataBuf[imuDataStartFlagIdx + 18 + idx];
            }

            Y_ACCL_OUT = cti.fl * 9.80665;

            for(int idx = 0; idx < 4; idx++)
            {
                    cti.ch[idx] = imuDataBuf[imuDataStartFlagIdx + 22 + idx];
            }
            Z_ACCL_OUT = cti.fl * 9.80665;
	

		if( fabs(X_ACCL_OUT) > 3.0*9.80665)
			continue;
		if( fabs(Y_ACCL_OUT) > 3.0*9.80665)
			continue;
		if( fabs(Z_ACCL_OUT) > 3.0*9.80665)
			continue;
       		 if( fabs(X_GYRO_OUT) > 3.0*9.80665)
			continue;
		if( fabs(Y_GYRO_OUT) > 3.0*9.80665)
			continue;
		if( fabs(Z_GYRO_OUT) > 3.0*9.80665)
			continue;

          //  std::cout<<"****************first********************"<<std::endl;
         //   std::cout<<"X_GYRO_OUT="<<X_GYRO_OUT<<std::endl;
        //    std::cout<<"Y_GYRO_OUT="<<Y_GYRO_OUT<<std::endl;
        //    std::cout<<"Z_GYRO_OUT="<<Z_GYRO_OUT<<std::endl;
        //    std::cout<<"X_ACCL_OUT="<<X_ACCL_OUT<<std::endl;
         //   std::cout<<"Y_ACCL_OUT="<<Y_ACCL_OUT<<std::endl;
        //    std::cout<<"Z_ACCL_OUT="<<Z_ACCL_OUT<<std::endl;
		

            ImuPosition positionTmp;
            ImuPosture  postureTmp;
            ImuSpeed    speedTmp;
  
            //目前读取数据与数据处理处于统一线程
            //(1)记录原始数据
            //(2)采用互补滤波更新数据
            //(3)IMU航迹推测
            //(4)验证数据是否正确
            imu.SetImuData(X_ACCL_OUT,Y_ACCL_OUT,Z_ACCL_OUT,X_GYRO_OUT,Y_GYRO_OUT,Z_GYRO_OUT);  
            imu.ImuUpdate(X_GYRO_OUT, Y_GYRO_OUT, Z_GYRO_OUT, X_ACCL_OUT, Y_ACCL_OUT, Z_ACCL_OUT);
            imu.ImuTrackDeduction();

		    imu.GetPostureYPR(postureTmp.Pitch, postureTmp.Roll, postureTmp.Yaw);
		    imu.GetPosition(positionTmp.x,positionTmp.y,positionTmp.theta);
		    imu.GetSpeed(speedTmp.vx,speedTmp.vy,speedTmp.w);

         //  std::cout<<"***************Position****************"<<std::endl;
        //    std::cout<<"position_cout.x="<<positionTmp.x<<std::endl;
       //    std::cout<<"position_cout.y="<<positionTmp.y<<std::endl;
           // std::cout<<"position_cout.z="<<positionTmp.z<<std::endl;
        //    std::cout<<"position_cout.theta="<<positionTmp.theta<<std::endl;
        //    std::cout<<"**************Posture*****************"<<std::endl;
        //    std::cout<<"posture_cout.Roll="<<postureTmp.Roll<<std::endl;
        //    std::cout<<"posture_cout.Pitch="<<postureTmp.Pitch<<std::endl;
        //    std::cout<<"posture_cout.Yaw="<<postureTmp.Yaw<<std::endl;
      //      std::cout<<"***************Speed*******************"<<std::endl;
       //     std::cout<<"speed_cout.vx="<<speedTmp.vx<<std::endl;
      //      std::cout<<"speed_cout.vy="<<speedTmp.vy<<std::endl;
       //     std::cout<<"speed_cout.w="<<speedTmp.w<<std::endl;	
	
        }
    }
}

int  ImuData::StartReadImuDataThread()
{
    pthread_t readImuThread;
    pthread_create(&readImuThread, NULL, ReadImuData, (void*)this);
}

void ImuData::MahonyAHRSUpdate()
{
    //if()
}

void ImuData::ImuUpdate(float _gravity_x, float _gravity_y, float _gravity_z,float _accelerate_x, float _accelerate_y, float _accelerate_z)
{
	float yaw_deviation[] = {0.0,180,-180.0,0.0};
	static char yaw_flag = 0;
	static float yaw_last = 0.0;
    float norm;
	float vx, vy, vz;
	float ex, ey, ez; 
	
    if( 0 == _accelerate_x )
	{
		_accelerate_x = 0.000001;
	}
	if( 0 == _accelerate_y )
	{
		_accelerate_y = 0.000001;
	}
	if( 0 == _accelerate_z )
	{
		_accelerate_z = 0.000001;
	}
  //  std::cout << "old  accelerate_x accelerate_y accelerate_z:" << " " << this->Accelerometer.accelerate_x << " "<< this->Accelerometer.accelerate_y << " " << this->Accelerometer.accelerate_z << " "<< std::endl;

    // normalise the measurements
    norm = invSqrt( pow(_accelerate_x, 2) + pow(_accelerate_y, 2) + pow(_accelerate_z, 2));
   // std::cout << "norm = " << norm << std::endl;
	if( fabs(norm) < 0.000001)
	{
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
		norm = 0.00000001;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
	}
    else if( fabs(norm) > 1000)
	{
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
		norm = 1.0;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
	}
    this->Accelerometer.accelerate_x = _accelerate_x * norm;
    this->Accelerometer.accelerate_y = _accelerate_y * norm;
    this->Accelerometer.accelerate_z = _accelerate_z * norm;

  //  std::cout << "new accelerate_x accelerate_y accelerate_z:" << " " << this->Accelerometer.accelerate_x << " "<< this->Accelerometer.accelerate_y << " " << this->Accelerometer.accelerate_z << " "<< std::endl;

	
	
    // estimated direction of gravity 重力方向
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = pow(q0, 2) - pow(q1, 2) - pow(q2, 2) + pow(q3, 2); 

    // error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (this->Accelerometer.accelerate_y * vz - this->Accelerometer.accelerate_z * vy);
	ey = (this->Accelerometer.accelerate_z * vx - this->Accelerometer.accelerate_x * vz);
	ez = (this->Accelerometer.accelerate_x * vy - this->Accelerometer.accelerate_y * vx);


	if(ex !=0.0f && ey != 0.0f && ez != 0.0f)
	{
    // integral error scaled integral gain
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;


    // adjusted gyroscope measurements
		this->Gyro.gravity_x = this->Gyro.gravity_x + Kp*ex + exInt;
		this->Gyro.gravity_y = this->Gyro.gravity_y + Kp*ey + eyInt;
		this->Gyro.gravity_z = this->Gyro.gravity_z + Kp*ez + ezInt;
	}
	static long int LastTime = 0;
	long int NowTime=getSysTime();
	if( 0 == LastTime )
	{
		LastTime = NowTime +10;
	}
	long int  DeltaTime = NowTime - LastTime ;
	LastTime = NowTime;
	//std::cout<<"IMU DeltaTime="<<DeltaTime<<std::endl;
	
    // integrate quaternion rate and normalise  一阶龙格库塔法
	//q0 = q0 + (-q1 * this->Gyro.gravity_x  - q2 * this->Gyro.gravity_y - q3 * this->Gyro.gravity_z) * (DeltaTime /1000.0);
	//q1 = q1 + (q0 * this->Gyro.gravity_x + q2 * this->Gyro.gravity_z - q3 * this->Gyro.gravity_y) * (DeltaTime /1000.0);
	//q2 = q2 + (q0 * this->Gyro.gravity_y - q1 * this->Gyro.gravity_z + q3 * this->Gyro.gravity_x) * (DeltaTime /1000.0);
	//q3 = q3 + (q0 *this->Gyro.gravity_z + q1 * this->Gyro.gravity_y - q2 * this->Gyro.gravity_x) * (DeltaTime /1000.0);    
// integrate quaternion rate and normalise  一阶龙格库塔法
	q0 = q0 + (-q1 * this->Gyro.gravity_x  - q2 * this->Gyro.gravity_y - q3 * this->Gyro.gravity_z) * halfT;
	q1 = q1 + (q0 * this->Gyro.gravity_x + q2 * this->Gyro.gravity_z - q3 * this->Gyro.gravity_y) * halfT;
	q2 = q2 + (q0 * this->Gyro.gravity_y - q1 * this->Gyro.gravity_z + q3 * this->Gyro.gravity_x) * halfT;
	q3 = q3 + (q0 *this->Gyro.gravity_z + q1 * this->Gyro.gravity_y - q2 * this->Gyro.gravity_x) * halfT; 

  //  std::cout << "q0 q1 q2 q3 11111:" << " " << q0 << " "<< q1 << " " << q2 << " " << q3 << std::endl;
 
	
    // normalise quaternion 归一化
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   // std::cout << "norm = " << norm << std::endl;
	if( fabs(norm) < 0.000001)
	{
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
		norm = 0.00000001;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "***************************************" << std::endl;
	}
	q0 = q0 * norm;	
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

 //   std::cout << "q0 q1 q2 q3 222222:" << " " << q0 << " "<< q1 << " " << q2 << " " << q3 << std::endl;


    //求去姿态角
    this->PostureYPR.Pitch  = this->UpDateBenchmarkYPR.Pitch + asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; //偏航角 pitch ,转换为度数
    this->PostureYPR.Roll = this->UpDateBenchmarkYPR.Roll + atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
    
   // this->PostureYPR.Yaw = atan2(2*(q1*q2 - q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3; 
    this->PostureYPR.Yaw = this->UpDateBenchmarkYPR.Yaw + atan2(q1 * q2 - q0 * q3,q2 * q2 + q0 * q0) * 57.3; 
    //this->PostureYPR.Yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 57.3; 
	
	if(( this->PostureYPR.Yaw  < 0 ) && ( this->PostureYPR.Yaw > -30 ) && ( 0 == yaw_flag ))
	{
		yaw_flag = 3;
	}
	else if((this->PostureYPR.Yaw < -60) && ( 0 == yaw_flag  ))
	{
		yaw_flag = 1;
	}
	else if((this->PostureYPR.Yaw > 60) && ( 1 == yaw_flag  ))
	{
		yaw_flag = 0;
	}
	else if((this->PostureYPR.Yaw > 0) && (this->PostureYPR.Yaw < 30 ) && ( 1 == yaw_flag ))
	{
		yaw_flag = 2;
	}
	else if((this->PostureYPR.Yaw < 0) && (this->PostureYPR.Yaw > -30) && ( 2 == yaw_flag ))
	{
		yaw_flag = 1;
	}
	else if((this->PostureYPR.Yaw < -60) && ( 2 == yaw_flag ))
	{
		yaw_flag = 3;
	}
	else if((this->PostureYPR.Yaw > 60) && ( 3 == yaw_flag ))
	{
		yaw_flag = 2;
	}
	else if((this->PostureYPR.Yaw > 0) && (this->PostureYPR.Yaw < 30) && ( 3 == yaw_flag ))
	{
		yaw_flag = 0;
	}
	this->PostureYPR.Yaw += yaw_deviation[yaw_flag];
	if(this->PostureYPR.Yaw > 180)
	{
		this->PostureYPR.Yaw = this->PostureYPR.Yaw-360;
		yaw_flag = 2;
	}
	else if(this->PostureYPR.Yaw < -180)
	{
		this->PostureYPR.Yaw = 360 + this->PostureYPR.Yaw;	
		yaw_flag = 1;
	}
	//printf("yaw_flag=%d\n",yaw_flag&0xff);
}


void ImuData::SetImuData(float _accelerate_x, float _accelerate_y, float _accelerate_z, float _gravity_x, float _gravity_y, float _gravity_z)
{
    this->Accelerometer.accelerate_x = _accelerate_x;
    this->Accelerometer.accelerate_y = _accelerate_y;
    this->Accelerometer.accelerate_z = _accelerate_z;

    this->Gyro.gravity_x = _gravity_x;
    this->Gyro.gravity_y = _gravity_y;
    this->Gyro.gravity_z = _gravity_z;
}

void ImuData::SetAccelerateXYZ(float _accelerate_x, float _accelerate_y, float _accelerate_z)
{
    this->Accelerometer.accelerate_x = _accelerate_x;
    this->Accelerometer.accelerate_y = _accelerate_y;
    this->Accelerometer.accelerate_z = _accelerate_z;
}

void ImuData::SetGravityXYZ(float _gravity_x, float _gravity_y, float _gravity_z)
{
    this->Gyro.gravity_x = _gravity_x;
    this->Gyro.gravity_y = _gravity_y;
    this->Gyro.gravity_z = _gravity_z;
}

void ImuData::SetPostureYPR(float _Roll, float _Pitch, float _Yaw)
{
    this->PostureYPR.Roll = _Roll;
    this->PostureYPR.Pitch = _Pitch;
    this->PostureYPR.Yaw = _Yaw;
}

void ImuData::SetPosition(float _x, float _y, float _theta)
{
    this->Position.x = _x;
    this->Position.y = _y;
    this->Position.theta = _theta;
}

void ImuData::SetSpeed(float _vx, float _vy, float _w)
{
    this->Speed.vx = _vx;
    this->Speed.vy = _vy;
    this->Speed.w  = _w;
}

void ImuData::SetLastAccelerometer(float _accelerate_x, float _accelerate_y, float _accelerate_z)
{
    this->LastAccelerometer.accelerate_x = _accelerate_x;
    this->LastAccelerometer.accelerate_y = _accelerate_y;
    this->LastAccelerometer.accelerate_z = _accelerate_z;
}


void ImuData::GetImuData(float &_accelerate_x, float &_accelerate_y, float &_accelerate_z, float &_gravity_x, float &_gravity_y, float &_gravity_z)
{
    _accelerate_x = this->Accelerometer.accelerate_x;
    _accelerate_y = this->Accelerometer.accelerate_y;
    _accelerate_z = this->Accelerometer.accelerate_z;

    _gravity_x = this->Gyro.gravity_x;
    _gravity_y = this->Gyro.gravity_y;
    _gravity_z = this->Gyro.gravity_z;
}

void ImuData::GetAccelerateXYZ(float &_accelerate_x, float &_accelerate_y, float &_accelerate_z)
{
    _accelerate_x = this->Accelerometer.accelerate_x;
    _accelerate_y = this->Accelerometer.accelerate_y;
    _accelerate_z = this->Accelerometer.accelerate_z;
}

void ImuData::GetGravityXYZ(float &_gravity_x, float &_gravity_y, float &_gravity_z)
{
    _gravity_x = this->Gyro.gravity_x;
    _gravity_y = this->Gyro.gravity_y;
    _gravity_z = this->Gyro.gravity_z;
}

void ImuData::GetPostureYPR(float &_Roll, float &_Pitch, float &_Yaw)
{
    _Roll = this->PostureYPR.Roll;
    _Pitch = this->PostureYPR.Pitch;
    _Yaw = this->PostureYPR.Yaw;
}

void ImuData::GetPosition(float &_x, float &_y, float &_theta)
{
    _x = this->Position.x;
    _y = this->Position.y;
    _theta = this->Position.theta;
}

void ImuData::GetSpeed(float &_vx, float &_vy, float &_w)
{
    _vx = this->Speed.vx;
    _vy = this->Speed.vy;
    _w  = this->Speed.w;
}

void ImuData::GetLastAccelerometer(float &_accelerate_x, float &_accelerate_y, float &_accelerate_z)
{
    _accelerate_x = this->LastAccelerometer.accelerate_x; //世界坐标系上
    _accelerate_y = this->LastAccelerometer.accelerate_y;
    _accelerate_z = this->LastAccelerometer.accelerate_z;  
}

void ImuData::ImuTrackDeduction()
{
     float accelerate_x, accelerate_y, accelerate_z;
     float lastAccelerate_xInXY, lastAccelerate_yInXY,lastAccelerate_zInXY;
     float Roll, Pitch, Yaw;
     float last_vx,last_vy,last_w;
     float last_x,last_y,last_z,last_theta;

    //上一次测量的y轴与x轴的速度 所得夹角
    GetPostureYPR(Roll, Pitch, Yaw);

    //IMU 估算航向角

    //IMU 车子的加速度换算成为世界坐标系
    GetAccelerateXYZ(accelerate_x, accelerate_y, accelerate_z);

    //accelerate_z = imu.GetAccelerateZ();
    float accelerate_xInXY = accelerate_x * cos(Yaw) - accelerate_y * sin(Yaw);
    float accelerate_yInXY = accelerate_x * sin(Yaw) - accelerate_y * cos(Yaw);

    //上一次世界坐标系上的加速度值
    GetLastAccelerometer(lastAccelerate_xInXY, lastAccelerate_yInXY,lastAccelerate_zInXY);
    //IMU 加速度平均值
    float averageAccelerate_xInXY = (lastAccelerate_xInXY + accelerate_xInXY) / 2.0;
    float averageAccelerate_yInXY = (lastAccelerate_yInXY + accelerate_yInXY) / 2.0;
    float averageAccelerate_zInXY = lastAccelerate_zInXY;
    SetLastAccelerometer(averageAccelerate_xInXY, averageAccelerate_yInXY, averageAccelerate_zInXY);

    //IMU 估算速度
    GetSpeed(last_vx, last_vy, last_w);
    float vx = averageAccelerate_xInXY * ImuDT + last_vx;
    float vy = averageAccelerate_yInXY * ImuDT + last_vy;
    float w = last_w;
    SetSpeed(vx, vy, w);

    //IMU 估算距离
    GetPosition(last_x,last_y,last_theta);
    float x = 1/2 * averageAccelerate_xInXY * pow(ImuDT, 2) + vx * ImuDT + last_x; 
    float y = 1/2 * averageAccelerate_yInXY * pow(ImuDT, 2) + vy * ImuDT + last_y; 
    float z = last_z;
    float theta = this->PostureYPR.Yaw;
    SetPosition(x,y, theta);
}

void ImuData::ImuUpdateYaw(float yaw)
{
	float _Yaw;
	GetPostureYPR(UpDateBenchmarkYPR.Roll, UpDateBenchmarkYPR.Pitch, _Yaw);
	UpDateBenchmarkYPR.Yaw = yaw;
}

void ImuData::ClearQuaternion(void)
{
	q0 = q1 = q2 = q3 = 0;
}


