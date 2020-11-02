#include <sys/socket.h>
#include <arpa/inet.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>	//显示点云类头文件
#include <unistd.h>
#include <time.h>
#include <iostream>

#include "../include/Lidar.h"

LidarData lidar;
LidarData*  plidar = new LidarData();
LidarSendCloudStruct LidarSendCloudData;
pthread_mutex_t LidarMutex;

pcl::PointCloud<pcl::PointXYZ>::Ptr safecloud;

int m_sock;										    //网口
std::vector< std::vector<float> > lidarAngle;	    //角度
std::vector< std::vector<int> > lidarDist;		    //距离
std::vector< std::vector<int> > lidarInstensity;	//强度

float theta[16] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };
double PI = 3.1415926;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
std::vector<std::vector<double>> data;
//struct timeval tv;
//struct timezone tz;   
//struct tm *t;

LidarData::LidarData()
{

}

LidarData::~LidarData()
{

}

int LidarData::InitSocket()
{
    //******************UDP通信初始化**************************//
	int retVal;

	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(2368);
	sockAddr.sin_addr.s_addr = inet_addr("192.168.1.102");

	retVal = bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));   

	return retVal;
}


int LidarData::InitLidar()
{
	int retVal;
	
	retVal = lidar.InitSocket(); //初始化

	if(0 == retVal)
	{
		pthread_t readLidarThread;
    	pthread_create(&readLidarThread, NULL, ReceiveLidarData, (void*)this);
		pthread_t parsingLidarThread;
		pthread_create(&parsingLidarThread, NULL, ParsingLidarData, (void*)this);
	}
	return retVal;
}

void* LidarData::ReceiveLidarData(void *arg)
{
	struct sockaddr_in addrFrom;
	unsigned int len = sizeof(sockaddr_in);
	
	//接收数据
	char recvBuf[1206] = {0};
	int recvLen;
	std::vector<std::vector<float> >angle;
	std::vector<std::vector<int> >distance;
	std::vector<std::vector<int> >intensity;
	angle.resize(16);
	distance.resize(16);
	intensity.resize(16);
	while (true)
	{
		//获取套接字接收内容
		recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (struct sockaddr *)&addrFrom, &len);
		if (recvLen > 0 && recvBuf[0] == 255 && recvBuf[1] == 238 /*&& recvBuf[1204] == 57 && recvBuf[1205] == 32*/) //接收到数据
		{
			//处理接收的数据
			std::vector<unsigned char> data;
			for (int i = 0; i < 1200; i++)
			{
				data.push_back(recvBuf[i]);	//只保留每包数据的前1200个字节，去掉尾部4个字节时间戳，2个字节参数
			}
			for (int i = 0; i < 12; i++)	//LS16线协议和Velodyne有区别
			{
				float tempAngle;
				if ((data[3 + 100 * i] * 256 + data[2 + 100 * i]) / 100.f >= 360.0)
				{
					tempAngle = 0;
				}
				else
				{
					tempAngle = (data[3 + 100 * i] * 256 + data[2 + 100 * i]) / 100.f;		//提取出一圈中的方位角度值
				}

				angle[0].push_back(tempAngle);
				if (angle[0].size() >= 2)
				{
					float temp = (tempAngle - angle[0][angle[0].size() - 2]) / 32.f;
					if(temp < 0)
					{
						temp = (tempAngle + 360 - angle[0][angle[0].size() - 2]) / 32.f;	
					}
					angle[0].insert(angle[0].end() - 1,  angle[0][angle[0].size() - 2] + temp * 16);
					for (int j = 1; j < 16; j++)
					{
						angle[j].push_back(angle[0][angle[0].size() - 3] + temp * j);
						angle[j].push_back(angle[0][angle[0].size() - 3] + temp * (j + 16));
					}
				}

				for (int j = 0; j < 16; j++)
				{
					distance[j].push_back((data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]));
					distance[j].push_back((data[53 + 3 * j + 100 * i] * 256 + data[52 + 3 * j + 100 * i]));
					intensity[j].push_back(data[6 + 3 * j + 100 * i]);
					intensity[j].push_back(data[54 + 3 * j + 100 * i]);
				}

				if (abs(angle[0].back() - angle[0].front()) < 0.5 && angle[0].size() > 500)
				{
					float temp = (tempAngle - angle[0][angle[0].size() - 3]) / 32.f;
					if(temp < 0)
					{
						temp = (tempAngle + 360 - angle[0][angle[0].size() - 2]) / 32.f;	
					}
					angle[0].push_back(angle[0][angle[0].size() - 2] + temp * 16);
					for (int j = 1; j < 16; j++)
					{
						angle[j].push_back(angle[0].back() + temp * j);
						angle[j].push_back(angle[0].back() + temp * (j + 16));
					}

					if (lidarAngle.empty())
					{
						lidarDist = distance;
						lidarInstensity = intensity;
						lidarAngle = angle;
					}

					angle.clear();
					angle.resize(16);
					distance.clear();
					distance.resize(16);
					intensity.clear();
					intensity.resize(16);
				}
			}
		}
	}
}

void LidarData::LidarParsing()
{
	double sinTheta[16] = { 0 };
	double cosTheta[16] = { 0 };
	for (int i = 0; i < 16; i++)
	{
		cosTheta[i] = cos(theta[i] * PI / 180.f);
		sinTheta[i] = sin(theta[i] * PI / 180.f);
	}

	std::vector<std::vector<double> > sinAngle;
	std::vector<std::vector<double> > cosAngle;
	sinAngle.resize(16);
	cosAngle.resize(16);

	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < (int)lidarAngle[i].size(); j++)
		{
			sinAngle[i].push_back(sin(lidarAngle[i][j]  * PI / 180.f));
			cosAngle[i].push_back(cos(lidarAngle[i][j]  * PI / 180.f));
		}
	}

	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < (int)lidarAngle[i].size(); j++)
		{
			if (lidarDist[i][j] < 0.2)
			{
				continue;
			}
			//pcl::PointXYZRGBA PointTemp1;
			std::vector<double> PointTemp1;
			PointTemp1.push_back ((lidarDist[i][j] * cosTheta[i] * sinAngle[i][j]) / 100.f);
			PointTemp1.push_back ((lidarDist[i][j] * cosTheta[i] * cosAngle[i][j]) / 100.f);
			PointTemp1.push_back ((lidarDist[i][j] * sinTheta[i]) / 100.f);
			data.push_back(PointTemp1);
		}
	}
}

 void* LidarData::ParsingLidarData(void *arg)
{
	int j = 0;

	while (true)
	{
		if (lidarAngle.size() > 0)
		{
			lidar.LidarParsing();

			
			
			//std::cout<<"received data"<<std::endl;
			j++;
			
			pcl::PointCloud<pcl::PointXYZ> cloud;
			std::vector<std::vector<double>> clouddata;
			for(int i=0; i<data.size(); i++)
			{
				
				if(data[i][2]>=-1.0&&data[i][2]<=2&&((0.8<=data[i][0]&&data[i][0]<=20)||(data[i][0]>=-20&&data[i][0]<=-0.4))&&data[i][1]<=20&&data[i][1]>=-20)
				{
					std::vector<double> dd;
					dd.push_back(data[i][0]);
					dd.push_back(data[i][1]);
					dd.push_back(data[i][2]);
					clouddata.push_back(dd);
				}
			}
			cloud.width = clouddata.size();
			cloud.height = 1;
			cloud.is_dense = false;
			cloud.points.resize(cloud.width * cloud.height);
			for(int i = 0; i<clouddata.size();i++)
			{
				cloud.points[i].x=clouddata[i][0];
				cloud.points[i].y=clouddata[i][1];
				cloud.points[i].z=clouddata[i][2];
			}
			
                        pthread_mutex_lock(&LidarMutex);

			safecloud = cloud.makeShared();
			std::cout<<safecloud->size()<<std::endl;
                        //memcpy(&LidarSendCloudData.cloud,&cloud,sizeof(LidarSendCloudData.cloud));
                        //LidarSendCloudData.CloudReadSucceesFlag = 1;
                        pthread_mutex_unlock(&LidarMutex);


			data.clear();
			lidarDist.clear();
			lidarInstensity.clear();
			lidarAngle.clear();
		}
		else
			usleep(1000);
	}
	return 0;
}


