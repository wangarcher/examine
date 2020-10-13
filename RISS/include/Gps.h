#pragma once
#ifndef _GPS_H
#define _GPS_H

typedef struct 
{
    /* data */
    float x;
    float y;
    float theta;
}GpsPosture;


struct gpgga
{
    double Ttc;  //Utc 时间(hhmmss.ss)
    double Lat;  //纬度
    char LatDir; //纬度方向
    double Lon;  //经度
    char LonDir; //经度方向
    int  Qual;   //GPS质量指示  0 = 定位不可用或无效 1 = 单点定位 2 = 伪距差分 
                 //4 = RTK 固定解 5 =RTK 浮点解 6 = 惯导定位 7 = 用户固定位置
};

struct gpzda
{
    double Utc;   //时间
    int    Day;   
    int    Month; 
    int    Year;  
};

struct gpvtg
{
    double TrackTrue; //以真北为参考基准的航向
    double SpeedOverGround; //地面速度,单位 km/hr
    char   ModeInd; //模式指示 A=自主定位 D=差分 E=估算 N=数据无效
};

struct gphdt
{
    double Utc;
    double Heading;  //相当于绕Z轴旋

};

class GPSdata
{
private:
   
    struct gpgga GPGGA;
    struct gpzda GPZDA;
    struct gpvtg GPVTG;
    struct gphdt GPHDT;
	
    GpsPosture Posture;
	
	double FirstLat_;
	double FirstLon_;
	
	
private:
    static void* ReadRtkData(void *arg);
    static void* SendSationData(void *arg);

public:
    
    GPSdata(){ }
    ~GPSdata(){ }

	int getNowGpsQual();
	
    int  InitGPS();
    void ReadGPSData();
	
	void setFirstLatLon();
	void getFirstLatLon(double &FirstLat,double &FirstLon);
	
    void AnalysisGpsposture();
    void getGpsPosture(float &x,float &y,float &theta);
	
    void setGPGGA(char *gpsdata);
    void getGPGGA(struct gpgga &GPGGA);

    void setGPZDA(char *gpsdata);
    void getGPZDA(struct gpzda &GPZDA);
    
    void setGPVTG(char *gpsdata);
    void getGPVTG(struct gpvtg &GPVTG);

    void setGPHDT(char *gpsdata);
    void getGPHDT(struct gphdt &GPHDT);

    int StartReadRtkDataThread();
    int StartSendStationDataThread();

};

extern GPSdata gps;
extern GPSdata* pgps;

#endif