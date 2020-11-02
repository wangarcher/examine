#include <iostream>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>
#include <cmath>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>


#include "../include/Gps.h"
#include "../include/UartCom.h"
#include "../include/qxwz_sdk.h"

#define   QianXunKey      "A48g1d6hinub"
#define   QianXunSecret   "195863f61d3d6c0b"
#define   QianXunDevId    "D48g1d6hinvm" 
#define   QianXunDevType  "normal"


pthread_mutex_t mytex;
char serial_gpgga[128];
char serial_gpzda[128];
char serial_gpvtg[128];
char serial_gpshr[128];


GPSdata gps;
GPSdata* pgps = new GPSdata();

static qxwz_uint32_t glob_flag = 0;
static qxwz_uint64_t glob_ts = 0;
static qxwz_uint32_t sdk_auth_flag = 0;
static qxwz_uint32_t sdk_start_flag = 0;
qxwz_sdk_cap_info_t sdk_cap_info = {0};

int fd = 0;

static qxwz_void_t demo_show_caps(qxwz_sdk_cap_info_t *cap_info)
{
    char buf[100];
    qxwz_int32_t loop = 0;

    for (loop = 0; loop < cap_info->caps_num; ++loop) {
        sprintf(buf,"idx: %d, cap_id: %u, state: %d, act_method: %d, expire_time: %llu\n",
            loop + 1,
            cap_info->caps[loop].cap_id,
            cap_info->caps[loop].state,
            cap_info->caps[loop].act_method,
            cap_info->caps[loop].expire_time
                );
    }
}

static qxwz_void_t demo_on_auth(qxwz_int32_t status_code, qxwz_sdk_cap_info_t *cap_info) {
    char buf[100];
    if (status_code == QXWZ_SDK_STAT_AUTH_SUCC) {
        sdk_auth_flag = 1;
        sdk_cap_info = *cap_info;
        demo_show_caps(cap_info);
    } else {
        printf("auth failed, code=%d\n", status_code);
    }
}

static qxwz_void_t demo_on_start(qxwz_int32_t status_code, qxwz_uint32_t cap_id) {
    char buf[100];
    sprintf(buf,"on start cap:status_code=%d, cap_id=%d\n", status_code, cap_id);
    sdk_start_flag = 1;
}


static qxwz_void_t demo_on_status(int code)
{
}

static void demo_on_data(qxwz_sdk_data_type_e type, const qxwz_void_t *data, qxwz_uint32_t len)
{
    char buf[100];

    switch (type) {
        case QXWZ_SDK_DATA_TYPE_RAW_NOSR:
            //strcpy(DEMO_GGA_STR,(char *)data);
            write(fd,data,len);
            break;
        default:
            printf("unknown type: %d\n", type);
    }
}



int GPSdata::InitGPS()
{
    
    pthread_mutex_init(&mytex,NULL); //互斥锁初始化

    fd = open("/dev/ttyUSB2",O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd <= 0)
    {
        std::cout << "Fail to open /dev/ttyUSB2!" << std::endl;
        return 1;
    }

    UART_Init_Baud(fd, 115200);

    gps.StartReadRtkDataThread();
    gps.StartSendStationDataThread();

    return 0;
}


void* GPSdata::ReadRtkData(void *arg)
{
    int ret;
    char buf[100];
	gps.setFirstLatLon();
    while(1)
    {
        usleep(10000);
        memset(buf, 0, 100);
		
        ret = read(fd,buf,100);

        if(ret > 0)
        {
            if(strstr(buf,"$GPGGA"))
            {
                pthread_mutex_lock(&mytex);
                strcpy(serial_gpgga,buf);
                pthread_mutex_unlock(&mytex);
				gps.setGPGGA(buf);
				std::cout<<"serial_gpgga:"<<serial_gpgga<<std::endl;
            }
            else if(strstr(buf,"$GPZDA"))
            {
                pthread_mutex_lock(&mytex);
                strcpy(serial_gpzda,buf);
                pthread_mutex_unlock(&mytex);
				gps.setGPZDA(buf);
				std::cout<<"serial_gpzda:"<<serial_gpzda<<std::endl;
            }
            else if(strstr(buf,"$GPVTG"))
            {
                pthread_mutex_lock(&mytex);
                strcpy(serial_gpvtg,buf);
                pthread_mutex_unlock(&mytex);
				gps.setGPVTG(buf);
				std::cout<<"serial_gpvtg:"<<serial_gpvtg<<std::endl;
            }
            else if(strstr(buf,"$GPHDT"))
            {
                pthread_mutex_lock(&mytex);
                strcpy(serial_gpshr,buf);
                pthread_mutex_unlock(&mytex);
				gps.setGPHDT(buf);
				std::cout<<"serial_gphdt:"<<serial_gpshr<<std::endl;
            }
			gps.AnalysisGpsposture();
        }
    }
}

qxwz_sdk_config_t sdk_config;

void* GPSdata::SendSationData(void *arg)
{  
    int ret = 0;   
    char serial_data[1024];       
	
	/**************************START Qianxun Init***************************/
	
    sdk_config.key_type = QXWZ_SDK_KEY_TYPE_AK,  
    strcpy(sdk_config.key,QianXunKey); 
    strcpy(sdk_config.secret, QianXunSecret); 
	
    strcpy(sdk_config.dev_id, QianXunDevId);   
    strcpy(sdk_config.dev_type, QianXunDevType);   
	
    sdk_config.status_cb    = demo_on_status;  
    sdk_config.data_cb      = demo_on_data;  
    sdk_config.auth_cb      = demo_on_auth;  
    sdk_config.start_cb     = demo_on_start;      
	
    ret = qxwz_sdk_init(&sdk_config);  
	
	/**************************END Qianxun Init***************************/ 
    if (ret < 0) 
    { 
        printf("sdk init failed\n");       
        goto END;  
    }   
    /*     * do authentication     */  
    ret = qxwz_sdk_auth();  
    if (ret < 0)
    {     
        printf("call sdk auth failed\n");    
        goto END;   
    }	  
    while (1) 
    {	
        pthread_mutex_lock(&mytex);  
        strcpy(serial_data,serial_gpgga);   
        pthread_mutex_unlock(&mytex);
        if(!strstr(serial_data,"$GPGGA"))
        {
            continue;
        }
		qxwz_sdk_upload_gga(serial_data, strlen(serial_data));	
		qxwz_sdk_start(QXWZ_SDK_CAP_ID_NOSR);     

	    
        usleep(1000 * 1000); /* 1 s */  
		
    }
    qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR); 
    /* stop NOSR capability */
END:   
    while (qxwz_sdk_cleanup() != 0) 
    {    
        usleep(100 * 1000);   
    }
}
	
int GPSdata::StartReadRtkDataThread()
{
    pthread_t readRtkThread;
    pthread_create(&readRtkThread, NULL, ReadRtkData, (void*)this);  
}

int GPSdata::StartSendStationDataThread()
{
    pthread_t sendStationThread;
    pthread_create(&sendStationThread, NULL, SendSationData, (void*)this);

}

char *ComeNextNumSemicolon(char *str,int num)
{
	while(*str)
	{
		if( ',' == *str)
		{
			num--;
			if( 0 == num )break;
		}
		str++;
	}
	if(NULL == str)
	{
		return NULL;
	}
	return str+1;
}


void GPSdata::setFirstLatLon()
{
	FILE *ReadFristLatLon = fopen("../FristLatLon.txt","r+");
	fscanf(ReadFristLatLon,"%lf",&this->FirstLat_);
	fscanf(ReadFristLatLon,"%lf",&this->FirstLon_);
	fclose(ReadFristLatLon);
}
void GPSdata::getFirstLatLon(double &FirstLat,double &FirstLon)
{
	FirstLat = this->FirstLat_;
	FirstLon = this->FirstLon_;
}

void GPSdata::AnalysisGpsposture()
{
	double difflat = this->GPGGA.Lat - this->FirstLat_;
	
	double difflon = this->GPGGA.Lon - this->FirstLon_;
	
	this->Posture.x = difflat / 100 * 111194.926644558737/0.6;
	this->Posture.y = difflon / 100 * 111194.926644 * cos(22.4048 * 3.1415926 / 180)/0.6;
	this->Posture.theta = this->GPHDT.Heading ;
}

/*
函数名称：getNowGpsQual()
返 回 值：return GPSQual  ( 1 - 5 )
*/
int GPSdata::getNowGpsQual()
{
	return this->GPGGA.Qual ;
}

void GPSdata::getGpsPosture(float &x,float &y,float &theta)
{
	x = this->Posture.x ;
	y = this->Posture.y ;
	theta = this->Posture.theta ;
}


void GPSdata::setGPHDT(char* gpsData)
{

	double gphdt;
	
	char *pdata = strstr(gpsData,"$GPHDT");
	if( NULL == pdata )
	{
		return ;
	}
	else 
	{
		sscanf(gpsData+7,"%lf",&gphdt);
		this->GPHDT.Heading = gphdt;
		
	}
}
void GPSdata::getGPHDT(struct gphdt &GPHDT)
{
	GPHDT.Heading = this->GPHDT.Heading;
}


void GPSdata::setGPZDA(char* gpsData)
{
    struct gpzda GPZDA;
	
	
	char *pdata=strstr(gpsData,"$GPZDA");
	
	
	if( NULL == pdata )
	{
		return ;
	}
	else 
	{
		pdata = ComeNextNumSemicolon(gpsData,1);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%lf",&GPZDA.Utc);
		if(NULL == pdata)
		{
			return ;
		}
		pdata = ComeNextNumSemicolon(gpsData,2);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%d",&GPZDA.Day);
		pdata = ComeNextNumSemicolon(gpsData,3);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%d",&GPZDA.Month);
		pdata = ComeNextNumSemicolon(gpsData,4);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%d",&GPZDA.Year);
		
		this->GPZDA.Utc = GPZDA.Utc;
		this->GPZDA.Day = GPZDA.Day;
		this->GPZDA.Month = GPZDA.Month;
		this->GPZDA.Year = GPZDA.Year;
	}
}
void GPSdata::getGPZDA(struct gpzda &GPZDA)
{
    
	GPZDA.Utc = this->GPZDA.Utc;
	GPZDA.Day = this->GPZDA.Day;
	GPZDA.Month = this->GPZDA.Month;
	GPZDA.Year = this->GPZDA.Year;
}

void GPSdata::setGPVTG(char* gpsData)
{
    struct gpvtg GPVTG;
	
	
	char *pdata = strstr(gpsData,"$GPVTG");
	
	if( NULL == pdata )
	{
		return ;
	}
	else 
	{
		pdata = ComeNextNumSemicolon(pdata,3);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%lf",&GPVTG.TrackTrue);
		pdata = ComeNextNumSemicolon(gpsData,7);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%lf",&GPVTG.SpeedOverGround);
		pdata = ComeNextNumSemicolon(gpsData,9);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%c",&GPVTG.ModeInd);
		
		this->GPVTG.TrackTrue = GPVTG.TrackTrue;
		this->GPVTG.SpeedOverGround = GPVTG.SpeedOverGround;
		this->GPVTG.ModeInd = GPVTG.ModeInd;
	}
}
void GPSdata::getGPVTG(struct gpvtg &GPVTG)
{
	GPVTG.TrackTrue = this->GPVTG.TrackTrue;
	GPVTG.SpeedOverGround = this->GPVTG.SpeedOverGround;
	GPVTG.ModeInd = this->GPVTG.ModeInd;
}


void GPSdata::setGPGGA(char* gpsData)
{
    struct gpgga GPGGA;
	
	char *pdata = strstr(gpsData,"$GPGGA");
	
	if( NULL == pdata )
	{
		return ;
	}
	else 
	{
		pdata = ComeNextNumSemicolon(gpsData,1);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%lf",&GPGGA.Ttc);
		pdata = ComeNextNumSemicolon(gpsData,2);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%lf",&GPGGA.Lat);
		pdata = ComeNextNumSemicolon(gpsData,3);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%c",&GPGGA.LatDir);
		pdata = ComeNextNumSemicolon(gpsData,4);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%lf",&GPGGA.Lon);
		pdata = ComeNextNumSemicolon(gpsData,5);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%c",&GPGGA.LonDir);
		pdata = ComeNextNumSemicolon(gpsData,6);
		if(NULL == pdata)
		{
			return ;
		}
		sscanf(pdata,"%d",&GPGGA.Qual);
		
		this->GPGGA.Ttc = GPGGA.Ttc;
		this->GPGGA.Lat = GPGGA.Lat;
		this->GPGGA.LatDir = GPGGA.LatDir;
		this->GPGGA.Lon = GPGGA.Lon;
		this->GPGGA.LonDir = GPGGA.LonDir;
		this->GPGGA.Qual = GPGGA.Qual;
	}
}


void GPSdata::getGPGGA(struct gpgga &GPGGA)
{
	GPGGA.Ttc = this->GPGGA.Ttc;
	GPGGA.Lat = this->GPGGA.Lat;
	GPGGA.LatDir = this->GPGGA.LatDir;
	GPGGA.Lon = this->GPGGA.Lon;
	GPGGA.LonDir = this->GPGGA.LonDir;
	GPGGA.Qual = this->GPGGA.Qual;
}

