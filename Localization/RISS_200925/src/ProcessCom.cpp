#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <dbus/dbus.h>
#include <stdbool.h>

#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/types.h> 

#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <errno.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/ProcessCom.h"

/*************** IPC信号量 ***********************/
int PosSemKey = 2463;
int PosSemId; //信号量 
/*************** IPC信号量 ***********************/

/*************** 消息队列通信 ***********************/
int PosMsgId;         //定义位置消息Id, 进程间消息队列共享
int PosMsgKey = 2464;  //消息键
/*************** 消息队列通信 ***********************/

/*************** 共享内存通信 ***********************/
int PosShmKey = 2465;
int PosShmId; //获取共享内存(Odom Imu Gps + Pos)句柄  
/*************** 共享内存通信 ***********************/

/************************** Dbus通信  *********************************/
DBusConnection *RobotPosInfoConnection; // 定位进程将机器人位置传给导航进程
/************************** Dbus通信  *********************************/

/***************************************************************  进程间的信号量通信 ***************************************************/
//(1)第一个参数key是长整型（唯一非零），系统建立IPC通讯 （ 消息队列、 信号量和 共享内存） 时必须指定一个ID值。
//通常情况下，该id值通过ftok函数得到，由内核变成标识符，要想让两个进程看到同一个信号集，只需设置key值不变就可以。
//(2)第二个参数nsem指定信号量集中需要的信号量数目
//(3)第三个参数flag是一组标志，当想要当信号量不存在时创建一个新的信号量，可以将flag设置为IPC_CREAT与文件权限做按位或操作。 
//设置了IPC_CREAT标志后，即使给出的key是一个已有信号量的key，也不会产生错误。而IPC_CREAT | IPC_EXCL则可以创建一个新的，唯一的信号量，如果信号量已存在，返回一个错误。一般我们会还或上一个文件权限

int InitSem(int &semId, int key, int num, int cmd)
{
    semun sem_union;
    sem_union.val = 0;

    semId = semget(key_t(key), num, cmd);//创建并打开 或者创建 打开拆开 用时打开

    if(semId < 0)
    {
        perror("InitSem failed");
        return -1;

    }
    //信号量存在则获取
    else
    {
        std::cout << "InitSem Success" << std::endl;
        return 0;
    }
    
    if (semctl(semId, 0, SETVAL, sem_union) == -1) //设置信号量的计数值为0
    {
        printf("error to open semaphore!\n");
        return -1;
    }

    //设置信号量集合中的信号量（元素）的计数值
    SetSem(semId,1);
    
    return 0; 
}

int CreatSem(int &semId, int key, int num, int cmd)
{
    semId = semget(key_t(key), num, cmd);
    if(semId < 0)
    {
        std::cout << "Creat Sem failed" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Creat Sem failed" << std::endl;
        return 0;
    }
    
}


//设置信号量集合里面的信号量的计数值val
int SetSem(int semId,int val)
{
  int ret = 0;

  if(semId < 0)
  {
    return -1;
  }
  semun su;
  su.val = val;
  ret = semctl(semId,0,SETVAL,su);
  return ret;
}

//获取信号量集合里面的信号量的计数值
int GetSem(int semId, int& val)
{
    int ret = 0;
    semun sem_union;
    sem_union.val = 0;
    
    if(semId < 0)
    {
        return -1;
    }

    ret = semctl(semId,0,GETVAL,sem_union);
    val = sem_union.val;
    return ret;
}


//struct sembuf{
//    short sem_num; // 除非使用一组信号量，否则它为0
//    short sem_op;  // 信号量在一次操作中需要改变的数据，通常是两个数，
                     // 一个是-1，即P（等待）操作，
                     // 一个是+1，即V（发送信号）操作。
//   short sem_flg;  // 通常为SEM_UNDO,使操作系统跟踪信号，
//                   // 并在进程没有释放该信号量而终止时，操作系统释放信号量
//};
int P_sem(int semId, int semIndex)
{
    struct sembuf s;
    s.sem_num = semIndex;
    s.sem_op = -1;
    s.sem_flg = SEM_UNDO;

    if(semop(semId,&s,1) < 0)
    {
        printf("op errno,%d: %s\n", errno, strerror(errno));
        return -1; 
    } 
}

int  V_sem(int semId, int semIndex)
{
    struct sembuf s;
    s.sem_num = semIndex;
    s.sem_op = 1;
    s.sem_flg = SEM_UNDO;
    if(semop(semId,&s,1) < 0)
    {
         printf("ov error,%d:%s\n",errno,strerror(errno)); 
         return -1;   
    }
}

// union semun {
// int val;
// struct semid_ds *buf;
// unsigned short *arry;
// };
// SETVAL：用来把信号量初始化为一个已知的值。
// p 这个值通过union semun中的val成员设置，其作用是在信号量第一次使用前对它进行设置。
// IPC_RMID：用于删除一个已经无需继续使用的信号量标识符。

int DestorySem(int semId)
{
    semun sem_union;
     
    if (semctl(semId, 0, IPC_RMID, sem_union) == -1) 
    {
        printf("err to delete semaphore!\n");
         
        return -1;
    }
}
/***************************************************************  进程间的信号量通信 ***************************************************/

/***************************************************************  进程间的消息队列通信 ***************************************************/
//msgflag：有两个选项IPC_CREAT和IPC_EXCL，
//单独使用IPC_CREAT，如果消息队列不存在则创建之，
//如果存在则打开返回；单独使用IPC_EXCL是没有意义的；
//两个同时使用，如果消息队列不存在则创建之，如果存在则出错返回。 
int MessageQueueInit(int &msgId, int msgKey, int cmd)
{
    //先检查消息队列是否存在
    msgId = msgget(msgKey, cmd);

    //不存在则创建
    if(msgId < 0)
    {
        std::cout << "InitMsg failed" << std::endl;
        return -1; 
    } 
    else
    {
        std::cout << "InitMsg Success" << std::endl;
        return 0;

    }
    
}

int CreatMessageQueue(int &msgId, int msgKey, int cmd)
{
    msgId = msgget(msgKey, cmd);

    if(msgId < 0)
    {
        fprintf(stderr, "CreatMsg failed with error:%d\n", errno);
        return -1; 
    }
    else
    {
        std::cout << "CreatMsg successful" << std::endl;
        return 0;
    }
}

//msgid：由msgget函数返回的消息队列标识码
//msgp：指针指向准备发送的消息
//msgze：msgp指向的消息的长度（不包括消息类型的long int长整型）
//msgflg: 0：当消息队列满时，msgsnd将会阻塞，直到消息能写进消息队列
//        IPC_NOWAIT：当消息队列已满的时候，msgsnd函数不等待立即返回
//        IPC_NOERROR：若发送的消息大于size字节，则把该消息截断，截断部分将被丢弃，且不通知发送进程。
int SendMessageQueue(int msgId, void* dataAdress, int msgSize, int cmd)
{
    if(-1 == msgsnd(msgId, dataAdress, msgSize, cmd))  //理解： 将数据地址(dataAdress)大小(msgSize)发送到消息队列
    {
        fprintf(stderr, "msgsnd failed: %d\n", errno);
        return -1;
    }
    else
    {
        std::cout << "msgsnd successful" << std::endl;
        return 0;
    }    
}

//int msgrcv(int msqid, void  *ptr, size_t  length, long  type, int  flag);
//type: 决定从队列中返回哪条消息：
//      =0 返回消息队列中第一条消息
//      >0 返回消息队列中等于mtype 类型的第一条消息。
//      <0 返回mtype<=type 绝对值最小值的第一条消息。
//msgflg为０表示阻塞方式，设置 IPC_NOWAIT 表示非阻塞方式

int RcvMessageQueue(int msgId, void* dataAdress, int msgSize, int cmd)
{
    if(-1 == msgrcv(msgId, dataAdress, msgSize, 0, cmd))    
    {
        fprintf(stderr, "msgrcv failed with errno: %d", errno);
        return -1;   
    }
    else
    {
        std::cout << "msgrcv successful" << std::endl;
        return 0;
    }
    
}

//msqid：由msgget函数返回的消息队列标识码
//cmd：有三个可选的值，在此我们使用IPC_RMID
//    IPC_STAT 把msqid_ds结构中的数据设置为消息队列的当前关联值
//    IPC_SET 在进程有足够权限的前提下，把消息队列的当前关联值设置为msqid_ds数据结构中给出的值
//    IPC_RMID 删除消息队列

int DestoryMessageQueue(int msgId)
{
    if(msgctl(msgId, IPC_RMID, NULL) < 0)
    {
        perror("msgctl");
        return -1;
    }
    return 0;
}
/***************************************************************  进程间的消息队列通信 ***************************************************/


/***************************************************************  进程间的内存共享通信 ***************************************************/
//key：共享内存名字，确保不同进程看到同一份IPC资源；
//其中key值的又由ftok函数创建。
//size：共享内存的大小，共享内存的创建是以页为单位的;
//页的大小是4096k（4kb）
//共享内存分别存放Odom Imu Gps Robot数据  请按照此顺序进行读取以及写入
int InitShareMemory(int &shmid, int key, int size, int cmd)
{  
    int pageSize = getpagesize();//获取系统页面的大小

    if(size <= 0)
    {
        std::cout << "共享内存创建非法，请检查！" << std::endl;
        return -1;
    }
    else if(size > pageSize)
    {
        std::cout << "创建共享内存段的大小超过页面的大小！" << std::endl;
        return -1;
    }
    else 
    {
        shmid = shmget(key, size, cmd);

        if(shmid < 0)
        {
            std::cout << "InitShm failed" << std::endl;
            return -1; 
        }
        else
        {
            std::cout << "InitShm success" << std::endl;
            return 0;
        }

        return 0;
    }

}

int CreatShareMemory(int &shmid, key_t key, int size, int cmd)
{
    shmid = shmget(key, size, IPC_CREAT | 0666);

    if( shmid < 0) 
    {
        perror("CreatShm failed");
        return -1;
    }
    else
    {
        std::cout << "CreatShm successful" << std::endl;
        return 0;
    }  
}

void* AttachShareMemory(int shmid)
{
    void* shmaddr;
    return shmaddr = (void *) shmat(shmid, NULL, 0);
}

void* WriteShareMemory(void* shmaddr, void* source, int size)
{
    return memcpy(shmaddr, source, size);
}

void* ReadShareMemory(void* destion, void* shmaddr, int size)
{
    return memcpy(destion, shmaddr, size);
}

int DisattachShareMemory(void* shmaddr)
{
    return shmdt(shmaddr);
}

int DestoryShareMemory(int shmid)
{
    return shmctl(shmid, IPC_RMID, NULL);
}
/***************************************************************  进程间的内存共享通信 ***************************************************/




/***************************************************************  进程间的Dbus通信 ***************************************************/
bool RegisterRobotPosInfo()
{
    DBusError err;
	DBusMessage * msg;
	int ret;

    dbus_error_init(&err); //错误初始化

    RobotPosInfoConnection = dbus_bus_get(DBUS_BUS_SESSION ,&err);// 连接到总线
    if(dbus_error_is_set(&err))
	{
        fprintf(stderr,"ConnectionErr : %s\n",err.message);
        dbus_error_free(&err); // 清除错误消息
    }
    if(RobotPosInfoConnection == NULL)
	{
		// TODO 链接失败 本地保存错误信息
		return false;
	}

    ret = dbus_bus_request_name(RobotPosInfoConnection, "RobotPosInfo.singal.source",DBUS_NAME_FLAG_REPLACE_EXISTING,&err);
    if(dbus_error_is_set(&err))
	{
        fprintf(stderr,"Name Err :%s\n",err.message);
        dbus_error_free(&err);
    }
    if(ret != DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER)
	{
		std::cout << "this name have be occupy!" << std::endl;
	}
	return true;
}

bool SendRobotPosInfo()
{
	DBusMessage * msg;

	DBusMessageIter datatoSendIter; // DBus数据迭代器
	dbus_uint32_t  serial =0;

	if((msg = dbus_message_new_signal("/RobotPosInfo/signal/Object","RobotPosInfo.signal.Type","RobotPosInfo"))== NULL)
	{
        fprintf(stderr,"MessageNULL\n");
        std::cout << "new signal error" << std::endl;
        return false;
	}

    float x , y, z,yaw,pitch,roll;
	robot.GetRobotPosition(x , y, z);
    robot.GetRobotRotation(pitch, roll, yaw);

    /************** 测试代码 ********/
    x = rand() / 100.0;
    y = rand() / 100.0;
    z = rand() / 100.0;
    yaw = rand() / 100.0;
    pitch = rand() / 100.0;
    roll = rand() / 100.0;
    /************** 测试代码 ********/
    
	dbus_message_iter_init_append(msg,&datatoSendIter);  
    dbus_message_iter_append_basic(&datatoSendIter, DBUS_TYPE_DOUBLE, &x);
    dbus_message_iter_append_basic(&datatoSendIter, DBUS_TYPE_DOUBLE, &y); 
    dbus_message_iter_append_basic(&datatoSendIter, DBUS_TYPE_DOUBLE, &z);
    dbus_message_iter_append_basic(&datatoSendIter, DBUS_TYPE_DOUBLE, &yaw);
    dbus_message_iter_append_basic(&datatoSendIter, DBUS_TYPE_DOUBLE, &pitch); 
    dbus_message_iter_append_basic(&datatoSendIter, DBUS_TYPE_DOUBLE, &roll);

	//步骤4: 将信号从连接中发送
	if(!dbus_connection_send(RobotPosInfoConnection,msg,&serial))
	{
		fprintf(stderr,"Out of Memory!\n");
		return -1;
	}
	dbus_connection_flush(RobotPosInfoConnection);
	//printf("Signal Send\n");

	//步骤5: 释放相关的分配的内存。
	dbus_message_unref(msg);
}
/***************************************************************  进程间的Dbus通信 ***************************************************/
void six2Trans(float yaw, float roll, float pitch, float x, float y, float z, Eigen::Matrix4f& transform)
{
    Eigen::Vector3f eulerAngle(yaw, pitch, roll);
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitZ()));

    Eigen::Matrix3f rotation;
    rotation = yawAngle * pitchAngle * rollAngle;

    Eigen::Translation3f translation(x, y, z);
    transform = (translation * rotation).matrix();
}



