#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h> 
using namespace std;

struct pose
{
    int x;
    int y;
};


int main(int argc,char **argv)
{
    struct sembuf arg;

    key_t key = ftok("../nt.txt", 0x33);
    key_t semkey = ftok("../nt.txt",0x22);

    int shmid = shmget(key, 600000, IPC_CREAT);
    int semid = semget(semkey, 1, IPC_CREAT);

    float *p = (float *)shmat(shmid,0,600000);


    arg.sem_num=0;
    arg.sem_op=1;
    semop(semid,&arg,1);

    int size = 10000;
    int j = 0;
    for(size_t i = 0 ; i < size; i++)
    {    
        arg.sem_num=0;
        arg.sem_op=-1;
        semop(semid,&arg,1);
        p[2*i] = j;
        p[2*i+1] = j+4;
        j++;
        printf("x=%f\n",p[2*i]);
        printf("y=%f\n",p[2*i+1]);

        arg.sem_num=0;
        arg.sem_op=1;
        semop(semid,&arg,1);
        sleep(1);
    }




    return 0;
}
