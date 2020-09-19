#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <sys/sem.h>
#include <iostream>
#include <unistd.h> 
using namespace std;

int main(int argc, char* argv[]){
    struct sembuf arg;

    key_t key = ftok("/home/archer/nt.txt", 0x55);
    key_t semkey = ftok("/home/archer/nt.txt",0x44);


    int shmid = shmget(key, 0, IPC_CREAT);
    int semid = semget(semkey, 1, IPC_CREAT);

    float *p = (float *)shmat(shmid,0,5);

    int i = 0;
    
    int size = 10000;
    int j = 0;
    int limit = (int) p[0];
    for(size_t i = 1 ; i < limit; i++)
    {    
        arg.sem_num=0;
        arg.sem_op=-1;
        semop(semid,&arg,1);

        printf("x=%f\n",p[2*i-1]);
        printf("y=%f\n",p[2*i]);


        arg.sem_num=0;
        arg.sem_op=1;
        semop(semid,&arg,1);
        //sleep(1);
    }

    shmdt(p);

    return 0;
}

