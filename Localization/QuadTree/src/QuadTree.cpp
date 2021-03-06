/*

Project: the quad tree for searching closest point on a map
Date: 20/08/08
@Author: Wang. 
Detail: 1. [Done on 20/08/13] refinement needed, 
        so as to answer the demands of the upper level data structure
        2. [Done on 20/08/13] the dynamic array, 
        the method we using now could cause undesired troubles!!!!!
Scenario: localization and loop detection


*/
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <time.h>
#include <typeinfo>
#include <cmath>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include "QuadTree.h" //structure define and the other shits
#include <vector>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;


typedef struct _SendDataPack
{
    int index;
    float x;
    float y;
    float z; 
    float roll;
    float pitch;
    float yaw;
}SendDataPack;

typedef struct _RecvDataPack
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}RecvDataPack;


struct Posexyi
{
    float x;
    float y;
    int index;
};


//A func to insert the elements(frames) into the quadtree grid
void insertEle(struct QuadTreeNode *node, struct ElePoint ele) 
{   
    //if the current grid is a leaf node, we check some details 
    if (1 == node->is_leaf) 
    {
        //if the current grid has reach its capacity limitation
        if (node->ele_num + 1 > MAX_ELE_NUM) 
        {
            splitNode(node);
            insertEle(node, ele);
        } 
        //else we add this element to this grid
        else 
        {
            struct ElePoint *ele_ptr = (struct ElePoint *) malloc(sizeof(struct ElePoint));
            ele_ptr->x = ele.x;
            ele_ptr->y = ele.y;
            ele_ptr->index = ele.index;
            strcpy(ele_ptr->desc, ele.desc);
            node->ele_list[node->ele_num] = ele_ptr;
            node->ele_num++;
        }

        return;
    }
    //else we search the leaf node which is regarded as a grid(the elements container)!!
    float mid_vertical = (node->region.up + node->region.bottom) / 2;
    float mid_horizontal = (node->region.left + node->region.right) / 2;
    if (ele.y > mid_vertical) 
    {
        if (ele.x > mid_horizontal) 
        {
            insertEle(node->RU, ele);
        } 
        else 
        {
            insertEle(node->LU, ele);
        }
    } 
    else 
    {
        if (ele.x > mid_horizontal) 
        {
            insertEle(node->RB, ele);
        } 
        else 
        {
            insertEle(node->LB, ele);
        }
    }
}

//A func to split the node, when the grid reached its capacity limitation
void splitNode(struct QuadTreeNode *node) 
{
    float mid_vertical = (node->region.up + node->region.bottom) / 2;
    float mid_horizontal = (node->region.left + node->region.right) / 2;

    node->is_leaf = 0;//establish the link to the childnodes
    node->RU = createChildNode(node, mid_vertical, node->region.up, mid_horizontal, node->region.right);
    node->LU = createChildNode(node, mid_vertical, node->region.up, node->region.left, mid_horizontal);
    node->RB = createChildNode(node, node->region.bottom, mid_vertical, mid_horizontal, node->region.right);
    node->LB = createChildNode(node, node->region.bottom, mid_vertical, node->region.left, mid_horizontal);

    for (int i = 0; i < node->ele_num; i++) {
        insertEle(node, *node->ele_list[i]);
        free(node->ele_list[i]);
        node->ele_num--;
    }
}


//structure of create childnode
struct QuadTreeNode *createChildNode(struct QuadTreeNode *node, float bottom, float up, float left, float right) 
{
    int depth = node->depth + 1;
    struct QuadTreeNode *childNode = (struct QuadTreeNode *) malloc(sizeof(struct QuadTreeNode));
    struct Region *region = (struct Region *) malloc(sizeof(struct Region));
    initRegion(region, bottom, up, left, right);
    initNode(childNode, depth, *region);

    return childNode;
}

void deleteEle(struct QuadTreeNode *node, struct ElePoint ele)
{
//future work
}

void combineNode(struct QuadTreeNode *node) 
{
//future work
}

//A func to find the element near the current pose
//making attempts to find the closest grid and find the elements in that grid
void queryEle(struct QuadTreeNode node, struct ElePoint ele, float pose[][3], int count) 
{
    if (node.is_leaf == 1) 
    {
        for (int j = 0; j < node.ele_num; j++) 
        {
            pose[count][0] = node.ele_list[j]->x;
            pose[count][1] = node.ele_list[j]->y;
            pose[count][2] = node.ele_list[j]->index;
            count++;
            pose[MAX_ELE_NUM*9-1][0] = count;
        }
        return;
    }

    float mid_vertical = (node.region.up + node.region.bottom) / 2;
    float mid_horizontal = (node.region.left + node.region.right) / 2;
    if (ele.y > mid_vertical) 
    {
        if (ele.x > mid_horizontal) 
        {
            queryEle(*node.RU, ele, pose, count);
        } 
        else 
        {
            queryEle(*node.LU, ele, pose, count);
        }
    } 
    else 
    {
        if (ele.x > mid_horizontal) 
        {
            queryEle(*node.RB, ele, pose, count);
        } 
        else 
        {
            queryEle(*node.LB, ele, pose, count);
        }
    }
}

//void disturbTerm()

void initNode(struct QuadTreeNode *node, int depth, struct Region region) 
{
    node->depth = depth;
    node->is_leaf = 1;
    node->ele_num = 0;
    node->region = region;
}

void initRegion(struct Region *region, float bottom, float up, float left, float right) 
{
    region->bottom = bottom;
    region->up = up;
    region->left = left;
    region->right = right;
}

int main()
{   
    std::vector<Posexyi> Pose;

    //import the transformation mat of those keyframes
    //[Done] read pose info from the optimized_pose.txt
    ifstream fin("../../../pose.txt"); // TODO ???????
    int queue_i = 0;
    while(!fin.eof())
    {
        float queue_x, queue_y, queue_z, roll, pitch, yaw;
        Posexyi a;
        fin >> queue_i >> queue_x >> queue_y >> queue_z >> yaw >> pitch >> roll;
        a.x = queue_x;
        a.y = queue_y;
        a.index = queue_i;
        Pose.push_back(a);
        cout << "some index" << a.index << endl;
        if(!fin.good()) break;
    }


    int recvfd=open("../../../former_location", O_RDWR); //TODO
    int sendfd=open("../../../index", O_RDWR); //TODO

    if(recvfd<0)
    {
        std::cout<<"recv default"<<std::endl;
        exit(1);
    }
    if(sendfd<0)
    {
        std::cout<<"send default"<<std::endl;
        exit(1);
    }

    SendDataPack senddatapack;
    memset(&senddatapack,0,sizeof(senddatapack));
    int loop = 0;
while(1)
{
    loop++;
    cout << "the N0. " << loop << " loop. " << endl;
    RecvDataPack recvdatapack;
    memset(&recvdatapack,0,sizeof(recvdatapack));
    read(recvfd,&recvdatapack,sizeof(recvdatapack));

    //TODO
    float x = recvdatapack.x;
    float y = recvdatapack.y;
    float z = recvdatapack.z;
    float roll = recvdatapack.roll;
    float pitch = recvdatapack.pitch;
    float yaw =  recvdatapack.yaw;
    cout << "x: "<< x << ", y: " << y << ", yaw: " << yaw << endl;



    float min_distance = 1000;
    int time_index = 10000;
    float distance = 0;

    for(int k = 0; k < queue_i; k++)
    {
        distance = hypot(x - Pose[k].x, y - Pose[k].y);
        //1. to eradicate the keyframes on the time-related sequence 
        //2. to find the closest point to the determined pose
        
         cout << "show index: " << Pose[k].index << " distance: "<< distance << endl;
        //if(time_index - Pose[k].index > 50)
        //{
            if(distance < min_distance)
            {
               //cout << "show index: " << Pose[k].index << " distance: "<< distance << endl;
               min_distance = distance;
               senddatapack.index = Pose[k].index;
            } 
        //}  
    }

    senddatapack.x = x;
    senddatapack.y = y;
    senddatapack.z = z;
    senddatapack.roll = roll;
    senddatapack.pitch = pitch;
    senddatapack.yaw = yaw;
    cout << "index= " << senddatapack.index << endl;
    write(sendfd,&senddatapack,sizeof(senddatapack));
}
    //feed the index back
    return 0;
}
