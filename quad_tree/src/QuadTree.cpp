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

void combineNode(struct QuadTreeNode *node)  //not finished
{
    if (node->RU->is_leaf && node->LU->is_leaf && node->RB->is_leaf && node->LB->is_leaf)
    {
        if (node->RU->ele_num + node->LU->ele_num + node->RB->ele_num + node->LB->ele_num < 20)
        {
            for (int h = 0; h < node->RU->ele_num; h++) 
            {
                struct ElePoint *ele_ptr = (struct ElePoint *) malloc(sizeof(struct ElePoint));
                ele_ptr = node->RU->ele_list[h];
                node->ele_list[node->ele_num] = ele_ptr;
                node->ele_num++;
            }
            for (int i = 0; i < node->LU->ele_num; i++) 
            {
                struct ElePoint *ele_ptr = (struct ElePoint *) malloc(sizeof(struct ElePoint));
                ele_ptr = node->LU->ele_list[i];
                node->ele_list[node->ele_num] = ele_ptr;
                node->ele_num++;
            }
            for (int j = 0; j < node->RB->ele_num; j++) 
            {
                struct ElePoint *ele_ptr = (struct ElePoint *) malloc(sizeof(struct ElePoint));
                ele_ptr = node->RB->ele_list[j];
                node->ele_list[node->ele_num] = ele_ptr;
                node->ele_num++;
            }
            for (int k = 0; k < node->LB->ele_num; k++) 
            {
                struct ElePoint *ele_ptr = (struct ElePoint *) malloc(sizeof(struct ElePoint));
                ele_ptr = node->LB->ele_list[k];
                node->ele_list[node->ele_num] = ele_ptr;
                node->ele_num++;
            }
            node->depth--;
            node->is_leaf = 1;
            struct Region *region = (struct Region *) malloc(sizeof(struct Region));
            initRegion(region, node->LB->region.bottom, node->RU->region.up, node->LU->region.left, node->RB->region.right);
            node->region = *region;
            free(node->RU);
            free(node->LU);
            free(node->RB);
            free(node->LB);
        }
    }
    if (node->RU->is_leaf == 0) combineNode(node->RU);
    if (node->LU->is_leaf == 0) combineNode(node->LU);
    if (node->RB->is_leaf == 0) combineNode(node->RB);
    if (node->LB->is_leaf == 0) combineNode(node->LB);
    return;
}

void deleteEle(struct QuadTreeNode node, struct ElePoint old_ele)    //TODO test needed, A
{
    if (node.is_leaf == 1) 
    {
        for (int j = 0; j < node.ele_num; j++) 
        {
            if (old_ele.index == node.ele_list[j]->index)
            {
                free(node.ele_list[j]);
                for (int bp = j; bp < node.ele_num-1; bp++)
                {
                    node.ele_list[bp] = node.ele_list[bp+1];
                }
                node.ele_num--;
            }
        }
        return;
    }
    float mid_vertical = (node.region.up + node.region.bottom) / 2;
    float mid_horizontal = (node.region.left + node.region.right) / 2;
    if (old_ele.y > mid_vertical) 
    {
        if (old_ele.x > mid_horizontal) 
        {
            deleteEle(*node.RU, old_ele);
        } 
        else 
        {
            deleteEle(*node.LU, old_ele);
        }
    } 
    else 
    {
        if (old_ele.x > mid_horizontal) 
        {
            deleteEle(*node.RB, old_ele);
        } 
        else 
        {
            deleteEle(*node.LB, old_ele);
        }
    }
}

//A func to find the element near the current pose
//making attempts to find the closest grid and find the elements in that grid

void queryEle(struct QuadTreeNode node, struct ElePoint ele, float pose[][3], int count) 
{
    if (node.is_leaf == 1) 
    {
        for (int j = 0; j < node.ele_num; j++) 
        {
            cout << "suibianxiajibashuchudiandongxi: " << node.ele_list[j]->x << ", " << node.ele_list[j]->y << endl;
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
    struct QuadTreeNode root;
    struct Region root_region;
    struct ElePoint ele;
    struct ElePoint old_ele; 
    initRegion(&root_region, - 50, 100, -50, 100);
    initNode(&root, 1, root_region);

    ifstream fin("../pose.txt");
    while(!fin.eof())
    {
        int queue_i;
        float queue_x, queue_y, queue_z, queue_yaw, queue_pitch, queue_roll;
        fin >> queue_i >> queue_yaw >> queue_pitch >> queue_roll >> queue_x >> queue_y >> queue_z;
        ele.x = queue_x;
        ele.y = queue_y;
        ele.index = queue_i;
        cout << "x_in: " << ele.x << ", y_in: " << ele.y << ", index_in:"<< ele.index << endl;
        insertEle(&root, ele);
        if(!fin.good()) break;
    }

    old_ele.x = -36.2755;
    old_ele.y = -10.2603;
    old_ele.index = 280;

    deleteEle(root, old_ele);



    float x = -36.0;
    float y = -10.0;
    float d = 0.4;
    float disturbance[9][2] = {{d, 0}, {d, d}, {0, d}, {0, 0}, {-d, d}, {-d, 0}, {-d, -d}, {0, -d}, {d, -d}};
    float pose[MAX_ELE_NUM*9][3] = {0};
    for(int i = 0; i < 9; i++)
    {   
        struct ElePoint test;
        test.x = x  + disturbance[i][0];
        test.y = y  + disturbance[i][1];
        queryEle(root, test, pose, pose[MAX_ELE_NUM*9-1][0]);  
    }

    float min_distance = 1000;
    float distance = 0;
    float found_x, found_y;
    int found_index = 0;
    for(int k = 0; k < pose[MAX_ELE_NUM*9-1][0]; k++)
    {
        distance = hypot(x - pose[k][0], y - pose[k][1]);
        //1. to eradicate the keyframes on the time-related sequence 
        //2. to find the closest point to the determined pose
        if(distance < min_distance)
        {
            min_distance = distance;
            found_x = pose[k][0];
            found_y = pose[k][1];
            found_index = pose[k][2];
        } 
    }
    cout << "found_x: " << found_x << ", found_y: " << found_y << ", found_index: " << found_index << endl;
}



