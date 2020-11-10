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
#include <vector>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>

#include "../include/QuadTree.h" //structure define and the other shits

using namespace std;

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
void queryEle(struct QuadTreeNode node, struct ElePoint ele, float candidate[][3], int count) 
{
    if (node.is_leaf == 1) 
    {
        for (int j = 0; j < node.ele_num; j++) 
        {
            candidate[count][0] = node.ele_list[j]->x;
            candidate[count][1] = node.ele_list[j]->y;
            candidate[count][2] = node.ele_list[j]->index;
            count++;
            candidate[MAX_ELE_NUM*9][0] = count;
        }
        return;
    }

    float mid_vertical = (node.region.up + node.region.bottom) / 2;
    float mid_horizontal = (node.region.left + node.region.right) / 2;
    if (ele.y > mid_vertical) 
    {
        if (ele.x > mid_horizontal) 
        {
            queryEle(*node.RU, ele, candidate, count);
        } 
        else 
        {
            queryEle(*node.LU, ele, candidate, count);
        }
    } 
    else 
    {
        if (ele.x > mid_horizontal) 
        {
            queryEle(*node.RB, ele, candidate, count);
        } 
        else 
        {
            queryEle(*node.LB, ele, candidate, count);
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
