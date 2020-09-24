/*

Project: plane detection
Date: 20/09/09
@Author: Wang. 
Detail: 1. refinement needed and test needed, 
           [Doing on 20/09/09], the performance was just fine, further test needed, to debug.
        2. socket communication
Scenario: obstacle, contaminant detection


*/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/make_shared.hpp>

#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <cstring>
#include <unistd.h>

//socket communication related
#define DEST_PORT  6061
#define DEST_IP_ADDRESS "192.168.0.168"
//socket datapack related



union fourtochar
{
	unsigned char ch[4];
	float fl;
	int in;
};

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//remove singel func
//data acquired: Cloud's ptr for processing.
void remove_single (PointCloud::Ptr cloud)
{
	PointCloud::Ptr cloud_filtered(new PointCloud);
	pcl::RadiusOutlierRemoval<PointT> RS;
	RS.setInputCloud(cloud);
	RS.setRadiusSearch(0.5);
	RS.setMinNeighborsInRadius(4);
	RS.filter(*cloud_filtered);
        *cloud = *cloud_filtered;
}

//another new filter to remove the point faraway from the origin
//added on 7/30 by Wang
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void remove_distant (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::StatisticalOutlierRemoval<PointT> RD;
    RD.setInputCloud(cloud);
    RD.setMeanK(50);
    RD.setStddevMulThresh(1.0);
    RD.filter(*cloud_filtered);
    *cloud = *cloud_filtered;

}

//a func to find out the centre of the cluster
void calcuCloudCentre(PointCloud::Ptr cloud, float *x_cen, float *y_cen, float *z_cen)
{
    float x = 0;
    float y = 0; 
    float z = 0;
    int size = cloud->points.size();
    //cout << x << endl;
    for(size_t i = 0; i < size; i++)
    {
        if(-100<cloud->points[i].x<100) x = x + cloud->points[i].x;
        if(-100<cloud->points[i].y<100) y = y + cloud->points[i].y;
        if(-100<cloud->points[i].z<100) z = z + cloud->points[i].z;
        //cout << x << endl;
    }
    *x_cen = x / size;
    *y_cen = y / size;
    *z_cen = z / size;
    //cout << *x_cen << " "  << *y_cen <<  " " << *z_cen << endl;  
}


//a func to find out the border limitations of the cluster
void calcuCloudBorder(PointCloud::Ptr cloud,
                      float *x_left, float *y_bottom, float *z_close,
                      float *x_right, float *y_up, float *z_far)
{
    float x_l = 0;
    float x_r = 0; 
    float y_b = 0;
    float y_u = 0;
    float z_c = 0;
    float z_f = 0;
    int size = cloud->points.size();
    for(size_t i = 0; i < size; i++)
    {
        if(cloud->points[i].x > x_r && -100<cloud->points[i].x<100)  x_r = cloud->points[i].x;
        if(cloud->points[i].y > y_u && -100<cloud->points[i].y<100)  y_u = cloud->points[i].y;
        if(cloud->points[i].z > z_f && -100<cloud->points[i].z<100)  z_f = cloud->points[i].z;
        if(cloud->points[i].x < x_l && -100<cloud->points[i].x<100)  x_l = cloud->points[i].x; 
        if(cloud->points[i].y < y_b && -100<cloud->points[i].y<100)  y_b = cloud->points[i].y;
        if(cloud->points[i].z < z_c && -100<cloud->points[i].z<100)  z_c = cloud->points[i].z;       
    }
    *x_left = x_l;
    *y_bottom = y_b;
    *z_close = z_c;
    *x_right = x_r;
    *y_up = y_u;
    *z_far = z_f;
    //cout << *x_left << " " << *y_bottom<< " " <<*z_close<< " "<< *x_right << " " << *y_up << " " << *z_far << endl; 
}

void socketCommu(float x_cen, float z_cen, 
                 float length, float height,
                 int flag, int j)
{

    char socketbuff[26] = {0};
    socketbuff[0] = 0xAA;
    socketbuff[1] = 0x55;
    socketbuff[2] = 23;           //length of the data
    socketbuff[3] = 0x77;
    socketbuff[4] = 0x10;



    if(flag == 1) socketbuff[5] = 0x01;
    if(flag == -1) socketbuff[5] = 0x02;
    if(flag == 0) socketbuff[5] = 0x03;  //flag 

    //data
    union fourtochar fotch1;
    fotch1.in = j;
    socketbuff[6] = fotch1.ch[0];
    socketbuff[7] = fotch1.ch[1];
    socketbuff[8] = fotch1.ch[2];
    socketbuff[9] = fotch1.ch[3];

    float sequence[4] = {x_cen, z_cen, length, height};
    for(int s = 0; s < 4; s++)
    {
        union fourtochar fotch2;
        fotch2.fl = sequence[s];
        socketbuff[10+s*4] = fotch2.ch[0];
        socketbuff[11+s*4] = fotch2.ch[1];
        socketbuff[12+s*4] = fotch2.ch[2];
        socketbuff[13+s*4] = fotch2.ch[3];

    }

    cout << "why?" << endl;
    //examine
    short int tempResult = 0x0000;
    for(int idx = 3; idx < 26; idx++)
    {
        tempResult += socketbuff[idx];
    }
    socketbuff[26] = (tempResult & 0xFF);


    for(int x=0; x < 27; x++)
        printf("%d,", socketbuff[x] & 0xFF);


    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_serv;
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(DEST_IP_ADDRESS);
    addr_serv.sin_port = htons(DEST_PORT);
    int send_num = sendto(sock_fd, socketbuff, 27, 0, (struct sockaddr *)&addr_serv, sizeof(addr_serv));
    close(sock_fd);
}



int main(int argc, char** argv)
{
    //cloud data in
    PointCloud::Ptr cloud(new PointCloud), cloud_f(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) 
    {
        PCL_ERROR("Couldn't read file .pcd\n");
        return(-1);
    }
    
    //pre process
	//remove_distant(cloud);
	remove_single(cloud);
    
    //segment parameter set
    pcl::SACSegmentation<PointT> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_plane(new PointCloud);
    pcl::PCDWriter writer;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int)cloud->points.size();
    while (cloud->points.size() > 0.5 * nr_points)
    {
        //Segment the largest planar component from the remaining cloud
        segmentation.setInputCloud(cloud);
        segmentation.segment(*inliers, *coefficients);
        //extract the points
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        //save to cloud_plane
        extract.filter(*cloud_plane);
        //std::cout <<cloud_plane->points.size() <<std::endl;
        //Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud = cloud_f;
    }
    
    //the creation of the KD tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl ::EuclideanClusterExtraction<PointT> ec;       //mean
    ec.setClusterTolerance(2);                         //2.5m search tolerance
    ec.setMinClusterSize(20);                          //the smallest number for clustering 
    ec.setMaxClusterSize(25000);                       //the largest number for clustering 
    ec.setSearchMethod(tree);                          //set method
    ec.setInputCloud(cloud);                           //set the cloud
    ec.extract(cluster_indices);                       //go.

    //for visualization and the others 
    //pcl::visualization::PCLVisualizer viewer("object segmention result");

    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {

        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        float x_cen, y_cen, z_cen = 0;
        float x_left, y_bottom, z_close, x_right, y_up, z_far = 0;
        float distance, length, height = 0;
        int flag = 0;

        calcuCloudCentre(cloud_cluster, &x_cen, &y_cen, &z_cen);
        calcuCloudBorder(cloud_cluster, &x_left, &y_bottom, &z_close, &x_right, &y_up, &z_far);
        distance = hypot(x_cen, z_cen);
        length = x_right - x_left;
        height = y_up - y_bottom;

/*
        cout << "#######################################" << endl;
        cout << x_cen << ", " << y_cen << ", " << z_cen << ", " << endl;
        cout << "#######################################" << endl;
        cout << x_left << ", " << y_bottom << ", "<< z_close << ", " 
             << x_right << ", " << y_up << ", " << z_far << endl; 
*/

        if (y_up < 0.25 && distance < 20)
        {
            //cout << "Be Advised! Found a possible contaminant object locate in " << x_cen << ", " << z_cen << endl;
            flag = 1;
        }
        else
        {
            //cout << "Be Cautious! Found a potential obstacle locate in " << x_cen << ", " << z_cen << endl;
            flag = -1;
        }

        socketCommu(x_cen, z_cen, length, height, flag, j);
        //pcl::visualization::PointCloudColorHandlerRandom<PointT> cloud_in_color_h(cloud);//randomly tint some color
        //viewer.addPointCloud(cloud_cluster, cloud_in_color_h, to_string(j));
        j++;
    }


    //wait 
   // while (!viewer.wasStopped())
   // {
   //     viewer.spinOnce(100);
   // }

    return (0);
}



