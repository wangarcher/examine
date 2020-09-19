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

//socket communication related
#define DEST_PORT  8000
#define DEST_IP_ADDRESS "127.0.0.1"

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
void calcuCloudCentre(PointCloud::Ptr cloud, double *x_cen, double *y_cen, double *z_cen)
{
    double x, y, z = 0;
    int size = cloud->points.size();
    for(size_t i = 0; i < size; i++)
    {
        x = x + cloud->points[i].x;
        y = y + cloud->points[i].y;
        z = z + cloud->points[i].z;
    }
    *x_cen = x / size;
    *y_cen = y / size;
    *z_cen = z / size;
}


//a func to find out the border limitations of the cluster
void calcuCloudBorder(PointCloud::Ptr cloud,
                      double *x_left, double *y_bottom, double *z_close,
                      double *x_right, double *y_up, double *z_far)
{
    double x_l, x_r, y_b, y_u, z_c, z_f = 0;
    int size = cloud->points.size();
    for(size_t i = 0; i < size; i++)
    {
        if(cloud->points[i].x > x_r)  x_r = cloud->points[i].x;
        if(cloud->points[i].y > y_u)  y_u = cloud->points[i].y;
        if(cloud->points[i].z > z_f)  z_f = cloud->points[i].z;
        if(cloud->points[i].x < x_l)  x_l = cloud->points[i].x; 
        if(cloud->points[i].y < y_b)  y_b = cloud->points[i].y;
        if(cloud->points[i].z < z_c)  z_c = cloud->points[i].z;       
    }
    *x_left = x_l;
    *y_bottom = y_b;
    *z_close = z_c;
    *x_right = x_r;
    *y_up = y_u;
    *z_far = z_f;
}

//a func for the socket communication
void socketCommu(double x_cen, double z_cen, int flag, 
                 double x_left, double x_right,
                 double y_bottom, double y_up)
{
    int sock_fd;
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)  
    {  
        perror("socket");  
        exit(1);  
    }
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(DEST_IP_ADDRESS);
    addr_serv.sin_port = htons(DEST_PORT);
    len = sizeof(addr_serv);
    int send_num1;
    char send_coord[100] = {0};
 
    sprintf(send_coord, "Coordinate: %f, %f, Property: %d", x_cen, z_cen, flag); 
    send_num1 = sendto(sock_fd, send_coord, strlen(send_coord), 0, (struct sockaddr *)&addr_serv, len);
    if(send_num1 < 0)
    {
        perror("sendto error: package I");
        exit(1);
    }
    int send_num2;
    char send_size[100] = {0};
 
    sprintf(send_size, "Length: %f, Height: %f",  x_right - x_left,  y_up - y_bottom); 
    send_num2 = sendto(sock_fd, send_size, strlen(send_size), 0, (struct sockaddr *)&addr_serv, len);
    if(send_num2 < 0)
    {
        perror("sendto error: package II");
        exit(1);
    }
    
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
	remove_distant(cloud);
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
    pcl::visualization::PCLVisualizer viewer("object segmention result");
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {

        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        
        double x_cen, y_cen, z_cen = 0;
        double x_left, y_bottom, z_close, x_right, y_up, z_far = 0;
        double distance = 0;
        int flag = 0;
        
        calcuCloudCentre(cloud_cluster, &x_cen, &y_cen, &z_cen);
        calcuCloudBorder(cloud_cluster, &x_left, &y_bottom, &z_close, &x_right, &y_up, &z_far);
        distance = hypot(x_cen, z_cen);
        //cout << "#######################################" << endl;
        //cout << x_cen << ", " << y_cen << ", " << z_cen << ", " << endl;
        //cout << "#######################################" << endl;
        //cout << x_left << ", " << y_bottom << ", "<< z_close << ", " 
         //    << x_right << ", " << y_up << ", " << z_far << endl; 

        if (y_up < 0.25 && distance < 20)
        {
            cout << "Be Advised! Found a possible contaminant object locate in " << x_cen << ", " << z_cen << endl;
            flag = 1;
        }
        else
        {
            cout << "Be Cautious! Found a potential obstacle locate in " << x_cen << ", " << z_cen << endl;
            flag = -1;
        }
        socketCommu(x_cen, z_cen, flag, x_left, x_right, y_bottom, y_up);
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        pcl::visualization::PointCloudColorHandlerRandom<PointT> cloud_in_color_h(cloud);//randomly tint some color
        viewer.addPointCloud(cloud_cluster, cloud_in_color_h, std::to_string(j));
        j++;
    }
    
    //wait 
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);

    }

    return (0);
}
