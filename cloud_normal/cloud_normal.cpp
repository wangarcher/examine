/*

Project: advanced icp, enhanced by the <point to plane> methodology
Date: 20/07/31
@Author: Wang
Detail: align two pointclouds with a larger angle difference(about 25 degrees)
Scenario: loop detection

*/

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

//filters
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudN;


typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;

//another attempt for p2p
//it seems worked, just for now
void point_to_plane (const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2, Eigen::Matrix4f&final_transform) 
{
    pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud1, *src);
    pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud2, *tgt);

    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
    norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
    norm_est.setKSearch (10);
    norm_est.setInputCloud (tgt);
    norm_est.compute (*tgt);

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
    icp.setTransformationEstimation(point_to_plane);
    icp.setInputCloud(src);
    icp.setInputTarget(tgt);
    //icp.setRANSACOutlierRejectionThreshold(ransac_par);
    //icp.setRANSACIterations(100);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-3);
    pcl::PointCloud<pcl::PointNormal> output;
    icp.align(output);
    final_transform = icp.getFinalTransformation();
}


//uniform sampling func
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void uniform_sampling (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    //cout << "Cloud points number is:" << cloud->points.size() << endl;
    pcl::UniformSampling<PointT> US;
    US.setInputCloud(cloud);
    US.setRadiusSearch(0.25f);
    US.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
    //cout << "Cloud points number after uniform sampling:" << cloud->points.size() << endl;
}


//remove singel func
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void remove_single (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    //cout << "Cloud points number is:" << cloud->points.size() << endl;
    pcl::RadiusOutlierRemoval<PointT> RS;
    RS.setInputCloud(cloud);
    RS.setRadiusSearch(0.5);
    RS.setMinNeighborsInRadius(4);
    RS.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
    //cout << "Cloud points number after remove single:" << cloud->points.size() << endl;

}

//another new filter to remove the point faraway from the origin
//added on 7/30 by Wang
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void remove_distant (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    //cout << "Cloud points number is:" << cloud->points.size() << endl;
    pcl::StatisticalOutlierRemoval<PointT> RD;
    RD.setInputCloud(cloud);    
    RD.setMeanK(50);
    RD.setStddevMulThresh(1.0);
    RD.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
    //cout << "Cloud points number after remove distant:" << cloud->points.size() << endl;

}

int main ()
{
    //open the .pcd file
    Eigen::Matrix4f transformation_matrix;
    Eigen::Matrix4f inverse_matrix;
    PointCloud::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud::Ptr cloud_mid (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile ("../1.pcd", *cloud_src);
    pcl::io::loadPCDFile ("../2.pcd", *cloud_tgt);
    uniform_sampling(cloud_src);
    remove_single(cloud_src);
    remove_distant(cloud_src);
    uniform_sampling(cloud_tgt);
    remove_single(cloud_tgt);
    remove_distant(cloud_tgt);


    point_to_plane(cloud_src, cloud_tgt, transformation_matrix);
    std::cout << transformation_matrix << std::endl;
    inverse_matrix = transformation_matrix.inverse();
    pcl::transformPointCloud (*cloud_tgt, *cloud_mid, inverse_matrix);
    *cloud_src = *cloud_src + *cloud_mid;


    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_src_handler (cloud_src, 0, 255, 0);//green
    viewer.addPointCloud (cloud_src, cloud_src_handler, "cloud_tgt");


    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

    return 0;
}


