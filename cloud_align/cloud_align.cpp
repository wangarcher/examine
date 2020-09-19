/*

Project: basic icp
Date: 20/08/04
@Author: Yang, Wang.
Detail: align numberous pointclouds 
Scenario: VO

*/


#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_lm.h>


#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/types.h>
#include <string.h>
#include <dirent.h>
#include <stdio.h>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormal;
//typedef Eigen::Matrix<double, 4, 4> Matrix4f;


//#pragma pack(1)

struct PCDMap
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int index =0;
    PointCloud::Ptr cloud;
    Eigen::Matrix4f global_transform;

    //Eigen::Vector3d move;
    //Eigen::Quaterniond q;

    //Eigen::Vector3d move_after_g2o;
    //Eigen::Quaterniond q_after_g2o;
    bool doing_g2o = false;
    PCDMap():cloud(new PointCloud){};
};

//uniform sampling func
//data acquired: Cloud's ptr for processing.
void uniform_sampling (PointCloud::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//cout << "原始点云个数：" << cloud->points.size() << endl;
	pcl::UniformSampling<pcl::PointXYZ> US;
   	US.setInputCloud(cloud);
    	US.setRadiusSearch(0.2f);
    	US.filter(*cloud_filtered);
	*cloud = *cloud_filtered;
    	//cout << "均匀采样之后点云的个数：" << cloud->points.size() << endl;
}

//remove singel func
//data acquired: Cloud's ptr for processing.
void remove_single (PointCloud::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        //cout << "原始点云个数：" << cloud->points.size() << endl;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> RS;
	RS.setInputCloud(cloud);
	RS.setRadiusSearch(0.5);
	RS.setMinNeighborsInRadius(4);
	RS.filter(*cloud_filtered);
        *cloud = *cloud_filtered;
	//cout << "除去离群点后点云的个数：" << cloud->points.size() << endl;
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


//icp point align algorithm
//data acquired: 1 & 2 is the Cloud's ptr for aligning, 3 is the Cloud's ptr for output, 4 is transformation matrix, 5 is unclear.
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f&final_transform, int n)
{
	int iterations = 100;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_src);
        icp.setInputTarget(cloud_tgt);
        icp.setTransformationEpsilon(1e-15);
	icp.setMaxCorrespondenceDistance(2);
        icp.setEuclideanFitnessEpsilon(0.001);
        icp.setMaximumIterations(iterations);
         
	icp.align(*output);
	//cout<<"\nThe "<<n<< " frame and the "<<n+1<<" frame have converged and score is "<<icp.getFitnessScore()<<endl;
	//cout<<"Transformation is\n"<<icp.getFinalTransformation()<<endl;
	final_transform = icp.getFinalTransformation();
}

//.pcd viewer related
//data acquired: Cloud's ptr for display
void showPCD (const PointCloud::Ptr clouddata)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer_final (new pcl::visualization::PCLVisualizer("result1"));
        viewer_final->setBackgroundColor(0, 0, 0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>clouddatacolor (clouddata, 0, 255, 0);
        viewer_final->addPointCloud<pcl::PointXYZ> (clouddata, clouddatacolor, "cloud data");
        viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud data");
	while (!viewer_final->wasStopped())
        {
                viewer_final->spinOnce(100);
	}

}




int main(int argc, char** argv)
{
    vector<PCDMap> data;
    vector<PCDMap> secdata;
    vector<PCDMap> thidata;
    vector<PCDMap> findata;

    string data_c1 = argv[1];
    string data_c2 = argv[2];
    //string data_c3 = argv[3];
    int data_s = stoi(data_c1);
    int data_e = stoi(data_c2);
    //int data_num = stoi(data_c3);
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(int i = data_s; i < data_e; i=i+2)
    {
        char namebuf[20];
        sprintf(namebuf, "../%d.pcd", i);
        cout<<namebuf<<endl;
        PCDMap m;
        pcl::io::loadPCDFile (namebuf, *m.cloud); // set the struct.cloud
        m.index = i;                              // set the struct.index
        //m.afterg2o = Eigen::Matrix4d::Identity();
        data.push_back(m);
    }

    PointCloud::Ptr source, target;
    int j = 0;
    int l = 0;
    int n = 0;

    //refinied 20/08/07
    //the first loop of align, for pre-process
    cout<<data.size()<<endl;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    for(size_t i = 0; i<data.size()-6; i=i+5)
    {
        target = data[i].cloud;
        uniform_sampling(target);
        remove_single(target);

        PointCloud::Ptr mid (new PointCloud);
        for(int fif = 1; fif < 6; fif++)
        {
            source = data[i+fif].cloud;
            uniform_sampling(source);
            remove_single(source);

            PointCloud::Ptr temp (new PointCloud);
            pairAlign(source, target, temp, pairTransform, i);
            pcl::transformPointCloud(*source, *mid, pairTransform);
            uniform_sampling(target);
            remove_single(target);
            remove_distant(target);
            *target = *target + *mid;
        }
        cout<<"1st: "<< j <<endl;
        GlobalTransform = pairTransform*GlobalTransform;
        PointCloud::Ptr target_1st (new PointCloud);
        pcl::transformPointCloud(*target, *target_1st, GlobalTransform);
        j++;                 
        PCDMap m_1st;
        *m_1st.cloud = *target_1st;
        m_1st.index = j;
        m_1st.global_transform = GlobalTransform;
        secdata.push_back(m_1st);
    }




    //the second loop of align, for the keyframe
    cout<<secdata.size()<<endl;
    PointCloud::Ptr source2, target2;
    Eigen::Matrix4f GlobalTransform2 = Eigen::Matrix4f::Identity (), pairTransform2;
    for(size_t k = 0; k< secdata.size()-4; k=k+3)
    {
        target2 = secdata[k].cloud;
        PointCloud::Ptr mid2 (new PointCloud);
        for(int fif = 1; fif < 4; fif++)
        {
            source2 = secdata[k+fif].cloud;
            PointCloud::Ptr temp (new PointCloud);
            pairAlign(source2, target2, temp, pairTransform2, k);
            pcl::transformPointCloud(*source2, *mid2, pairTransform2);
            *target2 = *target2 + *mid2;
            uniform_sampling(target2);
            remove_single(target2);

        }
        cout<<"2nd: "<< l <<endl;
        GlobalTransform2 = pairTransform2*GlobalTransform2;
        PointCloud::Ptr target_2nd (new PointCloud);
        pcl::transformPointCloud(*target2, *target_2nd, GlobalTransform2);
        l++;                 
        PCDMap m_2nd;
        *m_2nd.cloud = *target_2nd;
        m_2nd.index = l;
        m_2nd.global_transform = GlobalTransform2;
        thidata.push_back(m_2nd);
    }

    //the third loop of align, for the result
    cout<<thidata.size()<<endl;
    PointCloud::Ptr source3, target3;
    Eigen::Matrix4f GlobalTransform3 = Eigen::Matrix4f::Identity (), pairTransform3;
    PointCloud::Ptr result (new PointCloud);
    result = thidata[0].cloud;
    for(size_t m = 1; m < thidata.size();m++)
    {    
        target3 = thidata[m-1].cloud;
        source3 = thidata[m].cloud;
        PointCloud::Ptr temp (new PointCloud);
        PointCloud::Ptr mid3 (new PointCloud);
        pairAlign(source3, target3, temp, pairTransform3, m);
        GlobalTransform3 = pairTransform3*GlobalTransform3;
        pcl::transformPointCloud(*source3, *mid3, GlobalTransform3);
        *result = *result + *mid3;

        PCDMap m_fin;
        m_fin.index = n;
        m_fin.global_transform = GlobalTransform3;
        findata.push_back(m_fin);

    }

    //get the estimated global transform of the No. num keyframe
    //int num = data_num;
    for(size_t o = 0; o < secdata.size()-5;o++)
    {
        Eigen::Matrix4f GlobalTransform_real = Eigen::Matrix4f::Identity ();
        cout << "We would like to have the global transform of the No. " << o << " frame" << endl;
        GlobalTransform_real = secdata[o].global_transform * thidata[o/3].global_transform * findata[o/3].global_transform;
        cout << GlobalTransform_real << endl;
    }

    uniform_sampling(result);
    remove_single(result);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
    cout << "time cost is " << time_used.count() << " seconds." << endl;
    pcl::io::savePCDFileASCII ("../test.pcd", *result);
    showPCD(result);


}

