#ifndef __GROUND_POINTS_FILTER_H
#define __GROUND_POINTS_FILTER_H

// #include <random>
#include <ros/ros.h>

// PCL
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

class GroundPointsFilter
{
public:
    GroundPointsFilter(void);

    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
    typedef pcl::PointXYZRGB PointXYZRGB;
    typedef pcl::PointCloud<PointXYZRGB> CloudXYZRGB;
    typedef pcl::PointCloud<PointXYZRGB>::Ptr CloudXYZRGBPtr;

    void process(void);
    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void while_plane_removal(double , double, int);

private:

    double SET_DISTANCE_THRE;
    double RATIO;
    double MIN_COUNT;

    CloudXYZINPtr ground_cloud_ptr{new CloudXYZIN};
    CloudXYZINPtr mem_cloud{new CloudXYZIN};
    CloudXYZINPtr temp_cloud{new CloudXYZIN}; // 壁以外
    CloudXYZINPtr cloud_ptr{new CloudXYZIN};

    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    //subscriber
    ros::Subscriber sub_points;
    //publisher
    ros::Publisher pub_points_wo_ground;
};

#endif