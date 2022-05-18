#include "ransac_ground_filter/ground_points_filter.h"
GroundPointsFilter::GroundPointsFilter(void) : private_nh("~")
{
    private_nh.param("SET_DISTANCE_THRE", SET_DISTANCE_THRE, {1.0});
    private_nh.param("RATIO",RATIO , {0.6});
    private_nh.param("MIN_COUNT", MIN_COUNT, {20});

    sub_points = nh.subscribe("/velodyne_points",1, &GroundPointsFilter::cloud_callback,this);
    pub_points_wo_ground = nh.advertise<sensor_msgs::PointCloud2>("/without_ground_points", 1);
}
void GroundPointsFilter::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "callback!!!" << std::endl;
    pcl::fromROSMsg(*msg, *cloud_ptr);
	std::cout << "cloud size: " << cloud_ptr->points.size() << std::endl;
    ground_cloud_ptr->header = cloud_ptr->header;
    double cloud_size = cloud_ptr->points.size();
    double ratio = RATIO;
    double min_count = MIN_COUNT; // 要調整
    while_plane_removal(cloud_size, ratio, min_count);
    pub_points_wo_ground.publish(ground_cloud_ptr);
	std::cout << "ground size: " << ground_cloud_ptr->points.size() << std::endl;
	ground_cloud_ptr->clear();
}
void GroundPointsFilter::while_plane_removal(double first_cloud_size, double ratio, int min_count)
{
	int count = 0;
	//std::cout << "first cloud size: " << first_cloud_size << std::endl;
	while(true){
		pcl::SACSegmentation<PointXYZIN> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		//CloudXYZINPtr mem_cloud(new CloudXYZIN);
		pcl::ExtractIndices<PointXYZIN> extract;

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100); // memo 100
		seg.setDistanceThreshold(SET_DISTANCE_THRE); //閾値
		Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0); //y axis
		seg.setAxis(axis);
		seg.setEpsAngle(30.0f * (M_PI/180.0f));
		seg.setInputCloud(cloud_ptr);
		seg.segment(*inliers, *coefficients);
		// 平面除去
        if(inliers->indices.size() == 0)
            break;
		extract.setInputCloud(cloud_ptr);
		extract.setIndices(inliers);
		extract.setNegative(true); // false にすると壁・地面を除去
		extract.filter(*cloud_ptr);
		*ground_cloud_ptr += *cloud_ptr; // 平面追加
		count ++;
		if((double)cloud_ptr->points.size() / first_cloud_size < ratio || count > min_count)
			break;
	}
	std::cout << "extract count: " << count << std::endl;
}
void GroundPointsFilter::process()
{
    ros::spin();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_points_filter");
    GroundPointsFilter ground_points_filter;
    ground_points_filter.process();
    return 0;
}