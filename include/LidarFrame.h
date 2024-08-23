# ifndef LIDAR_FRAME_H_
# define LIDAR_FRAME_H_

#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>

using QuatroPointType = pcl::PointXYZ; // can be changed

typedef pcl::PointCloud<QuatroPointType> PointCloud;
class LidarFrame
{

public:
    Eigen::Matrix4d pose_; //Identical as transform the point from base to odom;
    PointCloud::Ptr pcd_;

    LidarFrame(PointCloud::Ptr pcd, Eigen::Matrix4d pose) : pcd_(pcd), pose_(pose){};
};



# endif