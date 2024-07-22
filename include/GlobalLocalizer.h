#ifndef GLOBAL_LOCALIZER_H_
#define GLOBAL_LOCALIZER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <quatro/quatro_module.h>

#include <thread>

using QuatroPointType = pcl::PointXYZ; //can be changed

typedef pcl::PointCloud<QuatroPointType> PointCloud;

class GlobalLocalizer {
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GlobalLocalizer();
    ~GlobalLocalizer();
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
private:
    void loadMap();
    void publishPose();

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher map_pub_;
    ros::Publisher aligned_pub_;
    tf::TransformBroadcaster* br_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    pcl::GeneralizedIterativeClosestPoint<QuatroPointType, QuatroPointType> gicp_;
    PointCloud::Ptr map_;
    Eigen::Matrix4d last_transformation_;
    std::shared_ptr<quatro<QuatroPointType>> m_quatro_handler;

    std::thread pose_publishing_thread_; 

};


#endif
