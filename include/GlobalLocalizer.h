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

using QuatroPointType = pcl::PointXYZ; // can be changed

typedef pcl::PointCloud<QuatroPointType> PointCloud;

class GlobalLocalizer
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GlobalLocalizer();
    ~GlobalLocalizer();

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void loadMap();
    void publishPose();
    
    void initializeParticles(const Eigen::Matrix4d &quatro_transformation);
    Eigen::Matrix4d randomTransform();

    void weightParticles(PointCloud::Ptr cloud);
    void sensorUpdate(const PointCloud::Ptr &cloud);
    void resampleParticles();
    
    double computeMSE(const PointCloud &cloud1, const PointCloud &cloud2);
    void upateTransformation(Eigen::Matrix4d transformation);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher map_pub_;
    ros::Publisher Quatro_pub_;
    ros::Publisher MCL_pub_;
    ros::Publisher Particle_pub_;

    tf::TransformBroadcaster *br_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    pcl::GeneralizedIterativeClosestPoint<QuatroPointType, QuatroPointType> gicp_;
    PointCloud::Ptr map_;

    Eigen::Matrix4d map_to_odom_transformation_;

    std::shared_ptr<quatro<QuatroPointType>> m_quatro_handler;

    std::thread pose_publishing_thread_;

    // GICP parameters
    int gicp_max_iterations_;
    double gicp_transformation_epsilon_;
    double gicp_euclidean_fitness_epsilon_;

    // Voxelize parameters
    double voxel_leaf_size_;

    // Quatro parmeters
    double quatro_mse_th_;

    // MCL parameters
    int num_particles_;
    double mcl_converge_th_;
    int mcl_max_iteration_;
    double mcl_mse_th_;
    double particle_x_bound_, particle_y_bound_, particle_z_bound_, particle_yaw_bound_;

    double best_mse_ = 100;
    struct Particle
    {
        Eigen::Matrix4d pose;
        double weight;
        double score;
    };

    std::vector<Particle> particles_;
};

#endif
