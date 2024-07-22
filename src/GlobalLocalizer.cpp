#include "GlobalLocalizer.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <chrono>

GlobalLocalizer::GlobalLocalizer() {
    nh_ = ros::NodeHandle("~");
    // Load and voxelize the initial map from the PCD file
    loadMap();

    // Subscribe to point cloud topic
    sub_ = nh_.subscribe("/Laser_map", 1, &GlobalLocalizer::cloudCallback, this);
    
    // Initialize publishers
    robot_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/voxelized_map", 1);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned", 1);
    
    // Initilize Quatro
    m_quatro_handler = std::make_shared<quatro<QuatroPointType>>(3.5, 5.0 , 0.3, 1.4, 0.00011, 500, false);

    // Initialize transform broadcaster
    br_ = new tf::TransformBroadcaster();
    
    // Set up the TF listener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Initialize last transformation to identity
    last_transformation_ = Eigen::Matrix4d::Identity();

    // Start the publishPose function in a separate thread
    pose_publishing_thread_ = std::thread(&GlobalLocalizer::publishPose, this);
}

GlobalLocalizer::~GlobalLocalizer() {
    if (pose_publishing_thread_.joinable()) {
        pose_publishing_thread_.join();
    }
}

void GlobalLocalizer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ROS_INFO_STREAM("Received point cloud message.");
    // Publish the voxelized map
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_, map_msg);
    map_msg.header.frame_id = "map";
    map_pub_.publish(map_msg);

    // Convert ROS PointCloud2 to PCL PointCloud
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    ROS_INFO_STREAM("Input cloud size: " << cloud->points.size());
    
    // Downsample the point cloud
    PointCloud::Ptr downsampled_cloud(new PointCloud);
    pcl::VoxelGrid<QuatroPointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);  // Adjust the leaf size based on your data
    voxel_grid.filter(*downsampled_cloud);

    ROS_INFO_STREAM("Downsampled cloud size: " << downsampled_cloud->points.size());

    // Check for NaN or infinite values in the downsampled cloud
    bool has_nan = false;
    for (const auto& point : downsampled_cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            has_nan = true;
            break;
        }
    }
    if (has_nan)
    {
        ROS_WARN("Downsampled cloud contains NaN or infinite values.");
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    bool if_valid_;
    ROS_INFO_STREAM("Quatro Aligning point clouds...");
    Eigen::Matrix4d transformation = (m_quatro_handler->align(*downsampled_cloud, *map_, if_valid_));
    pcl::transformPointCloud(*downsampled_cloud, *downsampled_cloud, transformation);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO_STREAM("Quatro alignment took " << elapsed.count() << " seconds.");
    if (if_valid_) {
        ROS_INFO_STREAM("Quatro converged. Transformation: \n" << transformation);
        last_transformation_ = transformation;

        // Publish the aligned point cloud for visualization
        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(*downsampled_cloud, aligned_msg);
        aligned_msg.header.frame_id = "map";
        aligned_pub_.publish(aligned_msg);
    } else {
        ROS_WARN("QUATRO did not converge.");
    }
}

void GlobalLocalizer::loadMap() {
    std::string pcd_file;
    nh_.param<std::string>("pcd_file", pcd_file, "map.pcd");
    PointCloud::Ptr original_map(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_file, *original_map) == -1)
    {
        ROS_ERROR("Couldn't read PCD file %s", pcd_file.c_str());
        return;
    }
    ROS_INFO("Loaded PCD file %s with %zu points", pcd_file.c_str(), original_map->points.size());

    // Downsample the map
    map_.reset(new PointCloud);
    pcl::VoxelGrid<QuatroPointType> voxel_grid;
    voxel_grid.setInputCloud(original_map);
    voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);  // Adjust the leaf size based on your data
    voxel_grid.filter(*map_);
    ROS_INFO("Downsampled map size: %zu points", map_->points.size());
}

void GlobalLocalizer::publishPose() {
    ros::Rate rate(10); // Publish at 10 Hz
    while (ros::ok()) {
        // Get the odom->base_link transform from tf
        geometry_msgs::TransformStamped odom_to_base_link;
        try {
            odom_to_base_link = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            rate.sleep();
            continue;
        }

        // Compute map->base_link transformation
        tf::Transform tf_map_to_base_link;
        tf::Matrix3x3 tf3d;
        tf3d.setValue(last_transformation_(0, 0), last_transformation_(0, 1), last_transformation_(0, 2),
                      last_transformation_(1, 0), last_transformation_(1, 1), last_transformation_(1, 2),
                      last_transformation_(2, 0), last_transformation_(2, 1), last_transformation_(2, 2));
        tf::Vector3 tfVec(last_transformation_(0, 3), last_transformation_(1, 3), last_transformation_(2, 3));
        tf_map_to_base_link.setBasis(tf3d);
        tf_map_to_base_link.setOrigin(tfVec);

        tf::Transform tf_odom_to_base_link;
        tf::transformMsgToTF(odom_to_base_link.transform, tf_odom_to_base_link);
        tf::Transform tf_map_to_odom = tf_map_to_base_link * tf_odom_to_base_link.inverse();

        // Convert the map->odom transformation to a TransformStamped
        geometry_msgs::TransformStamped map_to_odom;
        tf::transformTFToMsg(tf_map_to_odom, map_to_odom.transform);
        map_to_odom.header.stamp = ros::Time::now();
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";

        // Publish the map->odom transform
        tf_broadcaster_.sendTransform(map_to_odom);

        // Publish the robot_pose topic
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.stamp = ros::Time::now();
        robot_pose.header.frame_id = "map";
        robot_pose.pose.position.x = map_to_odom.transform.translation.x;
        robot_pose.pose.position.y = map_to_odom.transform.translation.y;
        robot_pose.pose.position.z = map_to_odom.transform.translation.z;
        robot_pose.pose.orientation = map_to_odom.transform.rotation;
        robot_pose_pub_.publish(robot_pose);

        rate.sleep();
    }
}
