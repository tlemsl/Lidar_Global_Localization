#include "GlobalLocalizer.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <thread>
#include <chrono>
#include <random>

Eigen::Matrix4d convertTFToEigen(const geometry_msgs::TransformStamped& transformStamped) {
    Eigen::Matrix4d eigenMatrix = Eigen::Matrix4d::Identity();

    // Set the translation
    eigenMatrix(0, 3) = transformStamped.transform.translation.x;
    eigenMatrix(1, 3) = transformStamped.transform.translation.y;
    eigenMatrix(2, 3) = transformStamped.transform.translation.z;

    // Set the rotation (convert from quaternion to rotation matrix)
    Eigen::Quaterniond quaternion(transformStamped.transform.rotation.w,
                                  transformStamped.transform.rotation.x,
                                  transformStamped.transform.rotation.y,
                                  transformStamped.transform.rotation.z);
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    eigenMatrix.block<3,3>(0,0) = rotationMatrix;

    return eigenMatrix;
}

geometry_msgs::TransformStamped convertEigenToTF(const Eigen::Matrix4d& eigenMatrix, const std::string& target_frame, const std::string& source_frame, const ros::Time& time) {
    geometry_msgs::TransformStamped transformStamped;

    // Set the header information
    transformStamped.header.stamp = time;
    transformStamped.header.frame_id = target_frame;
    transformStamped.child_frame_id = source_frame;

    // Extract translation
    transformStamped.transform.translation.x = eigenMatrix(0, 3);
    transformStamped.transform.translation.y = eigenMatrix(1, 3);
    transformStamped.transform.translation.z = eigenMatrix(2, 3);

    // Extract rotation matrix
    Eigen::Matrix3d rotationMatrix = eigenMatrix.block<3, 3>(0, 0);

    // Convert rotation matrix to quaternion
    Eigen::Quaterniond quaternion(rotationMatrix);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    return transformStamped;
}



GlobalLocalizer::GlobalLocalizer() { 
    nh_ = ros::NodeHandle("~");

    // Subscribe to point cloud topic
    sub_ = nh_.subscribe("/Laser_map", 1, &GlobalLocalizer::cloudCallback, this);
    
    // Initialize publishers
    robot_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/voxelized_map", 1);
    Quatro_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/QUATRO_result", 1);
    MCL_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/MCL_result", 1);
    Particle_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1);

    // Read parameters
    nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.5);
    nh_.param("quatro_mse_th", quatro_mse_th_, 0.1);

    // MCL parameters
    nh_.param("num_particles", num_particles_, 30);
    nh_.param("mcl_converge_th", mcl_converge_th_, 0.01);
    nh_.param("mcl_max_iteration", mcl_max_iteration_, 20);
    nh_.param("mcl_mse_th", mcl_mse_th_, 0.1);
    nh_.param("particle_x_bound", particle_x_bound_, 1.0);
    nh_.param("particle_y_bound", particle_y_bound_, 1.0);
    nh_.param("particle_z_bound", particle_z_bound_, 0.5);
    nh_.param("particle_yaw_bound", particle_yaw_bound_, 0.2);


    // GICP parameters
    nh_.param("gicp_max_iterations", gicp_max_iterations_, 50);
    nh_.param("gicp_transformation_epsilon", gicp_transformation_epsilon_, 1e-8);
    nh_.param("gicp_euclidean_fitness_epsilon", gicp_euclidean_fitness_epsilon_, 1e-5);

    nh_.param("terminal_mse", terminal_mse_, 0.01);
    nh_.param("update_period", update_period_, 3);


    gicp_.setMaximumIterations(gicp_max_iterations_);
    gicp_.setTransformationEpsilon(gicp_transformation_epsilon_);
    gicp_.setEuclideanFitnessEpsilon(gicp_euclidean_fitness_epsilon_);
    
    // Load and voxelize the initial map from the PCD file
    loadMap();

    // Initialize Quatro
    m_quatro_handler = std::make_shared<quatro<QuatroPointType>>(3.5, 5.0, 0.3, 1.4, 0.00011, 500, false);

    // Initialize transform broadcaster
    br_ = new tf::TransformBroadcaster();
    
    // Set up the TF listener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Initialize last transformation to identity
    odom_to_map_transfromation_ = Eigen::Matrix4d::Identity();

    // Start the publishPose function in a separate thread
    pose_publishing_thread_ = std::thread(&GlobalLocalizer::publishPose, this);
}

GlobalLocalizer::~GlobalLocalizer() {
    if (pose_publishing_thread_.joinable()) {
        pose_publishing_thread_.join();
    }
}

void GlobalLocalizer::initializeParticles(const Eigen::Matrix4d& quatro_transformation) {
    particles_.clear();

    for (int i = 0; i < num_particles_; ++i) {
        Particle particle;
        particle.pose = quatro_transformation * randomTransform();
        particle.weight = 1.0 / num_particles_;
        particles_.emplace_back(particle);
    }
}

Eigen::Matrix4d GlobalLocalizer::randomTransform() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(-particle_x_bound_, particle_x_bound_);
    std::uniform_real_distribution<> dis_y(-particle_y_bound_, particle_y_bound_);
    std::uniform_real_distribution<> dis_z(-particle_z_bound_, particle_z_bound_);
    std::uniform_real_distribution<> dis_yaw(-particle_yaw_bound_, particle_yaw_bound_);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(dis_x(gen), dis_y(gen), dis_z(gen));
    double yaw = dis_yaw(gen);
    Eigen::AngleAxisd rot_yaw(yaw, Eigen::Vector3d::UnitZ());
    transform.block<3, 3>(0, 0) = rot_yaw.matrix();
    return transform;
}

void GlobalLocalizer::weightParticles(PointCloud::Ptr cloud) {
    double sum_weight = 0;
    for (auto& particle : particles_) {
        pcl::PointCloud<QuatroPointType>::Ptr transformed_cloud(new pcl::PointCloud<QuatroPointType>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, particle.pose);

        // Use ICP to refine the alignment and compute the fitness score
        pcl::IterativeClosestPoint<QuatroPointType, QuatroPointType> icp;
        icp.setInputSource(transformed_cloud);
        icp.setInputTarget(map_);
        pcl::PointCloud<QuatroPointType> aligned_cloud;
        
        icp.align(aligned_cloud);
        if(icp.hasConverged()){
            particle.pose = particle.pose * icp.getFinalTransformation().cast<double>();

            pcl::PointCloud<QuatroPointType>::Ptr refined_cloud(new pcl::PointCloud<QuatroPointType>);

            pcl::transformPointCloud(*cloud, *refined_cloud, particle.pose);
        
            // particle.score = icp.getFitnessScore();
            particle.score = computeMSE(*refined_cloud, *map_);
        }
        else{
            particle.score *= 1000;
        }
        particle.weight = 1.0 / particle.score;
        sum_weight += particle.weight;
    }
    for (auto& particle : particles_) {
        particle.weight = particle.weight / sum_weight;
    }

}

void GlobalLocalizer::resampleParticles() {
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    // Calculate the total weight and prepare for sampling
    std::vector<double> weights;
    for (const auto& p : particles_) {
        weights.push_back(p.weight);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> dist(weights.begin(), weights.end());

    for (int i = 0; i < num_particles_; i++) {
        int idx = dist(gen);
        new_particles.push_back(particles_[idx]);
    }

    particles_ = std::move(new_particles);
}

void GlobalLocalizer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ROS_INFO_STREAM("\n\n\nReceived point cloud message.");

    if(best_mse_ < terminal_mse_){
        ROS_INFO_STREAM("Already reach the terminal mse: " << best_mse_);
        std::this_thread::sleep_for(std::chrono::seconds(100));
        return;
    }
    
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
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.filter(*downsampled_cloud);

    ROS_INFO_STREAM("Downsampled cloud size: " << downsampled_cloud->points.size());

    // Check for NaN or infinite values in the downsampled cloud
    bool has_nan = false;
    for (const auto& point : downsampled_cloud->points) {
        if (!pcl::isFinite(point)) {
            has_nan = true;
            break;
        }
    }
    if (has_nan) {
        ROS_WARN("Downsampled cloud contains NaN or infinite values.");
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    bool if_valid_;
    ROS_INFO_STREAM("Quatro Aligning point clouds...");
    PointCloud::Ptr quatro_cloud(new PointCloud);
    Eigen::Matrix4d quatro_transformation = m_quatro_handler->align(*downsampled_cloud, *map_, if_valid_);
    pcl::transformPointCloud(*downsampled_cloud, *quatro_cloud, quatro_transformation);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO_STREAM("Quatro alignment took " << elapsed.count() << " seconds.");
    
    // Calculate MSE
    double quatro_mse = computeMSE(*quatro_cloud, *map_);
    ROS_INFO_STREAM("MSE between Quatro cloud and map: " << quatro_mse);

    double mcl_mse = 100;
    Eigen::Matrix4d mcl_transformation;
    if (if_valid_ && quatro_mse < quatro_mse_th_) {  // Check against Quatro MSE threshold
        ROS_INFO_STREAM("Quatro converged");

        start = std::chrono::high_resolution_clock::now();

        // Initialize particles based on Quatro transformation
        initializeParticles(quatro_transformation);
        // Calculate MSE for MCL

        PointCloud::Ptr mcl_cloud(new PointCloud);

        for (int i = 0; i < mcl_max_iteration_; ++i) {
            // Weight particles based on ICP alignment
            weightParticles(downsampled_cloud);
            resampleParticles();
            
            // Find the particle with the minimum score
            auto max_weight_particle = std::max_element(particles_.begin(), particles_.end(), 
                [](const Particle& a, const Particle& b) {
                    return a.weight < b.weight;
                });

            if (max_weight_particle != particles_.end()) {
                mcl_transformation = max_weight_particle->pose;
            }

            pcl::transformPointCloud(*downsampled_cloud, *mcl_cloud, mcl_transformation );
            mcl_mse = computeMSE(*mcl_cloud, *map_);
            ROS_INFO_STREAM("MSE between MCL cloud and map: " << mcl_mse);
            
            if (mcl_mse < mcl_mse_th_ && mcl_mse < quatro_mse && mcl_mse < best_mse_) {
                ROS_INFO_STREAM("MCL converged.");
                break;
            }

            // Visualize particles
            geometry_msgs::PoseArray particle_poses;
            particle_poses.header.frame_id = "map";
            particle_poses.header.stamp = ros::Time::now();

            for (const auto& particle : particles_) {
                geometry_msgs::Pose pose_msg;
                pose_msg.position.x = particle.pose(0, 3);
                pose_msg.position.y = particle.pose(1, 3);
                pose_msg.position.z = particle.pose(2, 3);
                Eigen::Quaterniond q(particle.pose.block<3, 3>(0, 0).cast<double>());
                pose_msg.orientation.x = q.x();
                pose_msg.orientation.y = q.y();
                pose_msg.orientation.z = q.z();
                pose_msg.orientation.w = q.w();
                particle_poses.poses.push_back(pose_msg);
            }

            Particle_pub_.publish(particle_poses);
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        ROS_INFO_STREAM("MCL refinement took " << elapsed.count() << " seconds.");
        

        // Publish the aligned point cloud for visualization
        sensor_msgs::PointCloud2 quatro_msg, mcl_msg;
        pcl::toROSMsg(*quatro_cloud, quatro_msg);
        pcl::toROSMsg(*mcl_cloud, mcl_msg);
        quatro_msg.header.frame_id = "map";
        mcl_msg.header.frame_id = "map";

        Quatro_pub_.publish(quatro_msg);
        MCL_pub_.publish(mcl_msg);
        double final_mse = quatro_mse < mcl_mse ? quatro_mse : mcl_mse;
        Eigen::Matrix4d final_transformation = quatro_mse < mcl_mse ? quatro_transformation : mcl_transformation;
        if(final_mse < best_mse_) {
            best_mse_ = final_mse;
            upateTransformation(final_transformation);
            ROS_INFO_STREAM("!!!Update Transformation with mse: " << best_mse_);
            ROS_INFO_STREAM(final_transformation);
        } 
    } else {
        ROS_WARN("QUATRO did not converge.");
    }
    std::this_thread::sleep_for(std::chrono::seconds(update_period_));
}

double GlobalLocalizer::computeMSE(const PointCloud& cloud1, const PointCloud& cloud2) {
    // Ensure both clouds are non-empty
    if (cloud1.empty() || cloud2.empty()) {
        ROS_WARN("One or both point clouds are empty.");
        return std::numeric_limits<double>::max();
    }

    // Perform nearest neighbor search for each point in cloud1 against cloud2
    pcl::KdTreeFLANN<QuatroPointType> kdtree;
    kdtree.setInputCloud(cloud2.makeShared());
    
    double mse = 0.0;
    int valid_points = 0;

    for (const auto& point : cloud1.points) {
        std::vector<int> point_idx(1);
        std::vector<float> point_squared_distance(1);

        if (kdtree.nearestKSearch(point, 1, point_idx, point_squared_distance) > 0) {
            mse += point_squared_distance[0];
            ++valid_points;
        }
    }

    if (valid_points > 0) {
        mse /= static_cast<double>(valid_points);
    } else {
        ROS_WARN("No valid points found for MSE calculation.");
        mse = std::numeric_limits<double>::max();
    }

    return mse;
}

void GlobalLocalizer::upateTransformation(Eigen::Matrix4d transformation) {
    // Get the odom->base_link transform from tf
    // geometry_msgs::TransformStamped base_to_odom;
    // try {
    //      base_to_odom = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    // } catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s", ex.what());
    //     return;
    // }
    // Eigen::Matrix4d base_to_odom_transformation = convertTFToEigen(base_to_odom);
    // odom_to_map_transfromation_ = transformation * base_to_odom_transformation.inverse();
    odom_to_map_transfromation_ = transformation;
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
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.setInputCloud(original_map);
    voxel_grid.filter(*map_);
    ROS_INFO("Downsampled map size: %zu points", map_->points.size());
}

void GlobalLocalizer::publishPose() {
    ros::Rate rate(100); // Publish at 100 Hz
    while (ros::ok()) {

        // Convert the map->odom transformation to a TransformStamped
        geometry_msgs::TransformStamped odom_to_map = convertEigenToTF(odom_to_map_transfromation_,
                                            "map", "odom",ros::Time::now());

        // Publish the map->odom transform
        tf_broadcaster_.sendTransform(odom_to_map);

        // Publish the robot_pose topic
        geometry_msgs::TransformStamped base_to_odom;
        try {
            base_to_odom = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            rate.sleep();
            continue;
        }

        Eigen::Matrix4d map_to_base_transformation = odom_to_map_transfromation_ * convertTFToEigen(base_to_odom);

        // Extract position and orientation from the Eigen matrix
        Eigen::Matrix4d base_pose = map_to_base_transformation;
        Eigen::Vector3d position = base_pose.block<3, 1>(0, 3);
        Eigen::Matrix3d rotationMatrix = base_pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotationMatrix);

        // Create and populate the PoseStamped message
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.stamp = ros::Time::now();
        robot_pose.header.frame_id = "map";
        robot_pose.pose.position.x = position.x();
        robot_pose.pose.position.y = position.y();
        robot_pose.pose.position.z = position.z();
        robot_pose.pose.orientation.x = quaternion.x();
        robot_pose.pose.orientation.y = quaternion.y();
        robot_pose.pose.orientation.z = quaternion.z();
        robot_pose.pose.orientation.w = quaternion.w();

        // Publish the PoseStamped message
        robot_pose_pub_.publish(robot_pose);

        rate.sleep();
    }
}
