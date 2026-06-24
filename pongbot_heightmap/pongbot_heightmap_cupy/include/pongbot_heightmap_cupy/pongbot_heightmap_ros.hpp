#pragma once

// STL
#include <iostream>
#include <mutex>
#include <cmath>

// Eigen
#include <Eigen/Dense>

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// Custom Message
#include <pongbot_heightmap_msgs/HeightMap.h>

// Grid Map
// #include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pongbot_heightmap_cupy/pongbot_heightmap_wrapper.hpp"

// LCM
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/hpp/feasibility_layer_t.hpp"
#include "lcm-types/hpp/elevation_layer_t.hpp"

#include "Utilities/Utilities.h"

namespace py = pybind11;

namespace pongbot_heightmap_cupy {

class HeightMapNode
{
    public:        
        HeightMapNode(ros::NodeHandle& nh);
        using RowMatrixXd               = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

        // Subscriber and Synchronizer for Pointcloud messages
        using PointCloudSubscriber      = message_filters::Subscriber<sensor_msgs::PointCloud2>;
        using PointCloudSubscriberPtr   = std::shared_ptr<PointCloudSubscriber>;
    
    private:
        void setupMapPublishers();

        void pointcloudCallback(const sensor_msgs::PointCloud2& cloud, const std::string& key);
        void inputPointCloud(const sensor_msgs::PointCloud2& cloud, const std::vector<std::string>& channels);
        
        void updateVariance(const ros::TimerEvent&);
        void updateTime(const ros::TimerEvent&);
        void updateHeightMap(const ros::TimerEvent&);
        void updateMapFrame(const ros::TimerEvent&);

        void publishMapROS(const ros::TimerEvent&);
        void publishMapLCM(const ros::TimerEvent&);

        void publishDataAsPoints(const HeightMapWrapper::RowMatrixXf& position, 
                                 const std::string name, 
                                 const HeightMapWrapper::RowMatrixXf& basic_layer, 
                                 const HeightMapWrapper::RowMatrixXf& data, 
                                 const ros::Publisher& pub,
                                 const float color_scale = 1.0f);
                                 
        void publishNormalArrow(const HeightMapWrapper::RowMatrixXf& normal_x,
                                const HeightMapWrapper::RowMatrixXf& normal_y,
                                const HeightMapWrapper::RowMatrixXf& normal_z,
                                const HeightMapWrapper::RowMatrixXf& elevation);


        visualization_msgs::Marker vectorToArrowMarker(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const int id);
        float getCellPosition(const int idx, const float center, const float resolution, const float map_length);             

        std::map<std::string, std::vector<std::string>> channels_;

        ros::NodeHandle nh_;

        ros::Publisher pointPub_, filterPub_, traversabilityPub_, validPub_;
        ros::Publisher normalPub_, esdfPub_, feasibilityPub_;

        std::vector<ros::Subscriber> pointcloudSubs_;
        tf::TransformListener transformListener_;

        ros::Timer updateHeightMapTimer_, updateMapFrameTimer_;
        ros::Timer publishROSTimer_, publishLCMTimer_;
        double updateFrameFps, updateMapFps;
        double publishROSFps, publishLCMFps;

        std::chrono::high_resolution_clock::time_point t_prev_map, t_prev_frame, t_prev_ros, t_prev_lcm; //TEST

        HeightMapWrapper map_;

        std::string mapFrameId_;
        std::string baseFrameId_;

        std::mutex mapMutex_;
        std::mutex poseMutex_;
        std::mutex rosPubMutex_;
        std::mutex lcmPubMutex_;
        std::mutex errorMutex_;

        lcm::LCM _visionLCM;
        elevation_layer_t elevation_layer_data_lcm;
        feasibility_layer_t feasibility_layer_data_lcm;

        std::shared_ptr<HeightMapWrapper::RowMatrixXf> elevation_ptr;
        std::shared_ptr<HeightMapWrapper::RowMatrixXf> filter_map_ptr;
        std::shared_ptr<HeightMapWrapper::RowMatrixXf> feasibility_ptr;
        std::shared_ptr<HeightMapWrapper::RowMatrixXf> map_pose_ptr;
        std::shared_ptr<HeightMapWrapper::RowMatrixXf> base_pose_ptr;

        // layer data
        std::vector<std::string> layers;    
        std::unordered_map<std::string, HeightMapWrapper::RowMatrixXf> layer_data;
};

}  // namespace pongbot_heightmap_cupy