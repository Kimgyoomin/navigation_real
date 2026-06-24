/**
 * @author SungJoon Yoon  (densee250@gmail.com)
 * @version 1.0
 * @date 2025-03-10
 * @copyright Copyright (c) 2025, Robotics & Control Lab.
 *
 */

#include "pongbot_heightmap_cupy/pongbot_heightmap_ros.hpp"

// ROS
#include <tf_conversions/tf_eigen.h>

namespace pongbot_heightmap_cupy {

namespace
{
template <typename T>
bool getParamPrivateOrGlobal(
    const ros::NodeHandle& nh,
    const std::string& name,
    T& value)
{
    if (nh.getParam(name, value)) {
        return true;
    }

    if (ros::param::get("/" + name, value)) {
        return true;
    }

    return false;
}
}  // namespace

HeightMapNode::HeightMapNode(ros::NodeHandle& nh):
    _visionLCM(getLCMUrl(255))
{
    nh_ = nh;

    std::string pose_topic, map_frame;

    XmlRpc::XmlRpcValue publishers;
    XmlRpc::XmlRpcValue subscribers;

    // Read parameters
    // nh.getParam("/subscribers", subscribers);
    // nh.getParam("/publishers", publishers);

    if (!subscribers.valid()) 
    {
        ROS_FATAL("There are no subscribers set in parameters.");
    }
    if (!publishers.valid()) 
    {
        ROS_FATAL("There are no publishers set in parameters.");
    }

    // nh.param<std::string>("/map_frame", mapFrameId_, "map");
    // nh.param<std::string>("/base_frame", baseFrameId_, "base");
    
    // nh.param<double>("/update_frame_fps", updateFrameFps, 0.0);
    // nh.param<double>("/update_map_fps", updateMapFps, 0.0);
    
    // nh.param<double>("/publish_ROS_fps", publishROSFps, 0.0);
    // nh.param<double>("/publish_LCM_fps", publishLCMFps, 0.0);

    getParamPrivateOrGlobal(nh, "subscribers", subscribers);
    getParamPrivateOrGlobal(nh, "publishers", publishers);

    if (!getParamPrivateOrGlobal(nh, "map_frame", mapFrameId_)) {
        mapFrameId_ = "map";
    }

    if (!getParamPrivateOrGlobal(nh, "base_frame", baseFrameId_)) {
        baseFrameId_ = "base";
    }

    if (!getParamPrivateOrGlobal(nh, "update_frame_fps", updateFrameFps)) {
        updateFrameFps = 0.0;
    }

    if (!getParamPrivateOrGlobal(nh, "update_map_fps", updateMapFps)) {
        updateMapFps = 0.0;
    }

    if (!getParamPrivateOrGlobal(nh, "publish_ROS_fps", publishROSFps)) {
        publishROSFps = 0.0;
    }

    if (!getParamPrivateOrGlobal(nh, "publish_LCM_fps", publishLCMFps)) {
        publishLCMFps = 0.0;
    }

    for (auto& subscriber : subscribers) 
    {
        std::string name                                                    = subscriber.first;

        auto type                                                           = static_cast<std::string>(subscriber.second["data_type"]);
        
        // Initialize subscribers depending on the type
        if (type == "pointcloud") 
        {
            std::string pointcloud_topic                                    = subscriber.second["topic_name"];
            
            channels_[name].push_back("x");
            channels_[name].push_back("y");
            channels_[name].push_back("z");
            boost::function<void(const sensor_msgs::PointCloud2&)> function = boost::bind(&HeightMapNode::pointcloudCallback, this, _1, name);
            ros::Subscriber PointCloudSub                                   = nh_.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1, function);

            pointcloudSubs_.push_back(PointCloudSub);
        }
        else 
        {
            ROS_WARN_STREAM("Subscriber Data Type [" << type << "] is not valid.");
            continue;
        }
    }

    map_.initialize(nh_);
    
    layers = {"elevation", 
              "is_valid", 
              "filter_map", 
              "variance", 
              "traversability", 
              "normal_x", 
              "normal_y", 
              "normal_z", 
              "esdf",
              "feasibility"};

    // map_data                = HeightMapWrapper::RowMatrixXf::Zero(map_.map_n_, map_.map_n_);

    setupMapPublishers();

    pointPub_               = nh_.advertise<pongbot_heightmap_msgs::HeightMap>("height_map", 1);
    esdfPub_                = nh_.advertise<pongbot_heightmap_msgs::HeightMap>("esdf", 1);
    // validPub_               = nh_.advertise<pongbot_heightmap_msgs::HeightMap>("valid", 1);
    filterPub_              = nh_.advertise<pongbot_heightmap_msgs::HeightMap>("filter_map", 1);
    // traversabilityPub_      = nh_.advertise<pongbot_heightmap_msgs::HeightMap>("traversability", 1);
    normalPub_              = nh_.advertise<visualization_msgs::MarkerArray>("normal", 1);
    feasibilityPub_         = nh_.advertise<pongbot_heightmap_msgs::HeightMap>("feasibility", 1);
    
}

// Setup map publishers 
void HeightMapNode::setupMapPublishers() 
{
    if(updateMapFps > 0.)
    {
        double duration  = 1. / (updateMapFps + 0.00001);  
        updateHeightMapTimer_ = nh_.createTimer(ros::Duration(duration), &HeightMapNode::updateHeightMap, this);
    }
    
    if(updateFrameFps > 0.)
    {
        double duration  = 1. / (updateFrameFps + 0.00001);  
        updateMapFrameTimer_ = nh_.createTimer(ros::Duration(duration), &HeightMapNode::updateMapFrame, this, false, true);

        // 	arg this  : obj
        //  arg false : oneshot     = If true, this timer will only fire once
        // 	arg true  : autostart   = If true (default), return timer that is already started
    }

    if(publishROSFps > 0.)
    {
        double duration  = 1. / (publishROSFps + 0.00001);  
        publishROSTimer_ = nh_.createTimer(ros::Duration(duration), &HeightMapNode::publishMapROS, this);
    }
    
    if(publishLCMFps > 0.)
    {
        double duration  = 1. / (publishLCMFps + 0.00001);  
        publishLCMTimer_ = nh_.createTimer(ros::Duration(duration), &HeightMapNode::publishMapLCM, this);
    }

}

void HeightMapNode::pointcloudCallback(const sensor_msgs::PointCloud2& cloud, const std::string& key) 
{
    //  get channels
    auto fields             = cloud.fields;
    std::vector<std::string> channels;
    
    for (int it = 0; it < fields.size(); it++) 
    {
        auto& field         = fields[it];

        // field name : x, y, z, rgb
        channels.push_back(field.name);

    }

    inputPointCloud(cloud, channels);
}


void HeightMapNode::inputPointCloud(const sensor_msgs::PointCloud2& cloud, const std::vector<std::string>& channels) 
{
    auto start              = ros::Time::now();
    auto* pcl_pc            = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(pcl_pc);
    pcl_conversions::toPCL(cloud, *pcl_pc);

    //  get channels
    auto fields             = cloud.fields;
    uint array_dimension    = channels.size(); 
    
    RowMatrixXd points      = RowMatrixXd(pcl_pc->width * pcl_pc->height, array_dimension); 
    /* realsense d435 : 921600  
     * (640 * 360 * 4 = 230400 * 4 = 921600, array_dim number : 4) 
     */

    for (unsigned int i = 0; i < pcl_pc->width * pcl_pc->height; ++i) 
    {
        for (unsigned int j = 0; j < channels.size(); ++j) 
        {
            float temp;
            uint point_idx  = i * pcl_pc->point_step + pcl_pc->fields[j].offset;

            /*  pcl_pc->point_step : 32byte
             * pcl_pc->fields[j].offset
                intensity : 16
             * channels.size() : 4 (x, y, , intensity)
             */

            memcpy(&temp, &pcl_pc->data[point_idx], sizeof(float));
            points(i, j)     = static_cast<double>(temp);
        }
    }

    /* mapFrameId_ : map / sensorFrameId_ : Frame ID (ROS topic)
     */

    //  get pose of sensor in map frame
    tf::StampedTransform transformTf;
    std::string sensorFrameId_          = cloud.header.frame_id;
    auto timeStamp                      = cloud.header.stamp;
    Eigen::Affine3d transformationSensorToMap;    

    try 
    {
        transformListener_.waitForTransform(mapFrameId_, sensorFrameId_, timeStamp, ros::Duration(1.0));
        transformListener_.lookupTransform(mapFrameId_, sensorFrameId_, timeStamp, transformTf);
        poseTFToEigen(transformTf, transformationSensorToMap);
        
        /* 
         * waitForTransform args : ros::Duration(1.0) means timeout(seconds)
         * Eigen::Affine3d : Affine Transformation (translation, rotation, scaling)
         * Affine3d utility -> .matrix(), .translation(), .rotation()
         */
    } 
    catch (tf::TransformException& ex) 
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    double positionError{0.0};
    double orientationError{0.0};

    map_.input(points, channels, transformationSensorToMap.rotation(), transformationSensorToMap.translation(), positionError, orientationError);

    ROS_DEBUG_THROTTLE(1.0, "HeightMap processed a point cloud (%i points) in %f sec.", static_cast<int>(points.size()), (ros::Time::now() - start).toSec());
    ROS_DEBUG_THROTTLE(1.0, "positionError: %f ", positionError);
    ROS_DEBUG_THROTTLE(1.0, "orientationError: %f ", orientationError);
}

void HeightMapNode::updateMapFrame(const ros::TimerEvent&) 
{   
    // auto t_start    = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> retime = t_start - t_prev_frame;
    // t_prev_frame    = t_start;
    // std::cout << "Update Map Frame Time (Return) = " << retime.count() << " ms\n";
    auto new_base_position       = std::make_shared<HeightMapWrapper::RowMatrixXf>(1, 3);

    //  get pose of base in map frame
    tf::StampedTransform transformTf;
    const auto& timeStamp               = ros::Time::now();

    Eigen::Affine3d transformationBaseToMap;   
    
    try 
    {
        // Base Position From Map Frame
        transformListener_.waitForTransform(mapFrameId_, baseFrameId_, timeStamp, ros::Duration(1.0));
        transformListener_.lookupTransform(mapFrameId_, baseFrameId_, timeStamp, transformTf);
        poseTFToEigen(transformTf, transformationBaseToMap);                
    }
    catch (tf::TransformException& ex) 
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    map_.move_to(transformationBaseToMap.translation(), transformationBaseToMap.rotation());

    (*new_base_position)(0, 0) = transformationBaseToMap.translation()(0);
    (*new_base_position)(0, 1) = transformationBaseToMap.translation()(1);
    (*new_base_position)(0, 2) = transformationBaseToMap.translation()(2);

    {
    std::lock_guard<std::mutex> lock(mapMutex_);       

    base_pose_ptr = new_base_position;
    }
}

static inline bool isValid(const HeightMapWrapper::RowMatrixXf& M) 
{
  return M.rows() > 0 && M.cols() > 0;
}

void HeightMapNode::updateHeightMap(const ros::TimerEvent&) 
{
    // auto t_start    = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> retime = t_start - t_prev_map;
    // t_prev_map      = t_start;
    // std::cout << "Update Height Map Time (Return) = " << retime.count() << " ms\n";

    auto new_elevation          = std::make_shared<HeightMapWrapper::RowMatrixXf>(map_.map_n_, map_.map_n_);
    auto new_filter_map         = std::make_shared<HeightMapWrapper::RowMatrixXf>(map_.map_n_, map_.map_n_);
    auto new_feasibility        = std::make_shared<HeightMapWrapper::RowMatrixXf>(map_.map_n_, map_.map_n_);
    auto new_map_position       = std::make_shared<HeightMapWrapper::RowMatrixXf>(1, 3);

    map_.get_position(*new_map_position);
    map_.get_layer_data("elevation", *new_elevation);
    map_.get_layer_data("filter_map", *new_filter_map);
    map_.get_layer_data("feasibility", *new_feasibility);
    
    if (!isValid(*new_elevation) || !isValid(*new_feasibility)) 
    {
        return;
    }

    {
    std::lock_guard<std::mutex> lock(mapMutex_);       

    map_pose_ptr                = new_map_position;
    elevation_ptr               = new_elevation;
    filter_map_ptr              = new_filter_map;
    feasibility_ptr             = new_feasibility;

    // map_.get_position(base_position);
    // map_.get_layer_data("elevation", elevation);
    // map_.get_layer_data("feasibility", feasibility);

    // std::string layer;
    // for (const auto& layer : layers)
    // {
    //     map_.get_layer_data(layer, map_data);
    //     layer_data[layer] = map_data;
    // }
    
    // map_.get_layer_data("is_valid", valid);
    // map_.get_layer_data("filter_map", filter_map);
    // map_.get_layer_data("variance", variance);
    // map_.get_layer_data("esdf", esdf);
    }
}

void HeightMapNode::publishMapROS(const ros::TimerEvent&) 
{
    // auto t_start    = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> retime = t_start - t_prev_ros;
    // t_prev_ros      = t_start;
    // std::cout << " Update ROS Time (Return) = " << retime.count() << " ms\n";

    std::shared_ptr<HeightMapWrapper::RowMatrixXf> ros_map_pose_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> ros_elevation_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> ros_filter_map_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> ros_feasibility_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> ros_esdf_ptr;

    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        ros_map_pose_ptr            = map_pose_ptr;
        ros_elevation_ptr           = elevation_ptr;
        ros_filter_map_ptr          = filter_map_ptr;
        ros_feasibility_ptr         = feasibility_ptr;
    }


    if (ros_map_pose_ptr && ros_elevation_ptr && ros_feasibility_ptr) 
    {
        publishDataAsPoints(*ros_map_pose_ptr, "elevation", *ros_elevation_ptr, *ros_elevation_ptr, pointPub_);
        publishDataAsPoints(*ros_map_pose_ptr, "filter_map", *ros_filter_map_ptr, *ros_filter_map_ptr, filterPub_);
        publishDataAsPoints(*ros_map_pose_ptr, "feasibility", *ros_filter_map_ptr, *ros_feasibility_ptr, feasibilityPub_, 0.2f);
    }

    // publishDataAsPoints(pos, "elevation",  elev, elev, pointPub_);
    // publishDataAsPoints(pos, "feasibility", feas, feas, feasibilityPub_, 0.2f);

    // publishDataAsPoints(base_position, "filter", elevation, filter_map, filterPub_);
    
    // publishDataAsPoints(base_position, "traversability", elevation, traversability, traversabilityPub_);
    // publishDataAsPoints(base_position, "valid", elevation, valid, validPub_, 0.2);
    // publishNormalArrow(normal_x, normal_y, normal_z, elevation);
}

void HeightMapNode::publishMapLCM(const ros::TimerEvent&) 
{
    // auto t_start    = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> retime = t_start - t_prev_lcm;
    // t_prev_lcm      = t_start;
    // std::cout << "Elevation LCM Time (Return) = " << retime.count() << " ms\n";
    // static int time = 0;
    // time++;

    int rows = map_.map_n_;
    int cols = map_.map_n_;

    std::shared_ptr<HeightMapWrapper::RowMatrixXf> lcm_map_pose_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> lcm_base_pose_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> lcm_elevation_ptr;
    std::shared_ptr<HeightMapWrapper::RowMatrixXf> lcm_feasibility_ptr;

    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        // lcm_elevation_ptr           = elevation_ptr;
        lcm_elevation_ptr           = filter_map_ptr;
        lcm_feasibility_ptr         = feasibility_ptr;
        lcm_map_pose_ptr            = map_pose_ptr;
        lcm_base_pose_ptr           = base_pose_ptr;
    }

    if (lcm_elevation_ptr && lcm_feasibility_ptr && lcm_map_pose_ptr && lcm_base_pose_ptr) 
    {
        for(int i=0; i<rows; ++i)
        {
            for(int j=0; j<cols; ++j)
            {
                elevation_layer_data_lcm.map[i][j]      = static_cast<int>(1000.*(*lcm_elevation_ptr)(i, j)); 
                feasibility_layer_data_lcm.map[i][j]    = static_cast<int>((*lcm_feasibility_ptr)(i, j));
            }
        }
        
        for(int i = 0; i < 3; ++i)
        {
            elevation_layer_data_lcm.map_frame[i]            = (*lcm_map_pose_ptr)(i);
            elevation_layer_data_lcm.base_frame[i]           = (*lcm_base_pose_ptr)(i);
        }

        _visionLCM.publish("elevation", &elevation_layer_data_lcm);
        _visionLCM.publish("feasibility", &feasibility_layer_data_lcm);
    }

    // auto t_end    = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> turn2 = t_end - t_start;
    // std::cout << "Elevation Time (Consume) = " << turn2.count() << " ms\n";
}

void HeightMapNode::updateVariance(const ros::TimerEvent&) 
{
    map_.update_variance();
}
  
void HeightMapNode::updateTime(const ros::TimerEvent&) 
{
    map_.update_time();
}

void HeightMapNode::publishDataAsPoints(const HeightMapWrapper::RowMatrixXf& position, 
                                        const std::string name, 
                                        const HeightMapWrapper::RowMatrixXf& basic_layer, 
                                        const HeightMapWrapper::RowMatrixXf& data, 
                                        const ros::Publisher& pub,
                                        const float color_scale) 
{
    int rows = map_.map_n_;
    int cols = map_.map_n_;
    int size = rows * cols;

    pongbot_heightmap_msgs::HeightMap msg;
    msg.header.frame_id = mapFrameId_;
    // msg.header.frame_id = baseFrameId_;
    msg.header.stamp    = ros::Time::now();
    msg.layers.push_back(name);

    msg.resolution      = map_.resolution_;  
    
    // resize data 
    msg.data.resize(size);
    msg.color.resize(size);

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int i           = r * cols + c;

            float x         = getCellPosition(r, position(0), map_.resolution_, map_.map_length_);
            float y         = getCellPosition(c, position(1), map_.resolution_, map_.map_length_);

            float z         = basic_layer(r, c);
            float color     = data(r, c) * color_scale;

            if (!std::isnan(z))
            {
                msg.data[i].x       = x;
                msg.data[i].y       = y;
                msg.data[i].z       = z;
                msg.color[i]        = color;
            }
        }
    }

    pub.publish(msg);
}

void HeightMapNode::publishNormalArrow(const HeightMapWrapper::RowMatrixXf& normal_x,
                                       const HeightMapWrapper::RowMatrixXf& normal_y,
                                       const HeightMapWrapper::RowMatrixXf& normal_z,
                                       const HeightMapWrapper::RowMatrixXf& elevation)
{
    auto startTime = ros::Time::now();
    visualization_msgs::MarkerArray markerArray;

    int rows = map_.map_n_;
    int cols = map_.map_n_;
    float scale = 0.05; 

    HeightMapWrapper::RowMatrixXf position(1, 3);
    map_.get_position(position);

    int marker_id = 0;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            Eigen::Vector3f normal(normal_x(r, c), normal_y(r, c), normal_z(r, c));

            float x = getCellPosition(r, position(0), map_.resolution_, map_.map_length_);
            float y = getCellPosition(c, position(1), map_.resolution_, map_.map_length_);
            float z = elevation(r, c);

            Eigen::Vector3f start, end;
            start << x, y, z;

            geometry_msgs::Point p_start, p_end;
            p_start.x = start(0);
            p_start.y = start(1);
            p_start.z = start(2);

            end = start + normal * scale;
            p_end.x = end(0);
            p_end.y = end(1);
            p_end.z = end(2);

            if (normal.norm() < 1e-3)
            {
                continue;
            }
            marker_id++;
            markerArray.markers.push_back(vectorToArrowMarker(start, end, marker_id));
        }
    }

    normalPub_.publish(markerArray);
    // ROS_INFO_THROTTLE(1.0, "Published normal arrows in %f sec.", (ros::Time::now() - startTime).toSec());
}


visualization_msgs::Marker HeightMapNode::vectorToArrowMarker(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id      = mapFrameId_;
    // marker.header.frame_id      = baseFrameId_;
    marker.header.stamp         = ros::Time::now();
    marker.ns                   = "normals";
    marker.id                   = id;
    marker.type                 = visualization_msgs::Marker::ARROW;
    marker.action               = visualization_msgs::Marker::ADD;
    marker.points.resize(2);

    marker.points[0].x          = start(0);
    marker.points[0].y          = start(1);
    marker.points[0].z          = start(2);

    marker.points[1].x          = end(0);
    marker.points[1].y          = end(1);
    marker.points[1].z          = end(2);

    marker.pose.orientation.x   = 0.0;
    marker.pose.orientation.y   = 0.0;
    marker.pose.orientation.z   = 0.0;
    marker.pose.orientation.w   = 1.0;

    marker.scale.x              = 0.01;  // shaft diameter
    marker.scale.y              = 0.02;  // head diameter
    marker.scale.z              = 0.01;  // head length

    marker.color.r              = 1.0;
    marker.color.g              = 1.0;
    marker.color.b              = 0.0;
    marker.color.a              = 1.0;

    return marker;
    
}

float HeightMapNode::getCellPosition(const int idx, const float center, const float resolution, const float map_length) 
{
    float data     = center + 0.5 * map_length + (- idx - 0.5) * resolution;
    return data;
}

}  // namespace pongbot_heightmap_cupy
