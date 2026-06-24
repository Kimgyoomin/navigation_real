#pragma once
#pragma GCC visibility push(default)

// STL
#include <iostream>

// Eigen
#include <Eigen/Dense>

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding
#include <pybind11/stl.h>
#include <pybind11/eigen.h>  // convert Eigen to Python Data

// ROS
#include <ros/ros.h>

namespace py = pybind11;

namespace pongbot_heightmap_cupy {

class HeightMapWrapper
{
    public:        
        using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using RowMatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        
        HeightMapWrapper();

        void initialize(ros::NodeHandle& nh);
        void input(const RowMatrixXd& points, const std::vector<std::string>& channels, const RowMatrixXd& R, const Eigen::VectorXd& t, const double positionNoise, const double orientationNoise);
        void move_to(const Eigen::VectorXd& t, const RowMatrixXd& R);

        void clear();
        bool exists_layer(const std::string& layerName);
        void get_layer_data(const std::string& layerName, RowMatrixXf& map);
        void get_position(RowMatrixXf& position);
        void update_variance();
        void update_time();

        double resolution_;
        double map_length_;
        int map_n_;
    private:
        void setParameters(ros::NodeHandle& nh);
        
        py::object map_;
        py::object map_param_;
};

}  // namespace pongbot_heightmap_cupy

#pragma GCC visibility pop
