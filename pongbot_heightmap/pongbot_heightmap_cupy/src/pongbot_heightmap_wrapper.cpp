// /**
//  * @author SungJoon Yoon  (densee250@gmail.com)
//  * @version 1.0
//  * @date 2025-03-17
//  * @copyright Copyright (c) 2025, Robotics & Control Lab.
//  *
//  */

#include "pongbot_heightmap_cupy/pongbot_heightmap_wrapper.hpp"

#include <utility>

// ROS
#include <ros/package.h>

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

HeightMapWrapper::HeightMapWrapper() {}

void HeightMapWrapper::initialize(ros::NodeHandle& nh)
{
    // Add the pongbot_heightmap path to sys.path
    auto threading                  = py::module::import("threading");
    py::gil_scoped_acquire acquire;

    auto sys                        = py::module::import("sys");
    auto path                       = sys.attr("path");

    std::string module_path = ros::package::getPath("pongbot_heightmap_cupy");
    module_path                     = module_path + "/script";
    path.attr("insert")(0, module_path);

    auto pongbot_heightmap          = py::module::import("heightmap_package_cupy.pongbot_heightmap");
    auto parameter                  = py::module::import("heightmap_package_cupy.parameter");

    map_param_                      = parameter.attr("Parameter")();
    setParameters(nh);
    map_                            = pongbot_heightmap.attr("HeightMap")(map_param_);

}


void HeightMapWrapper::setParameters(ros::NodeHandle& nh) 
{
    py::list paramNames             = map_param_.attr("get_names")();
    py::list paramTypes             = map_param_.attr("get_types")();
    py::gil_scoped_acquire acquire;

    // Try to find the parameter in the ros parameter server.
    for (int i = 0; i < paramNames.size(); i++) 
    {
        std::string name            = py::cast<std::string>(paramNames[i]);
        std::string type            = py::cast<std::string>(paramTypes[i]);

        // std::cout << " name : " << name << std::endl;
        // std::cout << " type : " << type << std::endl;

        // if (type == "float") 
        // {
        //     float param;
        //     if (nh.getParam(name, param)) 
        //     {
        //         map_param_.attr("set_value")(name, param);
        //     }
        // } 
        if (type == "float") 
        {
            double param;
            if (getParamPrivateOrGlobal(nh, name, param)) 
            {
                map_param_.attr("set_value")(name, param);
            }
        }
        // else if (type == "str") 
        // {
        //     std::string param;
        //     if (nh.getParam(name, param)) 
        //     {
        //         map_param_.attr("set_value")(name, param);
        //     }
        // } 
        // else if (type == "bool") 
        // {
        //     bool param;
        //     if (nh.getParam(name, param)) 
        //     {
        //         map_param_.attr("set_value")(name, param);
        //     }
        // } 
        // else if (type == "int") 
        // {
        //     int param;
        //     if (nh.getParam(name, param)) 
        //     {
        //         map_param_.attr("set_value")(name, param);
        //     }
        // }
        else if (type == "str") 
        {
            std::string param;
            if (getParamPrivateOrGlobal(nh, name, param)) 
            {
                map_param_.attr("set_value")(name, param);
            }
        } 
        else if (type == "bool") 
        {
            bool param;
            if (getParamPrivateOrGlobal(nh, name, param)) 
            {
                map_param_.attr("set_value")(name, param);
            }
        } 
        else if (type == "int") 
        {
            int param;
            if (getParamPrivateOrGlobal(nh, name, param)) 
            {
                map_param_.attr("set_value")(name, param);
            }
        }
    }

    // XmlRpc::XmlRpcValue subscribers;
    // nh.getParam("/subscribers", subscribers);
    XmlRpc::XmlRpcValue subscribers;
    if (!getParamPrivateOrGlobal(nh, "subscribers", subscribers)) {
        ROS_FATAL("[HeightMapWrapper] There are no subscribers set in parameters.");
        return;
    }


    py::dict subscriber_dict;
    for (auto& subscriber : subscribers) 
    {
        const char* name                = subscriber.first.c_str();
        const auto& sub_names           = subscriber.second;

        if (!subscriber_dict.contains(name)) 
        {
            subscriber_dict[name]       = py::dict();
        }

        for (auto& sub_mame : sub_names) 
        {
            const char* key             = sub_mame.first.c_str();
            const auto& value           = sub_mame.second;

            std::vector<std::string> array;
            switch(value.getType())
            {
                case XmlRpc::XmlRpcValue::TypeString:
                    subscriber_dict[name][key] = static_cast<std::string>(value);
                    break;
                case XmlRpc::XmlRpcValue::TypeInt:
                    subscriber_dict[name][key] = static_cast<int>(value);
                    break;
                case XmlRpc::XmlRpcValue::TypeDouble:
                    subscriber_dict[name][key] = static_cast<double>(value);
                    break;
                case XmlRpc::XmlRpcValue::TypeBoolean:
                    subscriber_dict[name][key] = static_cast<bool>(value);
                    break;
                case XmlRpc::XmlRpcValue::TypeArray:
                    
                    for (int32_t i = 0; i < value.size(); ++i)
                    {
                        auto element = static_cast<std::string>(value[i]);
                        array.push_back(element);
                    }
                    subscriber_dict[name][key] = array;
                    break;
                case XmlRpc::XmlRpcValue::TypeStruct:
                    break;
                default:
                    subscriber_dict[name][key] = py::cast(value);
                    break;
            }                
        }        
    }

    map_param_.attr("subscriber_config") = subscriber_dict;

    // Update Cell size, Map Length
    map_param_.attr("update")();

    resolution_ = py::cast<float>(map_param_.attr("get_value")("resolution"));
    map_length_ = py::cast<float>(map_param_.attr("get_value")("true_map_length"));
    map_n_      = py::cast<int>(map_param_.attr("get_value")("true_cell_n"));
}

void HeightMapWrapper::input(const RowMatrixXd& points, const std::vector<std::string>& channels, const RowMatrixXd& R, const Eigen::VectorXd& t, const double positionNoise, const double orientationNoise) 
{
    py::gil_scoped_acquire acquire;

    map_.attr("input_pointcloud")(Eigen::Ref<const RowMatrixXd>(points), channels, Eigen::Ref<const RowMatrixXd>(R), Eigen::Ref<const Eigen::VectorXd>(t), positionNoise, orientationNoise);
}

void HeightMapWrapper::move_to(const Eigen::VectorXd& t, const RowMatrixXd& R) 
{
    py::gil_scoped_acquire acquire;
    map_.attr("move_to")(Eigen::Ref<const Eigen::VectorXd>(t), Eigen::Ref<const RowMatrixXd>(R));
}

void HeightMapWrapper::clear() 
{
    py::gil_scoped_acquire acquire;
    map_.attr("clear")();
}

bool HeightMapWrapper::exists_layer(const std::string& layerName) 
{
    py::gil_scoped_acquire acquire;
    return py::cast<bool>(map_.attr("exists_layer")(layerName));
}

void HeightMapWrapper::get_position(RowMatrixXf& position) 
{
    py::gil_scoped_acquire acquire;
    map_.attr("get_position")(Eigen::Ref<RowMatrixXf>(position));
}

void HeightMapWrapper::get_layer_data(const std::string& layerName, RowMatrixXf& map) 
{
    py::gil_scoped_acquire acquire;
    map = RowMatrixXf(map_n_, map_n_);
    map_.attr("get_map_with_name")(layerName, Eigen::Ref<RowMatrixXf>(map));
}

void HeightMapWrapper::update_variance() 
{
    py::gil_scoped_acquire acquire;
    map_.attr("update_variance")();
}
  
void HeightMapWrapper::update_time() 
{
    py::gil_scoped_acquire acquire;
    map_.attr("update_time")();
}

// void HeightMapWrapper::get_grid_map(grid_map::GridMap& gridMap, const std::vector<std::string>& requestLayerNames) {
//     std::vector<std::string> basicLayerNames;
//     std::vector<std::string> layerNames = requestLayerNames;
//     std::vector<int> selection;
//     for (const auto& layerName : layerNames) {
//       if (layerName == "elevation") {
//         basicLayerNames.push_back("elevation");
//       }
//     }
  
//     RowMatrixXd pos(1, 3);
//     py::gil_scoped_acquire acquire;
//     map_.attr("get_position")(Eigen::Ref<RowMatrixXd>(pos));
//     grid_map::Position position(pos(0, 0), pos(0, 1));
//     grid_map::Length length(map_length_, map_length_);
//     gridMap.setGeometry(length, resolution_, position);
//     std::vector<Eigen::MatrixXf> maps;
  
//     for (const auto& layerName : layerNames) {
//       bool exists = map_.attr("exists_layer")(layerName).cast<bool>();
//       if (exists) {
//         RowMatrixXf map(map_n_, map_n_);
//         map_.attr("get_map_with_name_ref")(layerName, Eigen::Ref<RowMatrixXf>(map));
//         gridMap.add(layerName, map);
//       }
//     }
//     if (enable_normal_color_) {
//       RowMatrixXf normal_x(map_n_, map_n_);
//       RowMatrixXf normal_y(map_n_, map_n_);
//       RowMatrixXf normal_z(map_n_, map_n_);
//       map_.attr("get_normal_ref")(Eigen::Ref<RowMatrixXf>(normal_x), Eigen::Ref<RowMatrixXf>(normal_y), Eigen::Ref<RowMatrixXf>(normal_z));
//       gridMap.add("normal_x", normal_x);
//       gridMap.add("normal_y", normal_y);
//       gridMap.add("normal_z", normal_z);
//     }
//     gridMap.setBasicLayers(basicLayerNames);
//     if (enable_normal_color_) {
//       addNormalColorLayer(gridMap);
//     }
//   }


}  // namespace pongbot_heightmap_cupy