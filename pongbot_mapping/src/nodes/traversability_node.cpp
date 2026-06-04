#include <ros/ros.h>
// #include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <filters/filter_chain.hpp>
#include <string>

// Class definition
class TraversabilityNode {
public:
    TraversabilityNode()
    : chain_("grid_map::GridMap")               // * FilterChain Template 
    {
        ros::NodeHandle nh, pnh("~");

        // 1) Load Parameters
        in_topic_       = pnh.param<std::string>("input_topic",     "/elevation_mapping/elevation_map");
        out_topic_      = pnh.param<std::string>("output_topic",    "/traversability_map");

        // YAML is from at launch, rosparam load
        if (!chain_.configure("traversability_map_filters", pnh)) {
            ROS_WARN("FilterChain configure failed. Check YAML key 'traversability map filters'");
        }

        // 2) I/O topic
        sub_ = nh.subscribe(in_topic_, 1, &TraversabilityNode::cb, this);
        pub_ = nh.advertise<grid_map_msgs::GridMap>(out_topic_, 1, true);

    }

    // Callback Function (THE MOST IMPORTANT PART)
    void cb(const grid_map_msgs::GridMap& msg)
    {
        grid_map::GridMap in;
        grid_map::GridMapRosConverter::fromMessage(msg, in);        // ROS->GridMap (official converter) : content Reference [oaicite:4] {index=4}
        grid_map::GridMap out;
        if (!chain_.update(in, out)) {                              // FilterChain process (API)
            ROS_WARN_THROTTLE(1.0, "FilterChain update failed");
            return;
        }
        grid_map_msgs::GridMap out_msg;
        grid_map::GridMapRosConverter::toMessage(out, out_msg); // GridMap -> ROS (official converter) :contentReference[oaicite:6]{index=6}
        pub_.publish(out_msg);
    }

private:
    std::string in_topic_, out_topic_;
    filters::FilterChain<grid_map::GridMap> chain_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};



// main : Almost done
int main(int argc, char** argv)
{
    ros::init(argc, argv, "traversability_node");
    TraversabilityNode n;
    ros::spin();
    return 0;
}