#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/cmd_vel_t.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

class CmdVelBridgeTx
{
public:
    CmdVelBridgeTx()
    : nh_("~"),
      lcm_(getLcmUrl()),
      seq_(0)
    {
        if (!lcm_.good()) {
            throw std::runtime_error("LCM init failed (check URL / iface / permissions)");
        }

        // ROS params
        nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        nh_.param<std::string>("lcm_channel",   lcm_channel_,   "CMD_VEL");

        // default 값은 문자열이 아니라 double 리터럴이어야 함
        nh_.param("max_linear_x",  max_linear_x_,  0.5);
        nh_.param("max_linear_y",  max_linear_y_,  0.2);
        nh_.param("max_angular_z", max_angular_z_, 0.2);

        sub_ = nh_.subscribe(cmd_vel_topic_, 10, &CmdVelBridgeTx::cb, this);

        ROS_INFO_STREAM("[ros_cmdvel_to_lcm] subscribe: " << cmd_vel_topic_
                        << " -> LCM channel: " << lcm_channel_
                        << " url=" << lcm_url_);
    }

private:
    std::string getLcmUrl()
    {
        // 단일 PC 검증 기본값 : loopback
        nh_.param<std::string>("lcm_url", lcm_url_,
                               "udpm://239.255.76.67:7667?ttl=1");
        return lcm_url_;
    }

    static inline double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(v, hi));
    }

    void cb(const geometry_msgs::TwistConstPtr& msg)
    {
        // (안전) saturation
        // const double lx = clamp(msg->linear.x * 8.,  -max_linear_x_,  max_linear_x_);
        // const double ly = clamp(msg->linear.y * 3.,  -max_linear_y_,  max_linear_y_);
        // const double az = clamp(msg->angular.z * 5., -max_angular_z_, max_angular_z_);
        const double lx = clamp(msg->linear.x,  -max_linear_x_,  max_linear_x_);
        const double ly = clamp(msg->linear.y,  -max_linear_y_,  max_linear_y_);
        const double az = clamp(msg->angular.z, -max_angular_z_, max_angular_z_);

        robotlcm::cmd_vel_t out{};
        out.seq       = seq_++;
        out.utime     = static_cast<int64_t>(ros::Time::now().toNSec() / 1000); // usec
        out.linear_x  = static_cast<float>(lx);
        out.linear_y  = static_cast<float>(ly);
        out.angular_z = static_cast<float>(az);

        lcm_.publish(lcm_channel_, &out);

        // 로그 과다 방지
        if ((out.seq % 50) == 0) {
            ROS_INFO_STREAM("[ros_cmdvel_to_lcm] seq=" << out.seq
                            << " lx=" << out.linear_x
                            << " ly=" << out.linear_y
                            << " az=" << out.angular_z);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    std::string lcm_url_;
    lcm::LCM lcm_;
    
    std::string lcm_channel_;
    std::string cmd_vel_topic_;
    int32_t seq_;

    double max_linear_x_, max_linear_y_, max_angular_z_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_cmdvel_to_lcm");   // 노드명도 통일(대소문자 섞지 마)
    try {
        CmdVelBridgeTx node;
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("[ros_cmdvel_to_lcm] fatal: " << e.what());
        return 1;
    }
    return 0;
}
