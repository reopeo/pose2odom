#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Pose2Odom : public rclcpp::Node
{
public:
    Pose2Odom(const rclcpp::NodeOptions &options) : Pose2Odom("", options) {}
    Pose2Odom(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void zero_twist(nav_msgs::msg::Odometry &odom);
  double get_yaw(const geometry_msgs::msg::Quaternion &q_msg);
  double shortest_angular_distance(double from, double to);

  bool _initialized;
  geometry_msgs::msg::PoseWithCovarianceStamped _last_msg;
  rclcpp::Time _last_time;

  std::string _input_topic, _output_topic;
  std::string _odom_frame_id, _child_frame_id;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub;
};
