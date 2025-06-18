#include "../include/pose2odom/pose2odom.hpp"

Pose2Odom::Pose2Odom(const std::string &name_space, const rclcpp::NodeOptions &options) : Node("pose2odom", name_space, options)
{
    this->declare_parameter<std::string>("input_topic", "/pose_with_covariance");
    this->declare_parameter<std::string>("output_topic", "/odom");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("child_frame_id", "base_link");

    _input_topic = this->get_parameter("input_topic").as_string();
    _output_topic = this->get_parameter("output_topic").as_string();
    _odom_frame_id = this->get_parameter("odom_frame_id").as_string();
    _child_frame_id = this->get_parameter("child_frame_id").as_string();

    _initialized = false;

    _pub = this->create_publisher<nav_msgs::msg::Odometry>(_output_topic, 10);
    _sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        _input_topic, 
        10,
        std::bind(&Pose2Odom::pose_callback, this, std::placeholders::_1));
}

void Pose2Odom::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = _odom_frame_id;
    odom.child_frame_id = _child_frame_id;

    odom.pose.pose = msg->pose.pose;
    odom.pose.covariance = msg->pose.covariance;

    if (!_initialized)
    {
      zero_twist(odom);
      _last_msg = *msg;
      _last_time = msg->header.stamp;
      _initialized = true;
    }
    else
    {
      rclcpp::Time now = msg->header.stamp;
      double dt = (now - _last_time).seconds();
      if (dt <= 0.0) dt = 1e-6;

      double dx = msg->pose.pose.position.x  - _last_msg.pose.pose.position.x;
      double dy = msg->pose.pose.position.y  - _last_msg.pose.pose.position.y;

      double yaw_prev = get_yaw(_last_msg.pose.pose.orientation);
      double yaw_curr = get_yaw(msg->pose.pose.orientation);
      double dyaw = shortest_angular_distance(yaw_prev, yaw_curr);

      odom.twist.twist.linear.x  = dx / dt;
      odom.twist.twist.linear.y  = dy / dt;
      odom.twist.twist.angular.z = dyaw / dt;
      odom.twist.twist.linear.z  = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;

      for (auto &c : odom.twist.covariance) c = 0.0;

      _last_msg = *msg;
      _last_time = now;
    }

    _pub->publish(odom);
}

void Pose2Odom::zero_twist(nav_msgs::msg::Odometry &odom)
{
    odom.twist.twist.linear.x = odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = odom.twist.twist.angular.y = odom.twist.twist.angular.z = 0.0;
    for (auto &c : odom.twist.covariance) c = 0.0;
}

double Pose2Odom::get_yaw(const geometry_msgs::msg::Quaternion &q_msg)
{
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double Pose2Odom::shortest_angular_distance(double from, double to)
{
    double diff = to - from;
    while (diff >  M_PI) diff -= 2.0*M_PI;
    while (diff < -M_PI) diff += 2.0*M_PI;
    return diff;
}
