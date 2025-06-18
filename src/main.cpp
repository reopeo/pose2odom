#include "../include/pose2odom/pose2odom.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pose2Odom>());
    rclcpp::shutdown();
    return 0;
}
