#include "../pose2odom.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose2odom>());
    rclcpp::shutdown();
    return 0;
}
