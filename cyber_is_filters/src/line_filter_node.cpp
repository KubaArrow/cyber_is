#include <rclcpp/rclcpp.hpp>
#include "cyber_is_filters/line_filter.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
