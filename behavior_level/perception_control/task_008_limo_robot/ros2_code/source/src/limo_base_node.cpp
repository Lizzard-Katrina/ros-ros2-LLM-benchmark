#include "limo_driver.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgileX::LimoDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}