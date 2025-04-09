#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "misora2_operation/operator_gui_component.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<component_operator_gui::DistributeImage>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
}