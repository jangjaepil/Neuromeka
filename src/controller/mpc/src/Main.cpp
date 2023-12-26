
#include "demo.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto MPC_node = std::make_shared<demo>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(MPC_node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    MPC_node->init();
    MPC_node->run();

    executor_thread.join();

    rclcpp::shutdown();
    
    return 0;
}