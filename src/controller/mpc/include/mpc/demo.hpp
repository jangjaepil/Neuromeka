#include "MPC.hpp"
#include "RequiredHeaders.hpp"

class demo : public MPC ,public rclcpp::Node  
{
public:
    demo();
    void run();
    void commands_callback(const std_msgs::msg::Float64MultiArray & msg);
private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr states_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ctr_pub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commands_sub;
};