#include "demo.hpp"
#include "RequiredHeaders.hpp"

using namespace std::chrono_literals;
demo::demo() :Node("MPC")
{
    states_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/states", 1000);
    ctr_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ctr", 1000);
    commands_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/commands", 1000, std::bind(&demo::commands_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(1ms, std::bind(&demo::timer_callback, this));

}
 
void demo::timer_callback()
{

  auto states = std_msgs::msg::Float64MultiArray();
  auto ctr = std_msgs::msg::Float64MultiArray();

  Eigen::Matrix<double, 12, 1> x0 = getStates();
  Eigen::Vector4d ctrV = getCtr();
  states.data = {x0(0),x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11)};
  ctr.data = {ctrV(0),ctrV(1),ctrV(2),ctrV(3)};
  states_pub->publish(states);
  ctr_pub->publish(ctr);
}

void demo::commands_callback(const std_msgs::msg::Float64MultiArray & msg) 
{
    Eigen::Matrix<double, 12, 1> dx;
    dx(0)  = msg.data[0];
    dx(1)  = msg.data[1];
    dx(2)  = msg.data[2];
    dx(3)  = msg.data[3];
    dx(4)  = msg.data[4];
    dx(5)  = msg.data[5];
    dx(6)  = msg.data[6];
    dx(7)  = msg.data[7];
    dx(8)  = msg.data[8];
    dx(9)  = msg.data[9];
    dx(10)  = msg.data[10];
    dx(11)  = msg.data[11];
    
    setRef(dx);
}

void demo::run()
{
    rclcpp::Rate loop_rate(1000);
    while(rclcpp::ok())
    {
     solveProblem();
     loop_rate.sleep();   
    }

}



