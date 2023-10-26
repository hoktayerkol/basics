#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
	
	rclcpp::init(argc, argv);
	
	auto node = rclcpp::Node::make_shared("logger");
	
	rclcpp::Rate loop_rate(500ms);
	
	
	while(rclcpp::ok())
	{
		RCLCPP_INFO(node->get_logger(), "hey bro!");
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	
	rclcpp::shutdown();
	
	return 0;
}
