#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class LoggerNode : public rclcpp::Node
{
	private:
		rclcpp::TimerBase::SharedPtr timer_;
		
	
	
	public:
		LoggerNode() : Node("logger_node_class")
		{
			timer_ = create_wall_timer(500ms, std::bind(&LoggerNode::timer_callback, this));
		}
	
		void timer_callback()
		{
			RCLCPP_INFO(get_logger(), "hello logger class");
		}


};


int main(int argc, char *argv[])
{
	
	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<LoggerNode>();
	
	rclcpp::spin(node);
	
	rclcpp::shutdown();
	
	return 0;
}
