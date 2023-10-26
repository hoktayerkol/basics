#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SubPubNode : public rclcpp::Node
{
	private:
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
		std_msgs::msg::Int32 msg_;
		rclcpp::TimerBase::SharedPtr timer_;
		
	public:
	SubPubNode(): Node("subpupNode")
	{
		pub_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10);
		timer_ = create_wall_timer(500ms, std::bind(&SubPubNode::timer_callback, this));
		
		sub_ = create_subscription<std_msgs::msg::Int32>("int_topic", 10, std::bind(&SubPubNode::callback, this, _1));
	}
	
	void timer_callback()
	{
		msg_.data+=1;
		pub_->publish(msg_);
	}
	
	void callback(const std_msgs::msg::Int32::SharedPtr msgpub)
	{
	
		RCLCPP_INFO(get_logger(), "hello bro! %d", msgpub->data);
		
	}
};

int main(int argc, char *argv[])
{
	
	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<SubPubNode>();
	
	rclcpp::spin(node);
	
	rclcpp::shutdown();
	
	return 0;
}
