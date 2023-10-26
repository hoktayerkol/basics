#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace RandomDrive_NS
{
	class RandomDrive : public rclcpp::Node
	{
		private:
			rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
			rclcpp::TimerBase::SharedPtr timer_;
			geometry_msgs::msg::Twist twist_msg_;

			rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
			sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

			void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
			void control_callback(void);
			bool is_there_obstacle();
			void turn_right();
			void go_forward();
			void stop();

			enum State{FORWARD, RIGHT, STOP};
			State current_state_=STOP;
			State last_state_=STOP;

			const float LINEAR_SPEED = 0.3f;
			const float ANGULAR_SPEED = 0.3f;
		public:
			RandomDrive();
	};

} // end of RandomDrive_NS


namespace RandomDrive_NS
{
	RandomDrive::RandomDrive() : Node("rondom_move")
	{
		pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		timer_ = create_wall_timer(0.02s, std::bind(&RandomDrive::control_callback, this));

		sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan",
						rclcpp::SensorDataQoS(), std::bind(&RandomDrive::scan_callback, this, _1) );
	}

	void RandomDrive::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
	{
		last_scan_ = std::move(msg);
	}

	void RandomDrive::control_callback(void)
	{
		if (last_scan_==nullptr)
			return;

		if (is_there_obstacle())
			turn_right();
		else
			go_forward();


	}

	bool RandomDrive::is_there_obstacle()
	{
			transform(last_scan_->ranges.begin(), last_scan_->ranges.end(), last_scan_->ranges.begin(),
					[](float a){
						static constexpr float inc = 0.01749303564429283f;
						float i=-inc;
						i+=inc;
						return a*cos(i);
					});

			size_t ln = last_scan_->ranges.size();
			std::vector<bool> obstacles(ln/4);
			transform(last_scan_->ranges.begin(), last_scan_->ranges.begin()+45, obstacles.begin(), [](float a){return a<0.4f;});
			transform(last_scan_->ranges.rbegin(), last_scan_->ranges.rbegin()+45, obstacles.rbegin(), [](float a){return a<0.4f;});

			if ( std::find(obstacles.begin(), obstacles.end(), 1)==obstacles.end() )
				return 0;
			else
				return 1;
	}

	void RandomDrive::turn_right()
	{
		twist_msg_.linear.x=0.0;
		twist_msg_.angular.z=-ANGULAR_SPEED;
		pub_->publish(twist_msg_);

		if (current_state_ != RIGHT)
		{
			last_state_ = current_state_;
			current_state_ = RIGHT;
			RCLCPP_INFO(get_logger(), "turning right!");
		}
	}

	void RandomDrive::go_forward()
	{
		twist_msg_.linear.x=LINEAR_SPEED;
		twist_msg_.angular.z=0.0;
		pub_->publish(twist_msg_);

		if (current_state_ != FORWARD)
		{
			last_state_ = current_state_;
			current_state_ = FORWARD;
			RCLCPP_INFO(get_logger(), "going forward!");
		}
	}

	void RandomDrive::stop()
	{
		twist_msg_.linear.x = 0.0;
		twist_msg_.angular.x = 0.0;
		pub_->publish(twist_msg_);

		if (current_state_ != STOP)
		{
			last_state_ = current_state_;
			current_state_ = STOP;
			RCLCPP_INFO(get_logger(), "stopped!");
		}
	}

}// end of RandomDrive_NS

int main(int argc, char *argv[])
{

	rclcpp::init(argc, argv);

	auto node = std::make_shared<RandomDrive_NS::RandomDrive>();

	rclcpp::spin(node);

	rclcpp::shutdown();



	return 0;
}
