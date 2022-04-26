#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "message_package/msg/num.hpp"    

using namespace std::chrono_literals;

class InputPublisher : public rclcpp::Node
{
public:
    InputPublisher()
  : Node("Input"), count_(0)
  {
    publisher_ = this->create_publisher<message_package::msg::Num>("topic", 10);   
    timer_ = this->create_wall_timer(
      500ms, std::bind(&InputPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = message_package::msg::Num();                               
    message.num = this->count_++;                                        
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<message_package::msg::Num>::SharedPtr publisher_;        
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputPublisher>());
  rclcpp::shutdown();
  return 0;
}