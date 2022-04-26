#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "message_package/msg/states.hpp"
#include "message_package/msg/input.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Do not call this drone because now you could easily give information of another thing and it would still work

class Update_states : public rclcpp::Node
{
public:

  float x1;
  float x2;
  float x3;
  float x4;
  float x5;
  int i = 0;

  float thrust;
  float orient; 
  int k = 0;

  Update_states()
  : Node("States"), count_(0)
  {
   // First make constructer to create publisher
    subscription_=this->create_subscription<message_package::msg::States>("States_x",10,std::bind(&Update_states::subscribe_state_callback,this,_1));
    input_subscription_= this->create_subscription<message_package::msg::Input>(          
      "Inputs_u", 10, std::bind(&Update_states::input_callback, this, _1));
    publisher_ = this->create_publisher<message_package::msg::States>("States_x", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Update_states::publish_state_callback,this));
    
  }


  void Update_function(float x, float y, float theta, float xdot, float ydot){
    // x1 = x[0];
    x1 = x;
    x2 = y;
    x3 = theta;
    x4 = xdot;
    x5 = ydot;
    //x_vec = [x1 x2 x3 x4 x5];
  }

  void subscribe_state_callback(const message_package::msg::States::SharedPtr msg)
  {
    std::vector<double> values = msg->x;
    RCLCPP_INFO(this->get_logger(), "State that is subscribed: %f %f %f %f %f", values[i],values[i+1],values[i+2],values[i+3],values[i+4]);  

    Update_function(values[i],values[i+1],values[i+2],values[i+3],values[i+4]);

    i = i+5;
  }

  void update_inputs(float T, float w){
    thrust = T;
    orient = w;
}

void input_callback(const message_package::msg::Input::SharedPtr msgu)
{
    std::vector<double> values = msgu->u; // does not take float, is a double

      RCLCPP_INFO(this->get_logger(), "Input subscribed: %f %f", values[k], values[k+1]);

      update_inputs(values[k], values[k+1]);
    
    k=k+2;
}


  void publish_state_callback()
  {
    // publish in this callback.
    auto message = message_package::msg::States();                              
    message.x = {1.0,2.0,3.0,4.0,5.0};  
    //message.x[0]=3.14; change manually                                      
    RCLCPP_INFO(this->get_logger(), "Publish states");//, message.states);    
    publisher_->publish(message);
  }


private:
   
  rclcpp::Subscription<message_package::msg::Input>::SharedPtr input_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;      
  rclcpp::Publisher<message_package::msg::States>::SharedPtr publisher_; 
  rclcpp::Subscription<message_package::msg::States>::SharedPtr subscription_; 
  size_t count_;   
   
 
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Update_states>());

  rclcpp::shutdown();
  return 0;
}
