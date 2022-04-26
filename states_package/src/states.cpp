#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "message_package/msg/states.hpp"
#include "message_package/msg/input.hpp"
#include "Dynamics_package/src/NIM.h"
#include "Dynamics_package//src/ODEs.h"



using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

// Do not call this drone because now you could easily give information of another thing and it would still work

class Dynamics : public rclcpp::Node
{
public:

  NIM method;
  vector<double> x;
  int i = 0;

  float thrust;
  float orient; 
  vector<double> u;
  int k = 0;

  Dynamics()
  : Node("States"), count_(0)
  {
   // First make constructer to create publisher
    subscription_=this->create_subscription<message_package::msg::States>("States_x",10,std::bind(&Dynamics::subscribe_state_callback,this,_1));
    input_subscription_= this->create_subscription<message_package::msg::Input>(          
      "Inputs_u", 10, std::bind(&Dynamics::input_callback, this, _1));
    publisher_ = this->create_publisher<message_package::msg::States>("States_x", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Dynamics::publish_state_callback,this));

    x = method.getInitialState();
    //TODO initialize u.
  }

  void subscribe_state_callback(const message_package::msg::States::SharedPtr msg)
  {
    std::vector<double> values = msg->x;

 
    RCLCPP_INFO(this->get_logger(), "State that is subscribed:");// %d", values.size()); 

      
  }

  void update_inputs(double T, double w){
    u = {T, w};
  }

void input_callback(const message_package::msg::Input::SharedPtr msgu)
{
    std::vector<double> values = msgu->u; // does not take float, is a double
    std::cout<<"after input func";
      RCLCPP_INFO(this->get_logger(), "Input subscribed: %f %f", values[k], values[k+1]);
      


      update_inputs(values[k], values[k+1]);
  
      k=k+2;


}



void publish_state_callback()
{
    // publish in this callback.
    auto message = message_package::msg::States();                              
    
    //message.x = random_func();
    //message.x[0]=3.14;

    //x = method.update(x, u);
    message.x = method.update(x, u);

    //run_dynamics();
                                         
    RCLCPP_INFO(this->get_logger(), "Publish states: %f",message.x[0]);//, message.states);    
    publisher_->publish(message);


}


// void run_dynamics()
// {
//   std::cout<<"This works";
// }
// void run_dynamics(vector<double> x, const vector<double> u){
//     x[1] = 51;
//     x[2] = 6;
    
// }


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
  rclcpp::spin(std::make_shared<Dynamics>()); // returns callbacks until shutdown.

  rclcpp::shutdown();
  return 0;
}
