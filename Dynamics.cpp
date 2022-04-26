#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "message_package/msg/states.hpp"
#include "message_package/msg/input.hpp"
#include "NIM.h"
#include "ODEs.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

// The dynamics node gets inputs from the input_node



class Dynamics : public rclcpp::Node
{
public:

  
  vector<double> x_vec; // moet dit
  int i = 0;

  double thrust;
  double orient; 
  vector<double> u;
  int k = 0;

  Dynamics()
  : Node("States"), count_(0)
  {
   // First make constructer to create publisher and subscribers
    subscription_=this->create_subscription<message_package::msg::States>("States_x",10,std::bind(&Dynamics::subscribe_state_callback,this,_1));
    input_subscription_= this->create_subscription<message_package::msg::Input>(          
      "Inputs_u", 10, std::bind(&Dynamics::input_callback, this, _1));
    publisher_ = this->create_publisher<message_package::msg::States>("States_x", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Dynamics::publish_state_callback,this));

    //Initialize u.
  }

  void ask_nim()
{
    
    bool nim_method; // 1 is Forward Euler, 0 is Runge Kutta
    std::cout << "\nPlease enter enter either 0 (for Runge Kutta) or 1 (for Forward Euler)";
    std::cin >> nim_method;

    if (nim_method) {
        // This still requires a subscription to the STATE X and the INPUT U
        state = new f_euler;
        x_vec = state->getInitialState();
        std::cout << "\nForward Euler was selected\n";
    }
    else {
        // This still requires a subscription to the STATE X and the INPUT U
        state = new runge_kutta;
        x_vec = state->getInitialState();
        std::cout << "\nRunge Kutta was selected\n";

    }
}

  void subscribe_state_callback(const message_package::msg::States::SharedPtr msg)
  {
    std::vector<double> values = msg->x;
    //state->getInitialState();
    // for (i:xvec.size())
    // {
    //     xvec[i] = values[i];
    // }
    RCLCPP_INFO(this->get_logger(), "State that is subscribed:");

      
  }

  void update_inputs(double T, double w){
    u = {T, w};
  }

void input_callback(const message_package::msg::Input::SharedPtr msgu)
{
    std::vector<double> values = msgu->u; // does not take double, is a double
    
      RCLCPP_INFO(this->get_logger(), "Input subscribed: %f %f", values[k], values[k+1]);
      


      update_inputs(values[k], values[k+1]);
  
      k=k+2;


}




void publish_state_callback()
{
    // publish in this callback.
    auto message = message_package::msg::States();     
    message.x = {1.0,2.0,3.0,4.0,5.0};                         
    // message.x = x_vec; Gives error
                                         
    RCLCPP_INFO(this->get_logger(), "Publish states: %f",message.x[0]);//, message.states);    
    publisher_->publish(message);


}

private:
   
  rclcpp::Subscription<message_package::msg::Input>::SharedPtr input_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;      
  rclcpp::Publisher<message_package::msg::States>::SharedPtr publisher_; 
  rclcpp::Subscription<message_package::msg::States>::SharedPtr subscription_; 
  size_t count_;   
   
   NIM* state;
 
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  Dynamics ask;
  ask.ask_nim();

  rclcpp::spin(std::make_shared<Dynamics>()); // returns callbacks until shutdown.

  rclcpp::shutdown();
  return 0;
}






// #include <iostream>
// #include <vector>
// #include <string>
// #include <memory>
// #include <functional>
// #include <chrono>

// #include "NIM.h"
// #include "ODEs.h" 
// #include "rclcpp/rclcpp.hpp"
// #include "message_package/msg/input.hpp"  
// #include "message_package/msg/num.hpp"  // veranderen door states wanneer dit werkt
// #include "message_package/msg/num2.hpp"

// using namespace std;

// using std::placeholders::_1;


// // message now: Subscribed: Num // topic: States_x, msgx
// // Published: Num2 // topic: Update_states, msg

// // Subscribed: Input // topic: Inputs_u, msgu

// class Dynamics : public rclcpp::Node
// {
// public:

//     double thrust;
//     double orient; 
//     int k = 0;

//   Dynamics()
//   : Node("Dynamics") // make node with 2 subscribers and a publisher
//   {
//     input_subscription_ = this->create_subscription<message_package::msg::Input>(          
//       "Inputs_u", 10, std::bind(&Dynamics::input_callback, this, _1));

//     state_subscription_ = this->create_subscription<message_package::msg::Num>(          
//       "States_x", 10, std::bind(&Dynamics::state_callback, this, _1));

//     // !! fix that state subscription and input subscription together trigger a callback function maybe with if input_callback = 1 as first line of the state_callback func

//     publisher_ = this->create_publisher<message_package::msg::Num2>(
//       "Update_states", 10, std::bind(&Dynamics::update_state_callback, this, _1));  
    
//   }

// void update_inputs(double T, double w){
//     thrust = T;
//     orient = w;
// }

// void input_callback(const message_package::msg::Input::SharedPtr msgu)
// {
//     std::vector<double> values = msgu->u; // does not take double, is a double

//       RCLCPP_INFO(this->get_logger(), "Input subscribed: %f %f", values[k], values[k+1]);

//       update_inputs(values[k], values[k+1]);
    
//     k=k+2;
// }

// void state_callback(const message_package::msg::Num::SharedPtr msgx) 
//  const     {
//      RCLCPP_INFO(this->get_logger(), "State subscribed: '%d'", msgx->num);    // doet het          
// }

// void update_state_callback(const message_package::msg::Num2::SharedPtr msg)
//   const  {
//     auto message = message_package::msg::Num2();                              
//     message.num2 = this->count_++;                                        
//     RCLCPP_INFO(this->get_logger(), "Publish update states: '%d'", message.num2);    
//     publisher_->publish(message);
// }

// vector<double> run_dynamics(){
//     bool nim_method; // 1 is Forward Euler, 0 is Runge Kutta
//     std::cout << "\nPlease enter enter either 0 (for Runge Kutta) or 1 (for Forward Euler)";
//     std::cin >> nim_method;

//     Dynamics input_u;
//     vector<double> u = {input_u.thrust, input_u.orient};  

    
//     if (nim_method) {
//         // This still requires a subscription to the STATE X and the INPUT U
//         f_euler state;
//         vector<double> x = state.getInitialState();
//         state.forward_euler(x, u);
//         std::cout << "\nForward Euler has finished running\n";
//         return state.forward_euler(x, u);
//     }
//     else {
//         // This still requires a subscription to the STATE X and the INPUT U
//         runge_kutta state;
//         vector<double> x = state.getInitialState();
//         state.R_K(x, u);
//         std::cout << "\nRunge Kutta finished running!\n";
//         return state.R_K(x, u);

//     }
// }

// private:
//   rclcpp::Subscription<message_package::msg::Input>::SharedPtr input_subscription_;   
//   rclcpp::Subscription<message_package::msg::Num>::SharedPtr state_subscription_; 

//   rclcpp::Publisher<message_package::msg::Num2>::SharedPtr publisher_;  

//   rclcpp::TimerBase::SharedPtr timer_;       
//   size_t count_;   
// };




// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     //Dynamics run;
//     //run.run_dynamics();  

//     rclcpp::spin(std::make_shared<Dynamics>()); // is allowing the ros node to keep running and check for events on subscribed topics and service calls, and direct the callbacks
//     // First we need to subscribe to the state, then the callback function is triggered which needs to trigger the execution of NIM and ODEs

//     rclcpp::shutdown();

//     return 0;
// }


// /*
//     //subscription_state_ = this->create_subscription<message_package::msg::States>( //
//     //  "States_x", 10, std::bind(&Dynamics::input_callback, this, _1));//

//     //publisher_state_ = this->create_publisher<message_package::msg::States>("States_x", 10);//




//       //RCLCPP_INFO(this->get_logger(), "State subscription: %f %f %f %f %f", state_values[i], 
//       //state_values[i+1], state_values[i+2], state_values[i+3], state_values[i+4]);

//     //auto message = message_package::msg::States(); 
//     // if (i == 0){
//     //    message.x = {0.0, 0.0, 0.0, 0.0, 0.0}; // for now initialize as 5, later get from classes
//     //  }
//     // else {
//     //    message.x = run_dynamics();
//     //  }
    
//     // doe iets op de input door: ros2 topic pub --once /Inputs message_package/msg/Input "{u:[2.412, 1.234]}"
    
