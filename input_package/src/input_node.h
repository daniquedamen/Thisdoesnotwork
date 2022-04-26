/*
class input_class : public rclcpp::Node
{
private:

public: 
    publish_inputs()
    : Node("input_node")
    {
     // first creating a topic
    input_publisher_ = this->create_publisher<input_package::msg::Input>("inputTopic", 10); // is subscribed in dynamics node
    }

}


class ROS2_input_class : public rclcpp:Node//inheritance
{
 public:
    {
      double T = 0; // is this necessary? or do we just u[0] and u[1] or something
      double w = 0;
        ROS2_input_class()
        : Node("ROS2_input")
        timer_ = this->create_wall_timer( 50ms, std::bind(&ROS2_input_class::timer_callback, this));

    {
        void input_function()
        {
        auto message input_package::msg::Input();
        message.u={[T,w],[2.412,1.234],[2.0, 1.52],[1.5,0.5],[2.412,1.234]}; // create random inputs; T & omega
        input_publisher_->publish(message);
        }

// You can post on the topic which is called /input in the command windows with:  ros2 topic pub --once /input cargo_drone/msg/InputVector "{u:[2.412, 1.234]}"
    }

 private:
    // reading node
    void input_callback(const input_package::msg::Input::SharedPtr msg)
    {
    vector<double> values = msg->u; //size is automatically correct
    int n_elements = values.size();

    friend class input_class
    }

}
};
*/