#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    // constructor callout with node name and count_ initialized to 0
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        // this refers to the node
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // here publisher (name) method with msg type and topic-name declaration
        // with the msg capacity of 10 (queue size limit 10)
        // here specified what type of ros msg is gonna be publshed
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
        // here timer (name) method calls out the timer_callback function to be
        // with timer with first param
        // here bind function binds the timer_callback to the timer
        // which results in function being executed twice every second
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        // here first message data type auto
        // with ros msg
        // then when you run publisher node, cppinfo responsible for
        // printing on the same terminal
        // publisher responsible for parsing just the message on the topic
        // here specified the msg which is gonna be published
    }
    // declaration here
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    // rclcpp::spin starts processing data from node
    return 0;
}

// for dependencies refer ros doc https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html