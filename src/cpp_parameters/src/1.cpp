#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
    public:
        MinimalParam()
            : Node("minimal_param_node")
            {
                auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
                param_desc.description = "This parameter is mine";

                this->declare_parameter("my_paramter","world");
                // param=my_parameter and default value as world

                timer_ = this->create_wall_timer(1000ms,
                    std::bind(&MinimalParam::timer_callback, this));
            }
            void timer_callback()
            {
                std::string my_param =
                    this->get_parameter("my_parameter").get_parameter_value().get<std::string>();

                RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

                std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter","world")};    
            }
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
}

// the output changed only for a single output 
// but we are required to have that param continued after on 
// or we don't 
