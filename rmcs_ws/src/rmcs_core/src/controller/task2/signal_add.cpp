#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::signal {

class SignalAdd
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SignalAdd()
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input(get_parameter("sin_input").as_string(), sin_);
        register_input(get_parameter("cos_input").as_string(), cos_);
        register_output(get_parameter("signal_result").as_string(), result_);
               }
    void update() override{
        *result_=*sin_+*cos_;
        RCLCPP_INFO(get_logger(),"sin:%lf,cos:%lf,result:%lf",*sin_,*cos_,*result_);
    }  
private:
    InputInterface<double> sin_;
    InputInterface<double> cos_;
    OutputInterface<double> result_;



};
}// namespace rmcs_core::controller::signal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::signal::SignalAdd, rmcs_executor::Component)

