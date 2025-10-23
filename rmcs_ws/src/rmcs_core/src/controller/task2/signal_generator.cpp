#include <cmath>
#include <chrono>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::signal {

class SignalGenerator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SignalGenerator()
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)),
          clock_(RCL_ROS_TIME) 
    {
        omega_ = get_parameter("omega").as_double(); 

        register_input("/predefined/update_count", update_count_);
        register_output(get_parameter("sin_output").as_string(), sin_out_);
        register_output(get_parameter("cos_output").as_string(), cos_out_);
    }

    void update() override {
        // 获取当前时间（秒）
        double current_time = clock_.now().seconds();

        if (*update_count_ == 0) {
            start_time_ = current_time;
        }

        double t = current_time - start_time_; // elapsed seconds

        double s = std::sin(omega_ * t);
        double c = std::cos(omega_ * t);

        *sin_out_ = s;
        *cos_out_ = c;
    }

private:
    double omega_ = 0.0;
    rclcpp::Clock clock_;
    double start_time_ = 0.0;

    InputInterface<size_t> update_count_;
    OutputInterface<double> sin_out_;
    OutputInterface<double> cos_out_;
};

} // namespace rmcs_core::controller::signal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::signal::SignalGenerator, rmcs_executor::Component)
