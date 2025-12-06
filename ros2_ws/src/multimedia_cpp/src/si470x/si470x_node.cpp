#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "multimedia_cpp/si470x/si470x.hpp"

class Si470xNode : public rclcpp::Node
{
public:
    Si470xNode()
    : Node("si470x_node"),
      radio_(
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1"),
        declare_parameter<int>("i2c_address", 0x10),
        declare_parameter<int>("reset_gpio", 27))
    {
        // Frequency range parameters (in MHz)
        min_freq_mhz_ = declare_parameter<double>("min_freq_mhz", 87.5).get<double>();
        max_freq_mhz_ = declare_parameter<double>("max_freq_mhz", 108.0).get<double>();

        if (!radio_.init())
        {
            RCLCPP_FATAL(get_logger(), "Failed to initialize Si470x");
            throw std::runtime_error("Si470x init failed");
        }

        // Subscribe to final frequency in 0.1 MHz units (e.g. 995 = 99.5 MHz)
        sub_ = create_subscription<std_msgs::msg::Int32>(
            "/current/frequency_int",
            10,
            std::bind(&Si470xNode::on_frequency, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Si470x node started. Listening to /current/frequency_int (0.1 MHz units). "
            "Range: [%.1f, %.1f] MHz",
            min_freq_mhz_, max_freq_mhz_);
    }

private:
    void on_frequency(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int freq10 = msg->data;  // 0.1 MHz units

        // Clamp based on configured MHz range
        int min10 = static_cast<int>(min_freq_mhz_ * 10.0 + 0.5);
        int max10 = static_cast<int>(max_freq_mhz_ * 10.0 + 0.5);

        if (freq10 < min10) freq10 = min10;
        if (freq10 > max10) freq10 = max10;

        float freq_mhz = freq10 / 10.0f;

        RCLCPP_INFO(
            get_logger(),
            "Tuning Si470x to %.1f MHz (freq10=%d)",
            freq_mhz, freq10);

        if (!radio_.set_frequency_10k(freq10))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to tune Si470x to %.1f MHz (freq10=%d)",
                freq_mhz, freq10);
        }
    }

    Si470x radio_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

    double min_freq_mhz_;
    double max_freq_mhz_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Si470xNode>());
    rclcpp::shutdown();
    return 0;
}