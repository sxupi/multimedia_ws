#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "multimedia_cpp/lcd_1602/lcd_1602.hpp"

class Lcd1602Node : public rclcpp::Node
{
public:
    Lcd1602Node()
    : Node("lcd_1602_node"),
      display_(declare_parameter<int>("i2c_address", 0x27), 16, 2)
    {
        if (!display_.init())
        {
            RCLCPP_FATAL(get_logger(), "Failed to initialize LCD 1602 display");
            throw std::runtime_error("LCD init failed");
        }

        sub_line1_ = create_subscription<std_msgs::msg::String>(
            "1602_display/first_line", 10,
            std::bind(&Lcd1602Node::on_line1, this, std::placeholders::_1));

        sub_line2_ = create_subscription<std_msgs::msg::String>(
            "1602_display/second_line", 10,
            std::bind(&Lcd1602Node::on_line2, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&Lcd1602Node::on_timer, this));
    }

private:
    void on_line1(const std_msgs::msg::String::SharedPtr msg)
    {
        full_line1_ = msg->data;
        offset1_ = 0;
    }

    void on_line2(const std_msgs::msg::String::SharedPtr msg)
    {
        full_line2_ = msg->data;
        offset2_ = 0;
    }

    std::string make_marquee(const std::string &text, size_t &offset)
    {
        const auto width = static_cast<size_t>(display_.cols());

        if (text.size() <= width)
        {
            // no scrolling needed
            return text;
        }

        constexpr size_t gap = 3;
        std::string scroll = text + std::string(gap, ' ');

        std::string out;
        out.reserve(width);
        for (size_t i = 0; i < width; ++i)
        {
            out.push_back(scroll[(offset + i) % scroll.size()]);
        }

        offset = (offset + 1) % scroll.size();
        return out;
    }

    void on_timer()
    {
        auto l1 = make_marquee(full_line1_, offset1_);
        auto l2 = make_marquee(full_line2_, offset2_);

        display_.print_line(0, l1);
        display_.print_line(1, l2);
    }

    Lcd1602Display display_;

    std::string full_line1_;
    std::string full_line2_;
    size_t offset1_ = 0;
    size_t offset2_ = 0;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_line1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_line2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lcd1602Node>());
    rclcpp::shutdown();
    return 0;
}