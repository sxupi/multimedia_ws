#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "multimedia_cpp/ir_receiver/ir_receiver.hpp"

#include <thread>
#include <atomic>
#include <cstdio>
#include <sstream>

class IRReceiverNode : public rclcpp::Node
{
public:
    IRReceiverNode()
    : Node("ir_receiver_node"), running_(true)
    {
        pub_ = create_publisher<std_msgs::msg::String>("/ir_signal/command", 10);
        worker_ = std::thread(&IRReceiverNode::loop, this);
    }

    ~IRReceiverNode() override
    {
        running_ = false;
        if (worker_.joinable())
            worker_.join();
    }

private:
    void loop()
    {
        // Start LIRC client process
        FILE *pipe = popen("irw", "r");
        if (!pipe)
        {
            RCLCPP_ERROR(get_logger(), "Failed to start irw (is LIRC configured?)");
            return;
        }

        char buffer[256];

        while (running_ && fgets(buffer, sizeof(buffer), pipe))
        {
            std::string line(buffer);
            std::istringstream iss(line);

            // Example line: "000000037ff07bf 00 KEY_VOLUMEUP devinput"
            std::string hex, repeat, key, src;
            if (!(iss >> hex >> repeat >> key >> src))
                continue;

            auto cmd = IRMap::translate(key);

            std_msgs::msg::String msg;
            msg.data = cmd;
            pub_->publish(msg);

            RCLCPP_INFO(get_logger(), "IR: %s -> %s", key.c_str(), cmd.c_str());
        }

        pclose(pipe);
    }

    std::atomic<bool> running_;
    std::thread worker_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRReceiverNode>());
    rclcpp::shutdown();
    return 0;
}