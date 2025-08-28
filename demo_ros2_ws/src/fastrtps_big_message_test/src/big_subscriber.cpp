#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <unistd.h>
#include <iostream>

class BigMessageSubscriber : public rclcpp::Node
{
public:
    BigMessageSubscriber() : Node("big_message_subscriber"), received_count_(0), matched_(false)
    {
        //相同的QoS配置
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal)
            .history(rclcpp::HistoryPolicy::KeepLast);
        //创建订阅者对象，当消息类型、话题名和QoS兼容时才匹配。
        //绑定回调成员函数，将收到的消息的指针转给回调的第一个参数。每收到一条消息，都调用message_callback验证收到数据的完整性
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>("big_message_topic", qos,
            std::bind(&BigMessageSubscriber::message_callback, this, std::placeholders::_1));   
        //创建定时器，当30s内没匹配发布者时判定超时并退出。
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),  
            std::bind(&BigMessageSubscriber::timeout_check, this));
        //每1s检查是否已收满10条；满了就调用check_completion执行shutdown()
        completion_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BigMessageSubscriber::check_completion, this));
        //每500ms调用 check_publisher_connection()，用publisher_count判断是否匹配成功/断开。
        connection_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BigMessageSubscriber::check_publisher_connection, this));
        RCLCPP_INFO(this->get_logger(), "BigMessage Subscriber started");
        RCLCPP_INFO(this->get_logger(), "Waiting for publisher to connect...");
    }

private:
    //变量定义
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr completion_timer_;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    size_t received_count_;
    bool matched_;
    //检查收到的每条消息长度是否是1MB；
    void message_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        received_count_++;
        RCLCPP_INFO(this->get_logger(),"Received message %lu with %lu bytes payload (%lu/10)", received_count_, msg->data.size(), received_count_);
        //验证数据完整性
        if (msg->data.size() == 1000000) {
            RCLCPP_DEBUG(this->get_logger(), "Payload size correct");
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected payload size: %lu", msg->data.size());
        }
    }

    //检查是否有发布者连接
    void check_publisher_connection()
    {
        auto publisher_count = subscription_->get_publisher_count();
        
        if (publisher_count > 0) {
            if (!matched_) {
                matched_ = true;
                RCLCPP_INFO(this->get_logger(), "Publisher connected!");
                //一旦匹配，取消超时定时器
                if (timeout_timer_) {
                    timeout_timer_->cancel();
                    timeout_timer_.reset();
                }
                RCLCPP_INFO(this->get_logger(), "Waiting to receive messages...");
            }
        } else {
            if (matched_) {
                matched_ = false;
                RCLCPP_INFO(this->get_logger(), "Waiting for publisher to connect...");
            }
        }
    }

    //检查是否已经收到十条消息
    void check_completion()
    {
        if (received_count_ >= 10) {
            RCLCPP_INFO(this->get_logger(), "Successfully received all 10 messages!");
            rclcpp::shutdown();
        }
    }

    //超时检查定时器
    void timeout_check()
    {
        if (!matched_) {
            RCLCPP_ERROR(this->get_logger(), "TIMEOUT: No publisher connected in 30 seconds");
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "[big_subscriber] PID = " << getpid() << std::endl;
    
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Big Message Subscriber");
    
    auto node = std::make_shared<BigMessageSubscriber>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
