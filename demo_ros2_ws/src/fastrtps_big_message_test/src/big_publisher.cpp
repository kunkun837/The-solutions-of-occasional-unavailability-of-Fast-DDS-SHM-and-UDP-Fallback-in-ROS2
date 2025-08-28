#include <rclcpp/rclcpp.hpp>  //包含ROS 2 C++客户端库，用于创建和管理ROS 2节点
#include <std_msgs/msg/u_int8_multi_array.hpp>//ROS2消息类型UInt8MultiArray的定义，一种用于传输多个8位无符号整数的数据类型。
#include <chrono>     //定时器
#include <memory>     //智能指针相关功能，std::shared_ptr
#include <unistd.h>   //提供POSIX API，如getpid()来获取进程ID。
#include <iostream>   //std::cout

class BigMessagePublisher : public rclcpp::Node  //继承自rclcpp::Node的类，表示一个ROS2节点
{
public:
    //构造函数初始化
    BigMessagePublisher() : Node("big_message_publisher"), count_(0), matched_(false), 
                           sending_messages_(false), completed_(false)                  
    {
        //配置QoS策略
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal)
            .history(rclcpp::HistoryPolicy::KeepLast);
        //创建发布者，发布UInt8MultiArray类型的消息，主题名为big_message_topic，并应用上面配置的QoS策略。
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("big_message_topic", qos);       
        //创建定时器connection_timer_，每500毫秒调用check_subscriber_connection方法，检查是否有订阅者连接。
        connection_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BigMessagePublisher::check_subscriber_connection, this));
        //创建超时定时器connection_timeout_timer_，如果在30秒内没有订阅者连接，则触发connection_timeout。
        connection_timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&BigMessagePublisher::connection_timeout, this));
            
        RCLCPP_INFO(this->get_logger(), "BigMessage Publisher started");
        RCLCPP_INFO(this->get_logger(), "Waiting for subscriber to connect before sending messages...");
    }
private:
    //变量定义
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;  //模板类，封装“发布者”对象，内部对应到底层 RMW里的 DataWriter。
    rclcpp::TimerBase::SharedPtr connection_timer_;
    rclcpp::TimerBase::SharedPtr connection_timeout_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    size_t count_;
    bool matched_;
    bool sending_messages_;
    bool completed_;  //标记是否已完成发送
    
    void check_subscriber_connection()
    {
        //如果已经完成发送，不再检查连接状态
        if (completed_) {
            return;
        }
        //使用get_subscription_count()获取当前订阅者的数量
        auto subscriber_count = publisher_->get_subscription_count();
        //如果有订阅者连接且尚未开始发送消息，则将matched_置为true，表示已找到订阅者。    
        if (subscriber_count > 0) {
            if (!matched_) {
                matched_ = true;
                RCLCPP_INFO(this->get_logger(), "Subscriber connected!");
                //取消连接超时定时器
                if (connection_timeout_timer_) {
                    connection_timeout_timer_->cancel();
                    connection_timeout_timer_.reset();
                }
                //调用start_sending()开始发送消息。
                start_sending();
            }
        } 
        //如果没有订阅者连接且之前已开始发送，则停止发送并恢复到等待订阅者的状态。
        else {
            if (matched_ && !sending_messages_) {
                // 只有在还没开始发送或者发送过程中断开连接时才重新等待
                matched_ = false;
                RCLCPP_INFO(this->get_logger(), "Waiting for subscriber to connect before sending messages...");
                stop_sending();
            }
        }
    }

    //如果在30秒内没有连接订阅者，则打印超时错误信息并关闭ROS2节点。
    void connection_timeout()
    {
        if (!completed_) {
            RCLCPP_ERROR(this->get_logger(), "TIMEOUT: No subscriber connected in 30 seconds");
            rclcpp::shutdown();
        }
    }
    
    //每2秒调用publish_message方法发送消息。
    void start_sending()
    {
        if (!sending_messages_ && !completed_) {
            sending_messages_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting to send messages...");
            publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000),
                std::bind(&BigMessagePublisher::publish_message, this));
        }
    }

    //停止消息发送，取消定时器
    void stop_sending()
    {
        if (sending_messages_) {
            sending_messages_ = false;
            if (publish_timer_) {
                publish_timer_->cancel();
                publish_timer_.reset();
            }
        }
    }

    //发送count_条消息后标记completed_为true，停止所有定时器，最终调用rclcpp::shutdown()关闭节点。
    void publish_message()
    {
        if (count_ >= 10) {
            completed_ = true;
            RCLCPP_INFO(this->get_logger(), "Completed sending 10 messages");
            //立即停止所有定时器
            stop_sending();
            if (connection_timer_) {                 //检查指针有效性
                connection_timer_->cancel();         //取消定时器
                connection_timer_.reset();           //释放智能指针
            }
            if (connection_timeout_timer_) {
                connection_timeout_timer_->cancel();
                connection_timeout_timer_.reset();
            }
            //调用rclcpp::shutdown()关闭节点
            RCLCPP_INFO(this->get_logger(), "Publisher shutting down...");
            rclcpp::shutdown();
            return;
        }
        //检查是否还有订阅者
        if (!matched_) {
            return; // 如果没有订阅者，不发送
        }

        auto message = std_msgs::msg::UInt8MultiArray();
        //设置维度信息
        message.layout.dim.resize(1);
        message.layout.dim[0].label = "big_payload";
        message.layout.dim[0].size = 1000000;         // 1MB
        message.layout.dim[0].stride = 1000000;      //一维数组，步长等于总大小，表示数据是​​连续存储​​的单一区块
        message.layout.data_offset = 0;
        //填充1MB数据
        message.data.resize(1000000);
        std::fill(message.data.begin(), message.data.end(), 0xAB);
        
        count_++;
        //打印传输轮次和数据大小
        RCLCPP_INFO(this->get_logger(), "Publishing message %lu with %lu bytes payload", count_, message.data.size());
        //异常处理，用于安全地发布消息并捕获可能的错误
        try {
            //尝试执行可能失败的操作
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Message %lu sent successfully", count_);
        } catch (const std::exception& e) {
            //捕获并处理异常
            RCLCPP_ERROR(this->get_logger(), "Failed to send message %lu: %s", 
                count_, e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "[big_publisher] PID = " << getpid() << std::endl;
    
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Big Message Publisher");
    
    auto node = std::make_shared<BigMessagePublisher>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
