#ifndef PRESSURE_COMMAND_SENDER__PRESSURE_COMMAND_SENDER_HPP_
#define PRESSURE_COMMAND_SENDER__PRESSURE_COMMAND_SENDER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

// POSIX serial communication
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int64.hpp"

class PressureCommandSender : public rclcpp::Node
{
public:
    PressureCommandSender();
    ~PressureCommandSender();

private:
    // Serial communication methods
    bool init_serial();
    void close_serial();
    bool send_command_to_board(uint8_t board_id);
    void send_command_cycle();
    void continuous_transmission_timer();
    
    // Topic callbacks for command input
    void single_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void continuous_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void board_10_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void board_11_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void board_12_command_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Helper methods
    void set_continuous_mode(bool enable);
    void publish_status_updates();
    
    // Serial port parameters
    std::string serial_port_;
    int baud_rate_;
    int buffer_size_;
    int serial_fd_;
    
    // Command parameters
    int command_delay_ms_;
    bool continuous_mode_;
    std::vector<uint8_t> board_ids_;
    std::string frame_id_;
    
    // Status tracking
    uint8_t current_board_;
    uint64_t transmission_count_;
    
    // Command packet structure constants
    static constexpr uint8_t PACKET_HEADER = 0xAA;
    static constexpr uint8_t COMMAND_TYPE = 0xC1;
    static constexpr uint8_t RESERVED_BYTE = 0x00;
    static constexpr uint8_t COMMAND_PARAM = 0x21;
    static constexpr uint8_t PACKET_TAIL = 0x55;
    static constexpr int PACKET_SIZE = 16;
    
    // ROS 2 publishers (status output)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_board_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr count_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr continuous_status_publisher_;
    
    // ROS 2 subscribers (command input)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr single_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr continuous_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr board_10_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr board_11_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr board_12_command_sub_;
    
    // Timer for continuous transmission
    rclcpp::TimerBase::SharedPtr continuous_timer_;
};

#endif // PRESSURE_COMMAND_SENDER__PRESSURE_COMMAND_SENDER_HPP_