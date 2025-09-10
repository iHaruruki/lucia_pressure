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
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "lucia_pressure/srv/board_command.hpp"
#include "lucia_pressure/msg/command_status.hpp"

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
    
    // ROS 2 service callbacks
    void send_single_command_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void send_board_command_callback(
        const std::shared_ptr<pressure_command_sender::srv::BoardCommand::Request> request,
        std::shared_ptr<pressure_command_sender::srv::BoardCommand::Response> response);
    
    void start_continuous_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // External command topic callback
    void board_command_topic_callback(const std_msgs::msg::String::SharedPtr msg);
    
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
    
    // Command packet structure constants
    static constexpr uint8_t PACKET_HEADER = 0xAA;
    static constexpr uint8_t COMMAND_TYPE = 0xC1;
    static constexpr uint8_t RESERVED_BYTE = 0x00;
    static constexpr uint8_t COMMAND_PARAM = 0x21;
    static constexpr uint8_t PACKET_TAIL = 0x55;
    static constexpr int PACKET_SIZE = 16;
    
    // ROS 2 services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr send_single_service_;
    rclcpp::Service<pressure_command_sender::srv::BoardCommand>::SharedPtr send_board_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_continuous_service_;
    
    // ROS 2 publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<pressure_command_sender::msg::CommandStatus>::SharedPtr command_status_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr board_command_subscriber_;
    
    // Timer for continuous transmission
    rclcpp::TimerBase::SharedPtr continuous_timer_;
};

#endif // PRESSURE_COMMAND_SENDER__PRESSURE_COMMAND_SENDER_HPP_