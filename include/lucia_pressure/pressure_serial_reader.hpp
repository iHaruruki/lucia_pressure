#ifndef PRESSURE_SERIAL_READER__PRESSURE_SERIAL_READER_HPP_
#define PRESSURE_SERIAL_READER__PRESSURE_SERIAL_READER_HPP_

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
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PressureSerialReader : public rclcpp::Node
{
public:
    PressureSerialReader();
    ~PressureSerialReader();

private:
    // Serial communication methods
    bool init_serial();
    void send_serial_data();
    void read_serial_data();
    void process_packet(const std::vector<int>& vec);
    
    // Data processing methods
    void process_pressure_data(int board_id, const std::vector<int>& vec);
    void publish_pressure_data();
    void update_moving_average();
    bool check_stability();
    
    // Serial port parameters
    std::string serial_port_;
    int baud_rate_;
    int buffer_size_;
    int serial_fd_;
    
    // ROS parameters
    std::string frame_id_;
    double publish_rate_;
    
    // Data processing parameters
    static constexpr int PRESSURE_SENSORS = 24;
    static constexpr int STABILIZE_SAMPLES = 3;
    
    // Data storage for 3-sample moving average
    std::vector<std::vector<int>> matrix_;
    std::vector<double> pressure_avg_;
    int column_index_;
    int loop_count_;
    
    // ROS2 publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr> pressure_publishers_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_publisher_;
    
    // Timer for reading serial data
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // PRESSURE_SERIAL_READER__PRESSURE_SERIAL_READER_HPP_