/*
 * Pressure Serial Reader ROS2 Node
 * Reads pressure and potentiometer data from serial devices and publishes to ROS topics
 * 
 * This node:
 * - Opens and reads a serial port (default /dev/ttyUSB0) at 9600 baud using POSIX termios
 * - Continuously parses incoming packets with Header: 0xAA, Tail: 0x55
 * - Extracts fields for Board IDs 10, 11, 12 (pressure/biosignal board)
 * - When vec[1] == 197 (0xC5): Vital format with Pulse, SpO2, and BloPre values
 * - Publishes pressure values using sensor_msgs/msg/FluidPressure on topic /pressure/board_<id>
 */

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

using namespace std::chrono_literals;

class PressureSerialReaderNode : public rclcpp::Node
{
public:
    PressureSerialReaderNode()
    : Node("pressure_serial_reader_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 9600);
        this->declare_parameter<double>("read_frequency", 100.0); // Hz
        
        // Get parameters
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        double frequency = this->get_parameter("read_frequency").as_double();
        
        // Create publishers for each board ID (10, 11, 12)
        publisher_board_10_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pressure/board_10", 10);
        publisher_board_11_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pressure/board_11", 10);
        publisher_board_12_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pressure/board_12", 10);
        
        // Initialize serial port
        if (!init_serial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        // Create timer to read serial data
        auto period = std::chrono::duration<double>(1.0 / frequency);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&PressureSerialReaderNode::read_serial_data, this));
            
        RCLCPP_INFO(this->get_logger(), "Pressure Serial Reader Node started");
        RCLCPP_INFO(this->get_logger(), "Serial port: %s, Baud rate: %d", serial_port_.c_str(), baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Publishing pressure data on topics: /pressure/board_10, /pressure/board_11, /pressure/board_12");
    }
    
    ~PressureSerialReaderNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool init_serial()
    {
        // Open serial port
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
            return false;
        }
        
        // Configure serial port
        struct termios options;
        memset(&options, 0, sizeof(options));
        
        // Get current options
        if (tcgetattr(serial_fd_, &options) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        // Set baud rate
        speed_t baud;
        switch (baud_rate_) {
            case 9600:
                baud = B9600;
                break;
            case 19200:
                baud = B19200;
                break;
            case 38400:
                baud = B38400;
                break;
            case 57600:
                baud = B57600;
                break;
            case 115200:
                baud = B115200;
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 9600", baud_rate_);
                baud = B9600;
                break;
        }
        
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        // Configure port settings
        options.c_cflag |= CS8 | CLOCAL | CREAD; // 8 bits, local connection, enable receiver
        options.c_cflag &= ~PARENB;   // No parity
        options.c_cflag &= ~CSTOPB;   // 1 stop bit
        options.c_cflag &= ~CSIZE;    // Clear size mask
        
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // No flow control
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_oflag &= ~OPOST; // Raw output
        
        options.c_cc[VMIN] = 0;   // Minimum chars to read
        options.c_cc[VTIME] = 1;  // Timeout in deciseconds
        
        // Apply settings
        if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        // Flush buffers
        tcflush(serial_fd_, TCIOFLUSH);
        
        return true;
    }
    
    void read_serial_data()
    {
        if (serial_fd_ < 0) {
            return;
        }
        
        const int BUFF_SIZE = 100;
        unsigned char buffer[BUFF_SIZE];
        
        // Read data from serial port
        int len = read(serial_fd_, buffer, BUFF_SIZE);
        
        if (len <= 0) {
            return; // No data available or error
        }
        
        // Process received data - look for packets with 0xAA header and 0x55 tail
        for (int i = 0; i < len; i++) {
            if (buffer[i] == 0xAA) { // Packet start detected
                std::vector<int> packet;
                packet.push_back(buffer[i]);
                i++; // Move to next byte
                
                // Read until 0x55 (tail) or end of buffer
                while (i < len && buffer[i] != 0x55) {
                    packet.push_back(buffer[i]);
                    i++;
                }
                
                if (i < len && buffer[i] == 0x55) {
                    packet.push_back(buffer[i]); // Add tail
                    process_packet(packet);
                }
            }
        }
    }
    
    void process_packet(const std::vector<int>& vec)
    {
        // Check if packet has minimum required length
        if (vec.size() < 9) {
            return;
        }
        
        // Check for board IDs 10, 11, 12 (pressure/biosignal board)
        int board_id = vec[2];
        if (board_id != 10 && board_id != 11 && board_id != 12) {
            return;
        }
        
        // Check if this is vital format (vec[1] == 197 = 0xC5)
        if (vec[1] == 197 && vec.size() >= 9) {
            // Extract vital data
            int pulse = vec[4];
            double spo2 = static_cast<double>(vec[6] * 256 + vec[5]) * 0.1;
            double blo_pre = static_cast<double>(vec[8] * 256 + vec[7]) * 0.1;
            
            RCLCPP_DEBUG(this->get_logger(), "Board %d - Pulse: %d, SpO2: %.1f, BloPre: %.1f", 
                        board_id, pulse, spo2, blo_pre);
            
            // Publish pressure data (using BloPre as pressure value)
            publish_pressure_data(board_id, blo_pre);
        }
    }
    
    void publish_pressure_data(int board_id, double pressure_value)
    {
        auto message = sensor_msgs::msg::FluidPressure();
        
        // Set header
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "pressure_sensor_board_" + std::to_string(board_id);
        
        // Convert pressure value to Pascals (assuming input is in mmHg, convert to Pa)
        // 1 mmHg = 133.322 Pa
        message.fluid_pressure = pressure_value * 133.322;
        
        // Set variance (uncertainty in measurement)
        message.variance = 100.0; // Pa^2
        
        // Publish to appropriate topic based on board ID
        switch (board_id) {
            case 10:
                publisher_board_10_->publish(message);
                break;
            case 11:
                publisher_board_11_->publish(message);
                break;
            case 12:
                publisher_board_12_->publish(message);
                break;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Published pressure %.2f Pa from board %d", 
                    message.fluid_pressure, board_id);
    }
    
    // Member variables
    std::string serial_port_;
    int baud_rate_;
    int serial_fd_ = -1;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_board_10_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_board_11_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_board_12_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        rclcpp::spin(std::make_shared<PressureSerialReaderNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pressure_serial_reader"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}