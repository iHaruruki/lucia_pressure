/*
 * Pressure Serial Reader ROS2 Node
 * Reads pressure sensor data from serial communication and publishes it to ROS topics
 * Based on reference code from seRe_pressure/seRe_simple.cpp
 * 
 * This node:
 * - Opens and reads a serial port (default /dev/ttyUSB0) at 2000000 baud using POSIX termios
 * - Continuously parses incoming packets with Header: 0xAA, Tail: 0x55
 * - Processes pressure data when vec[1]==200 and len==13
 * - Extracts 8 pressure values from vec[4] to vec[11] for Board IDs 10, 11, 12
 * - Implements 3-sample moving average and stability detection
 * - Maps board IDs to sensor positions (Board 10→20-23, Board 11→12-15, Board 12→0-7)
 * - Publishes individual pressure values on /pressure/sensor_<ID> topics
 * - Publishes averaged pressure array on /pressure/averaged_array topic
 */

#include "lucia_pressure/pressure_serial_reader.hpp"
#include <cmath>

using namespace std::chrono_literals;

PressureSerialReader::PressureSerialReader()
: Node("pressure_serial_reader_node"),
  serial_fd_(-1),
  matrix_(PRESSURE_SENSORS, std::vector<int>(STABILIZE_SAMPLES, 0)),
  pressure_avg_(PRESSURE_SENSORS, 0.0),
  column_index_(0),
  loop_count_(0)
{
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 2000000);
    this->declare_parameter<int>("buffer_size", 26);
    this->declare_parameter<std::string>("frame_id", "pressure_sensor");
    this->declare_parameter<double>("publish_rate", 10.0);
    
    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    buffer_size_ = this->get_parameter("buffer_size").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    
    // Create publishers for individual pressure sensors (0-23)
    pressure_publishers_.resize(PRESSURE_SENSORS);
    for (int i = 0; i < PRESSURE_SENSORS; i++) {
        std::string topic_name = "/pressure/sensor_" + std::to_string(i);
        pressure_publishers_[i] = this->create_publisher<sensor_msgs::msg::FluidPressure>(topic_name, 10);
    }
    
    // Create publisher for averaged pressure array
    array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pressure/averaged_array", 10);
    
    // Initialize serial port
    if (!init_serial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", serial_port_.c_str());
        rclcpp::shutdown();
        return;
    }
    
    // Create timer to read serial data
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&PressureSerialReader::read_serial_data, this));
        
    RCLCPP_INFO(this->get_logger(), "Pressure Serial Reader Node started");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s, Baud rate: %d", serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Publishing individual pressure data on topics: /pressure/sensor_0 to /pressure/sensor_23");
    RCLCPP_INFO(this->get_logger(), "Publishing averaged pressure array on topic: /pressure/averaged_array");
}

PressureSerialReader::~PressureSerialReader()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}
bool PressureSerialReader::init_serial()
{
    // Open serial port
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        return false;
    }
    
    // Configure serial port using termios (matching reference code)
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VTIME] = 100;
    
    // Set baud rate (2000000 from reference code)
    // Note: B2000000 might not be available on all systems
    speed_t baud;
    switch (baud_rate_) {
        case 2000000:
            #ifdef B2000000
            baud = B2000000;
            #else
            RCLCPP_WARN(this->get_logger(), "B2000000 not supported, using B1000000");
            baud = B1000000;
            #endif
            break;
        case 1000000:
            baud = B1000000;
            break;
        case 115200:
            baud = B115200;
            break;
        case 57600:
            baud = B57600;
            break;
        case 38400:
            baud = B38400;
            break;
        case 19200:
            baud = B19200;
            break;
        case 9600:
            baud = B9600;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 115200", baud_rate_);
            baud = B115200;
            break;
    }
    
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    
    // Apply settings to device (matching reference code)
    if (tcsetattr(serial_fd_, TCSANOW, &tio) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    return true;
}
    
void PressureSerialReader::read_serial_data()
{
    if (serial_fd_ < 0) {
        return;
    }
    
    unsigned char buffer[buffer_size_];
    
    // Read data from serial port (matching reference code buffer size)
    int len = read(serial_fd_, buffer, buffer_size_);
    
    if (len == 0) {
        // No data available - continue
        return;
    }
    
    if (len < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error reading from serial port");
        return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Read %d bytes from serial port", len);
    
    // Process received data - look for packets with 0xAA header and 0x55 tail
    // This logic matches the reference code packet parsing
    static std::vector<int> vec;  // Static to maintain state between calls
    
    for (int i = 0; i < len; i++) {
        if (buffer[i] == 0xAA) {  // Packet start detected
            vec.clear();  // New packet starts - clear previous data
            vec.push_back(buffer[i]);
            i++;  // Move to next byte after header
            
            // Read until 0x55 (tail) or end of buffer
            while (i < len && buffer[i] != 0x55) {
                vec.push_back(buffer[i]);
                i++;
            }
            
            if (i < len) {  // 0x55 found
                // Process complete packet
                process_packet(vec);
            }
            // If packet is incomplete, vec will be maintained for next read
        }
    }
}
    
void PressureSerialReader::process_packet(const std::vector<int>& vec)
{
    // Check for pressure data format: vec[1]==200 && len==13 (matching reference code)
    if (vec.size() >= 13 && vec[1] == 200) {
        int board_id = vec[2];
        
        // Process only Board IDs 10, 11, 12 (matching reference code)
        if (board_id == 10 || board_id == 11 || board_id == 12) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "[Id=%d] %d, %d, %d, %d, %d, %d, %d, %d", 
                        board_id, vec[4], vec[5], vec[6], vec[7], 
                        vec[8], vec[9], vec[10], vec[11]);
            
            process_pressure_data(board_id, vec);
        }
    }
    else {
        if (vec.size() >= 3) {
            RCLCPP_DEBUG(this->get_logger(), "[ID=%d] ERROR - invalid format", vec[2]);
        }
    }
}

void PressureSerialReader::process_pressure_data(int board_id, const std::vector<int>& vec)
{
    // Update loop count for column management (matching reference code logic)
    if (loop_count_ % 3 == 0) {
        column_index_ = (loop_count_ / 3) % STABILIZE_SAMPLES;
    }
    loop_count_++;
    
    // Map board IDs to matrix indices (exactly matching reference code)
    if (board_id == 10) {
        // Board 10: vec[4-11] → matrix[20-27] (j +16 - 4 = j + 12)
        for (int j = 4; j <= 11; j++) {
            if (j < static_cast<int>(vec.size())) {
                matrix_[j + 16 - 4][column_index_] = vec[j];
            }
        }
    }
    else if (board_id == 11) {
        // Board 11: vec[4-11] → matrix[12-19] (j + 8 - 4 = j + 4)
        for (int j = 4; j <= 11; j++) {
            if (j < static_cast<int>(vec.size())) {
                matrix_[j + 8 - 4][column_index_] = vec[j];
            }
        }
    }
    else if (board_id == 12) {
        // Board 12: vec[4-11] → matrix[0-7] (j - 4)
        for (int j = 4; j <= 11; j++) {
            if (j < static_cast<int>(vec.size())) {
                matrix_[j - 4][column_index_] = vec[j];
            }
        }
    }
    
    // After collecting enough samples (9 cycles like reference code), calculate averages
    if (loop_count_ >= 9) {
        update_moving_average();
        
        // Check stability (matching reference code logic)
        if (check_stability()) {
            // Stability achieved - publish data
            publish_pressure_data();
        } else {
            // Not stable - reset loop count
            loop_count_ = 0;
        }
    }
}
    
void PressureSerialReader::update_moving_average()
{
    // Calculate 3-sample moving average for each sensor (matching reference code)
    for (int i = 0; i < PRESSURE_SENSORS; i++) {
        if (matrix_[i].size() >= STABILIZE_SAMPLES) {
            pressure_avg_[i] = (matrix_[i][0] + matrix_[i][1] + matrix_[i][2]) / 3.0;
            RCLCPP_DEBUG(this->get_logger(), 
                        "PRESSURE_AVG[%d] = %d + %d + %d = %.2f", 
                        i, matrix_[i][0], matrix_[i][1], matrix_[i][2], pressure_avg_[i]);
        }
    }
}

bool PressureSerialReader::check_stability()
{
    // Check stability: all samples must be within ±5 of average (matching reference code)
    for (int i = 0; i < PRESSURE_SENSORS; i++) {
        if (matrix_[i].size() >= STABILIZE_SAMPLES) {
            for (int j = 0; j < STABILIZE_SAMPLES; j++) {
                if (std::abs(pressure_avg_[i] - matrix_[i][j]) >= 5) {
                    return false;  // Not stable
                }
            }
        }
    }
    return true;  // All sensors are stable
}

void PressureSerialReader::publish_pressure_data()
{
    auto timestamp = this->get_clock()->now();
    
    // Publish individual sensor data (sensors 0-23)
    for (int i = 0; i < PRESSURE_SENSORS; i++) {
        auto message = sensor_msgs::msg::FluidPressure();
        
        // Set header
        message.header.stamp = timestamp;
        message.header.frame_id = frame_id_;
        
        // Set pressure value (convert to Pascals if needed)
        message.fluid_pressure = pressure_avg_[i];  // Using raw value for now
        
        // Set variance (uncertainty in measurement)
        message.variance = 1.0;  // Pa^2
        
        // Publish to sensor-specific topic
        pressure_publishers_[i]->publish(message);
    }
    
    // Publish averaged array
    auto array_msg = std_msgs::msg::Float64MultiArray();
    array_msg.data = pressure_avg_;
    array_publisher_->publish(array_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published stable pressure data for all %d sensors", PRESSURE_SENSORS);
}
    
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        rclcpp::spin(std::make_shared<PressureSerialReader>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pressure_serial_reader"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}