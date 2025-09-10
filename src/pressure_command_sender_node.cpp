#include "lucia_pressure/pressure_command_sender.hpp"
#include <cstring>

using namespace std::chrono_literals;

PressureCommandSender::PressureCommandSender()
: Node("pressure_command_sender_node"),
  buffer_size_(16),
  serial_fd_(-1),
  command_delay_ms_(0),
  continuous_mode_(false),
  current_board_(0),
  transmission_count_(0)
{
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 2000000);
    this->declare_parameter("command_delay_ms", 100);
    this->declare_parameter("continuous_mode", true);
    this->declare_parameter("board_ids", std::vector<int64_t>{12, 11, 10});
    this->declare_parameter("frame_id", "pressure_command");

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    command_delay_ms_ = this->get_parameter("command_delay_ms").as_int();
    continuous_mode_ = this->get_parameter("continuous_mode").as_bool();
    frame_id_ = this->get_parameter("frame_id").as_string();
    
    // Convert board IDs from int64_t to uint8_t
    auto board_ids_param = this->get_parameter("board_ids").as_integer_array();
    board_ids_.clear();
    for (const auto& id : board_ids_param) {
        board_ids_.push_back(static_cast<uint8_t>(id));
    }

    // Initialize serial communication
    if (!init_serial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication");
        return;
    }

    // Create publishers (status output)
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("~/command_status", 10);
    current_board_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("~/current_board", 10);
    count_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/transmission_count", 10);
    continuous_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("~/continuous_mode_active", 10);

    // Create subscribers (command input)
    single_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/command_single", 10,
        std::bind(&PressureCommandSender::single_command_callback, this, std::placeholders::_1));
    
    continuous_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/command_continuous", 10,
        std::bind(&PressureCommandSender::continuous_command_callback, this, std::placeholders::_1));
    
    board_10_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/command_board_10", 10,
        std::bind(&PressureCommandSender::board_10_command_callback, this, std::placeholders::_1));
    
    board_11_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/command_board_11", 10,
        std::bind(&PressureCommandSender::board_11_command_callback, this, std::placeholders::_1));
    
    board_12_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/command_board_12", 10,
        std::bind(&PressureCommandSender::board_12_command_callback, this, std::placeholders::_1));

    // Start continuous mode if enabled
    if (continuous_mode_) {
        continuous_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(command_delay_ms_ * board_ids_.size()),
            std::bind(&PressureCommandSender::continuous_transmission_timer, this));
        RCLCPP_INFO(this->get_logger(), "Continuous transmission mode started");
    }

    RCLCPP_INFO(this->get_logger(), "Pressure Command Sender Node started");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s, Baud rate: %d", serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Command delay: %d ms, Continuous mode: %s", 
                command_delay_ms_, continuous_mode_ ? "enabled" : "disabled");
    
    // Print board IDs
    std::string board_list;
    for (size_t i = 0; i < board_ids_.size(); ++i) {
        board_list += std::to_string(board_ids_[i]);
        if (i < board_ids_.size() - 1) board_list += ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Board IDs: [%s]", board_list.c_str());
    
    // Publish initial status
    publish_status_updates();
}

PressureCommandSender::~PressureCommandSender()
{
    close_serial();
}

bool PressureCommandSender::init_serial()
{
    struct termios oldtio, newtio;
    
    // Open serial port
    serial_fd_ = open(serial_port_.c_str(), O_RDWR);
    
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Serial port opened, fd: %d", serial_fd_);
    
    // Configure serial port (matching reference code exactly)
    ioctl(serial_fd_, TCGETS, &oldtio);
    newtio.c_cflag = baud_rate_ | CS8 | CREAD;
    tcsetattr(serial_fd_, TCSANOW, &newtio);
    ioctl(serial_fd_, TCSETS, &newtio);
    
    return true;
}

void PressureCommandSender::close_serial()
{
    if (serial_fd_ >= 0) {
        struct termios oldtio;
        ioctl(serial_fd_, TCGETS, &oldtio);
        ioctl(serial_fd_, TCSETS, &oldtio);
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
}

bool PressureCommandSender::send_command_to_board(uint8_t board_id)
{
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not open");
        return false;
    }

    // Create command packet matching reference code exactly
    unsigned char txData[PACKET_SIZE] = {};
    txData[0] = PACKET_HEADER;   // 0xAA
    txData[1] = COMMAND_TYPE;    // 0xC1
    txData[2] = board_id;        // Board ID (0x0A, 0x0B, 0x0C)
    txData[3] = RESERVED_BYTE;   // 0x00
    txData[4] = COMMAND_PARAM;   // 0x21
    txData[5] = PACKET_TAIL;     // 0x55

    // Send command
    ssize_t bytes_written = write(serial_fd_, txData, PACKET_SIZE);
    
    if (bytes_written != PACKET_SIZE) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write command to board %d", board_id);
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Command sent to board %d (0x%02X)", board_id, board_id);

    // Update status tracking
    current_board_ = board_id;
    transmission_count_++;

    // Publish status updates
    publish_status_updates();

    return true;
}

void PressureCommandSender::send_command_cycle()
{
    for (const auto& board_id : board_ids_) {
        if (!send_command_to_board(board_id)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command to board %d", board_id);
            continue;
        }
        
        // Add delay between commands (matching reference code)
        if (command_delay_ms_ > 0) {
            usleep(command_delay_ms_ * 1000);  // Convert ms to microseconds
        }
    }
}

void PressureCommandSender::continuous_transmission_timer()
{
    send_command_cycle();
}

void PressureCommandSender::single_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        send_command_cycle();
        RCLCPP_INFO(this->get_logger(), "Single command cycle executed");
    }
}

void PressureCommandSender::continuous_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    set_continuous_mode(msg->data);
}

void PressureCommandSender::board_10_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        send_command_to_board(10);
        RCLCPP_INFO(this->get_logger(), "Board 10 command executed");
    }
}

void PressureCommandSender::board_11_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        send_command_to_board(11);
        RCLCPP_INFO(this->get_logger(), "Board 11 command executed");
    }
}

void PressureCommandSender::board_12_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        send_command_to_board(12);
        RCLCPP_INFO(this->get_logger(), "Board 12 command executed");
    }
}

void PressureCommandSender::set_continuous_mode(bool enable)
{
    try {
        continuous_mode_ = enable;
        
        if (continuous_mode_) {
            if (!continuous_timer_) {
                continuous_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(command_delay_ms_ * board_ids_.size()),
                    std::bind(&PressureCommandSender::continuous_transmission_timer, this));
            }
            RCLCPP_INFO(this->get_logger(), "Continuous transmission started");
        } else {
            if (continuous_timer_) {
                continuous_timer_.reset();
            }
            RCLCPP_INFO(this->get_logger(), "Continuous transmission stopped");
        }
        
        // Publish status update
        publish_status_updates();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Continuous mode change failed: %s", e.what());
    }
}

void PressureCommandSender::publish_status_updates()
{
    // Publish command status
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "Command sent to board " + std::to_string(current_board_);
    status_publisher_->publish(status_msg);

    // Publish current board
    auto board_msg = std_msgs::msg::UInt8();
    board_msg.data = current_board_;
    current_board_publisher_->publish(board_msg);

    // Publish transmission count
    auto count_msg = std_msgs::msg::UInt64();
    count_msg.data = transmission_count_;
    count_publisher_->publish(count_msg);

    // Publish continuous mode status
    auto continuous_msg = std_msgs::msg::Bool();
    continuous_msg.data = continuous_mode_;
    continuous_status_publisher_->publish(continuous_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        rclcpp::spin(std::make_shared<PressureCommandSender>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pressure_command_sender"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}