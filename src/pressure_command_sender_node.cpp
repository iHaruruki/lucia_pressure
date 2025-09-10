#include "lucia_pressure/pressure_command_sender.hpp"
#include <cstring>

using namespace std::chrono_literals;

PressureCommandSender::PressureCommandSender()
: Node("pressure_command_sender_node"),
  serial_fd_(-1),
  buffer_size_(16)
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

    // Create services
    send_single_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/send_single_command",
        std::bind(&PressureCommandSender::send_single_command_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    send_board_service_ = this->create_service<pressure_command_sender::srv::BoardCommand>(
        "~/send_board_command",
        std::bind(&PressureCommandSender::send_board_command_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    start_continuous_service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/start_continuous",
        std::bind(&PressureCommandSender::start_continuous_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Create publishers
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("~/command_status", 10);
    command_status_publisher_ = this->create_publisher<pressure_command_sender::msg::CommandStatus>(
        "~/command_status_detailed", 10);

    // Create subscriber
    board_command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "~/board_command", 10,
        std::bind(&PressureCommandSender::board_command_topic_callback, this, std::placeholders::_1));

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

    // Publish command status
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "Command sent to board " + std::to_string(board_id);
    status_publisher_->publish(status_msg);

    // Publish detailed status
    auto detailed_status = pressure_command_sender::msg::CommandStatus();
    detailed_status.header.stamp = this->get_clock()->now();
    detailed_status.header.frame_id = frame_id_;
    detailed_status.board_id = board_id;
    detailed_status.success = true;
    detailed_status.status = "Command transmitted successfully";
    command_status_publisher_->publish(detailed_status);

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

void PressureCommandSender::send_single_command_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    try {
        send_command_cycle();
        response->success = true;
        response->message = "Single command cycle completed";
        RCLCPP_INFO(this->get_logger(), "Single command cycle executed");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Failed to execute command cycle: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Single command cycle failed: %s", e.what());
    }
}

void PressureCommandSender::send_board_command_callback(
    const std::shared_ptr<pressure_command_sender::srv::BoardCommand::Request> request,
    std::shared_ptr<pressure_command_sender::srv::BoardCommand::Response> response)
{
    try {
        bool success = send_command_to_board(request->board_id);
        response->success = success;
        response->message = success ? 
            "Command sent to board " + std::to_string(request->board_id) :
            "Failed to send command to board " + std::to_string(request->board_id);
        
        RCLCPP_INFO(this->get_logger(), "Board command service: %s", response->message.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Exception: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Board command service failed: %s", e.what());
    }
}

void PressureCommandSender::start_continuous_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    try {
        continuous_mode_ = request->data;
        
        if (continuous_mode_) {
            if (!continuous_timer_) {
                continuous_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(command_delay_ms_ * board_ids_.size()),
                    std::bind(&PressureCommandSender::continuous_transmission_timer, this));
            }
            response->message = "Continuous transmission started";
        } else {
            if (continuous_timer_) {
                continuous_timer_.reset();
            }
            response->message = "Continuous transmission stopped";
        }
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Failed to change continuous mode: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Continuous mode change failed: %s", e.what());
    }
}

void PressureCommandSender::board_command_topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    try {
        // Parse board ID from string message
        int board_id = std::stoi(msg->data);
        if (board_id >= 0 && board_id <= 255) {
            send_command_to_board(static_cast<uint8_t>(board_id));
            RCLCPP_INFO(this->get_logger(), "Board command topic: sent to board %d", board_id);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid board ID from topic: %d", board_id);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse board ID from topic: %s", e.what());
    }
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