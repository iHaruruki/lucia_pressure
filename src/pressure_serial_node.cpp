#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

class PressureSerialNode : public rclcpp::Node {
public:
  PressureSerialNode()
  : Node("pressure_serial_node"),
    fd_(-1),
    running_(true),
    send_thread_(nullptr),
    read_thread_(nullptr)
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 2000000);
    command_delay_ms_ = declare_parameter<int>("command_delay_ms", 100);
    board_ids_ = declare_parameter<std::vector<int64_t>>("board_ids", std::vector<int64_t>{10, 11, 12});

    // Prepare publishers for each board ID
    for (auto id64 : board_ids_) {
      int id = static_cast<int>(id64);
      std::string topic = "pressure/board/" + std::to_string(id);
      publishers_[id] = this->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
      RCLCPP_INFO(get_logger(), "Advertising: %s", topic.c_str());
    }

    if (!open_and_configure(serial_port_, baud_rate_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open/configure serial port: %s", serial_port_.c_str());
      throw std::runtime_error("Serial open failed");
    }

    // Threads for sending requests and reading responses
    send_thread_ = std::make_unique<std::thread>(&PressureSerialNode::send_loop, this);
    read_thread_ = std::make_unique<std::thread>(&PressureSerialNode::read_loop, this);
  }

  ~PressureSerialNode() override {
    running_ = false;
    if (send_thread_ && send_thread_->joinable()) send_thread_->join();
    if (read_thread_ && read_thread_->joinable()) read_thread_->join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  bool open_and_configure(const std::string &device, int baud) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %m", device.c_str());
      return false;
    }

    // Get current options
    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %m");
      return false;
    }

    cfmakeraw(&tty);

    // Set baud
    speed_t speed;
    switch (baud) {
      case 2000000: speed = B2000000; break;
      case 1000000: speed = B1000000; break;
      case 921600:  speed = B921600;  break;
      case 460800:  speed = B460800;  break;
      case 230400:  speed = B230400;  break;
      case 115200:  speed = B115200;  break;
      default:
        RCLCPP_WARN(get_logger(), "Unsupported baud %d; falling back to 2000000 if available", baud);
        speed = B2000000;
        break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    // Read timeout
    tty.c_cc[VMIN] = 0;   // non-blocking read
    tty.c_cc[VTIME] = 1;  // 0.1 sec timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %m");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Serial opened %s @ %d", device.c_str(), baud);
    return true;
  }

  void send_loop() {
    rclcpp::Rate rate(1000.0 / std::max(1, command_delay_ms_));
    size_t idx = 0;
    std::vector<uint8_t> ids;
    ids.reserve(board_ids_.size());
    for (auto v : board_ids_) ids.push_back(static_cast<uint8_t>(v));

    while (rclcpp::ok() && running_) {
      if (ids.empty()) { rate.sleep(); continue; }
      uint8_t id = ids[idx % ids.size()];
      send_request(id);
      idx++;
      rate.sleep();
    }
  }

  void send_request(uint8_t board_id) {
    // Frame: AA C1 [ID] 00 21 55
    uint8_t frame[6] = {0xAA, 0xC1, board_id, 0x00, 0x21, 0x55};
    ssize_t n = ::write(fd_, frame, sizeof(frame));
    if (n != (ssize_t)sizeof(frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Serial write incomplete (%zd/6): %m", n);
    }
  }

  void read_loop() {
    std::vector<uint8_t> buf;
    buf.reserve(256);
    uint8_t tmp[256];

    while (rclcpp::ok() && running_) {
      ssize_t n = ::read(fd_, tmp, sizeof(tmp));
      if (n > 0) {
        buf.insert(buf.end(), tmp, tmp + n);
        parse_frames(buf);
      } else {
        // Sleep briefly to avoid busy loop
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void parse_frames(std::vector<uint8_t> &buf) {
    // Expected RX frame length is 13 bytes:
    // 0:0xAA 1:0xC8 2:ID 3:0x00 4..11:data(8) 12:0x55
    const size_t FRAME_LEN = 13;

    // Iterate and consume frames
    size_t i = 0;
    while (buf.size() - i >= FRAME_LEN) {
      // Find header 0xAA
      while (i < buf.size() && buf[i] != 0xAA) i++;
      if (buf.size() - i < FRAME_LEN) break;

      if (buf[i] == 0xAA && buf[i+1] == 0xC8 && buf[i+3] == 0x00 && buf[i+12] == 0x55) {
        uint8_t id = buf[i+2];
        // Data bytes 8 channels
        std::array<uint8_t, 8> ch{};
        for (int k = 0; k < 8; ++k) ch[k] = buf[i + 4 + k];

        publish_data(id, ch);

        // Consume this frame
        i += FRAME_LEN;
      } else {
        // Not a valid frame; skip one byte
        i += 1;
      }
    }
    // Erase consumed bytes
    if (i > 0) buf.erase(buf.begin(), buf.begin() + i);
  }

  void publish_data(uint8_t id, const std::array<uint8_t,8> &ch) {
    int id_dec = static_cast<int>(id);
    auto it = publishers_.find(id_dec);
    if (it == publishers_.end()) {
      // If unknown id, create a publisher on the fly for visibility
      auto pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("pressure/board/" + std::to_string(id_dec), 10);
      publishers_[id_dec] = pub;
      it = publishers_.find(id_dec);
      RCLCPP_INFO(get_logger(), "Auto-advertised pressure/board/%d", id_dec);
    }

    std_msgs::msg::UInt8MultiArray msg;
    msg.data.assign(ch.begin(), ch.end());
    it->second->publish(msg);

    RCLCPP_DEBUG(get_logger(), "ID 0x%02X -> [%u %u %u %u %u %u %u %u]",
                 id, ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7]);
  }

  // Params
  std::string serial_port_;
  int baud_rate_;
  int command_delay_ms_;
  std::vector<int64_t> board_ids_;

  // Serial
  int fd_;
  std::atomic<bool> running_;
  std::unique_ptr<std::thread> send_thread_;
  std::unique_ptr<std::thread> read_thread_;

  // Publishers per board id
  std::map<int, rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr> publishers_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PressureSerialNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}