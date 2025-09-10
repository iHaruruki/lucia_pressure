#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <thread>
#include <map>

using namespace std::chrono_literals;

struct Publishers {
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hr;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spo2;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sys_bp;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dia_bp;
};

class VitalSensorNode : public rclcpp::Node {
public:
  VitalSensorNode()
  : Node("vital_sensor_node"), serial_fd_(-1)
  {
    // シリアルポートオープン
    serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
      return;
    }
    // ボーレート設定
    termios tio = {};
    cfsetispeed(&tio, B2000000);
    cfsetospeed(&tio, B2000000);
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tcsetattr(serial_fd_, TCSANOW, &tio);

    // 各センサIDごとのパブリッシャを準備
    for (auto id : {0x0A, 0x0B, 0x0C}) {
      char suffix = id == 0x0A ? 'A' : id == 0x0B ? 'B' : 'C';
      auto &pub = pubs_[id];
      pub.hr = this->create_publisher<std_msgs::msg::Int32>(
        "vital/" + std::string(1, suffix) + "/hr", 10);
      pub.spo2 = this->create_publisher<std_msgs::msg::Float64>(
        "vital/" + std::string(1, suffix) + "/spo2", 10);
      pub.sys_bp = this->create_publisher<std_msgs::msg::Int32>(
        "vital/" + std::string(1, suffix) + "/sys_bp", 10);
      pub.dia_bp = this->create_publisher<std_msgs::msg::Int32>(
        "vital/" + std::string(1, suffix) + "/dia_bp", 10);
    }

    // タイマー：200msごとにセンサをポーリング
    timer_ = this->create_wall_timer(200ms, std::bind(&VitalSensorNode::poll_sensors, this));
  }

  ~VitalSensorNode() override {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  void poll_sensors() {
    for (auto id : {0x0A, 0x0B, 0x0C}) {
      // 【1】リクエスト送信
      std::array<uint8_t, 6> req = {{0xAA, 0xC1, static_cast<uint8_t>(id), 0x00, 0x20, 0x55}};
      write(serial_fd_, req.data(), req.size());
      std::this_thread::sleep_for(100ms);

      // 【2】レスポンス受信
      uint8_t buf[16] = {};
      int n = read(serial_fd_, buf, sizeof(buf));
      if (n < 9 || buf[0] != 0xAA || buf[1] != 0xC5) {
        RCLCPP_WARN(this->get_logger(), "Invalid response from ID=0x%02X (%d bytes)", id, n);
        continue;
      }

      // 【3】パース（spo2は10倍された値で送信される）
      int hr = buf[4];
      uint16_t spo2_raw = static_cast<uint16_t>(buf[5]) | (static_cast<uint16_t>(buf[6]) << 8);
      double spo2 = spo2_raw / 10.0;
      int sys_bp = buf[7];
      int dia_bp = buf[8];

      // デバッグ出力
      RCLCPP_INFO(this->get_logger(),
        "ID=0x%02X: hr=%d, spo2=%.1f, sys_bp=%d, dia_bp=%d",
        id, hr, spo2, sys_bp, dia_bp);

      // 【4】パブリッシュ
      auto &pub = pubs_[id];
      auto msg_hr = std::make_shared<std_msgs::msg::Int32>(); msg_hr->data = hr;
      pub.hr->publish(*msg_hr);

      auto msg_sp = std::make_shared<std_msgs::msg::Float64>(); msg_sp->data = spo2;
      pub.spo2->publish(*msg_sp);

      auto msg_sys = std::make_shared<std_msgs::msg::Int32>(); msg_sys->data = sys_bp;
      pub.sys_bp->publish(*msg_sys);

      auto msg_dia = std::make_shared<std_msgs::msg::Int32>(); msg_dia->data = dia_bp;
      pub.dia_bp->publish(*msg_dia);
    }
  }

  int serial_fd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<uint8_t, Publishers> pubs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VitalSensorNode>());
  rclcpp::shutdown();
  return 0;
}