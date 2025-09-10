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
      std::array<uint8_t, 6> req = {{0xAA, 0xC1, static_cast<uint8_t>(id), 0x00, 0x21, 0x55}};
      write(serial_fd_, req.data(), req.size());
      std::this_thread::sleep_for(100ms);

      // 【2】レスポンス受信
      uint8_t buf[16] = {};
      int n = read(serial_fd_, buf, sizeof(buf));
      if (n < 12 || buf[0] != 0xAA || buf[1] != 0xC8) {
        RCLCPP_WARN(this->get_logger(), "Invalid response from ID=0x%02X (%d bytes)", id, n);
        continue;
      }

      // 【3】パース
      int ch1 = buf[4];
      int ch2 = buf[5];
      int ch3 = buf[6];
      int ch4 = buf[7];
      int ch5 = buf[8];
      int ch6 = buf[9];
      int ch7 = buf[10];
      int ch8 = buf[11];

      for(int i=0; i<12; i++){
        std::cout << std::hex << (int)buf[i] << std::endl;
      }
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