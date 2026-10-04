#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <exception>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge() : Node("serial_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 9600);

    button_pub_ = create_publisher<std_msgs::msg::String>("/arduino/button", 10);
    front_distance_pub_ =
      create_publisher<std_msgs::msg::Int32>("/arduino/front_distance_cm", 10);
    rear_distance_pub_ =
      create_publisher<std_msgs::msg::Int32>("/arduino/rear_distance_cm", 10);
    // 예전엔 HALL:이 상태 변화 시에만 오는 edge-triggered 값이라 transient_local로
    // 늦게 붙는 구독자에게 마지막 상태를 즉시 전달했는데, CycloneDDS 환경에서
    // transient_local late-join 재전달이 간헐적으로 실패해 GUI가 값을 아예 못
    // 받는 문제가 있었다. 지금은 아두이노가 변화 없어도 1초마다 HALL:을
    // 재전송하므로(all_in_one_module.ino updateHall() 참고) durability를 DIST/
    // DIST_REAR와 같은 기본(volatile) QoS로 맞춰 그 실패 지점을 없앤다.
    payload_pub_ = create_publisher<std_msgs::msg::Bool>("/arduino/payload_loaded", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/arduino/status", rclcpp::QoS(10).transient_local());

    buzzer_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/arduino/buzzer", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        sendCommand(msg->data ? "BUZZER:1" : "BUZZER:0");
      });

    // 통합 펌웨어가 상태별 점등/점멸을 직접 처리하므로 상태 이름을 그대로 전달한다.
    // 예: IDLE -> LED:IDLE, RUN -> LED:RUN
    led_sub_ = create_subscription<std_msgs::msg::String>(
      "/arduino/led", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.empty()) {
          RCLCPP_WARN(get_logger(), "빈 LED 명령은 전송하지 않습니다.");
          return;
        }
        std::string state =
          msg->data.rfind("LED:", 0) == 0 ? msg->data.substr(4) : msg->data;

        if (
          state != "IDLE" && state != "RUN" && state != "ARRIVE" &&
          state != "STOP" && state != "ERROR" && state != "DOCKED")
        {
          RCLCPP_WARN(get_logger(), "지원하지 않는 LED 상태: %s", state.c_str());
          return;
        }

        sendCommand("LED:" + state);
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&SerialBridge::readSerial, this));
    // 아두이노 재부팅·USB 재연결 시 노드를 다시 띄우지 않고 스스로 복구한다.
    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&SerialBridge::tryConnect, this));
    tryConnect();
  }

  ~SerialBridge() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void tryConnect()
  {
    if (fd_ >= 0) {
      return;
    }
    if (!openSerial(port_, baud_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "시리얼 포트 열기 실패: %s (%s) — 1초마다 재시도합니다.", port_.c_str(),
        std::strerror(errno));
      return;
    }
    rx_buffer_.clear();
    RCLCPP_INFO(get_logger(), "시리얼 포트 연결됨: %s (%d baud)", port_.c_str(), baud_);
  }

  void handleDisconnect(const char * reason)
  {
    RCLCPP_ERROR(get_logger(), "시리얼 연결 끊김 (%s): 재연결을 시도합니다.", reason);
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    publishStatus("DISCONNECTED");
  }

  bool openSerial(const std::string & port, int baud)
  {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      const int saved_errno = errno;
      close(fd_);
      fd_ = -1;
      errno = saved_errno;
      return false;
    }

    const speed_t speed = (baud == 115200) ? B115200 : B9600;
    if (baud != 9600 && baud != 115200) {
      RCLCPP_WARN(get_logger(), "지원하지 않는 baud %d: 9600을 사용합니다.", baud);
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const int saved_errno = errno;
      close(fd_);
      fd_ = -1;
      errno = saved_errno;
      return false;
    }
    return true;
  }

  void sendCommand(const std::string & command)
  {
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "시리얼 미연결 — 명령 폐기: %s", command.c_str());
      return;
    }
    const std::string wire_data = command + "\n";
    // 논블로킹 fd라 한 번에 다 못 쓰거나 EAGAIN이 날 수 있다. 전송 버퍼가 비기를
    // 잠깐 기다리며 끝까지 보낸다.
    std::size_t sent = 0;
    for (int attempt = 0; sent < wire_data.size() && attempt < 20; ++attempt) {
      const ssize_t n = write(fd_, wire_data.data() + sent, wire_data.size() - sent);
      if (n > 0) {
        sent += static_cast<std::size_t>(n);
        continue;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        handleDisconnect(std::strerror(errno));
        return;
      }
      pollfd pfd{fd_, POLLOUT, 0};
      poll(&pfd, 1, 5);
    }
    if (sent != wire_data.size()) {
      RCLCPP_ERROR(
        get_logger(), "시리얼 전송 실패 (%s): %zu/%zu 바이트만 전송", command.c_str(), sent,
        wire_data.size());
      return;
    }
    RCLCPP_INFO(get_logger(), "시리얼 명령 전송: %s", command.c_str());
  }

  void readSerial()
  {
    if (fd_ < 0) {
      return;
    }
    char buffer[256];
    const ssize_t count = read(fd_, buffer, sizeof(buffer));
    if (count < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        handleDisconnect(std::strerror(errno));
      }
      return;
    }

    for (ssize_t i = 0; i < count; ++i) {
      const char c = buffer[i];
      if (c == '\n') {
        if (!rx_buffer_.empty()) {
          processLine(rx_buffer_);
          rx_buffer_.clear();
        }
      } else if (c != '\r') {
        rx_buffer_ += c;
        if (rx_buffer_.size() > 512) {
          RCLCPP_WARN(get_logger(), "너무 긴 시리얼 입력을 폐기합니다.");
          rx_buffer_.clear();
        }
      }
    }
  }

  bool parseInt(const std::string & text, int32_t & value) const
  {
    char * end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int32_t>::min() ||
      parsed > std::numeric_limits<int32_t>::max())
    {
      return false;
    }
    value = static_cast<int32_t>(parsed);
    return true;
  }

  void publishStatus(const std::string & value)
  {
    std_msgs::msg::String msg;
    msg.data = value;
    status_pub_->publish(msg);
  }

  void processLine(const std::string & line)
  {
    if (line == "READY") {
      publishStatus("READY");
      RCLCPP_INFO(get_logger(), "아두이노 준비됨 (READY)");
      return;
    }

    if (line == "BTN_01" || line == "BTN_02" || line == "BTN_03") {
      std_msgs::msg::String msg;
      msg.data = line;
      button_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "버튼 입력: %s", line.c_str());
      return;
    }
    // 단일 버튼 펌웨어의 BTN:1(눌림)을 E-stop 버튼 BTN_01로 변환한다.
    // BTN:0(뗌)은 토글이 두 번 발생하지 않도록 발행하지 않는다.
    if (line == "BTN:1") {
      std_msgs::msg::String msg;
      msg.data = "BTN_01";
      button_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "버튼 입력: BTN_01");
      return;
    }
    if (line == "BTN:0") {
      return;
    }

    if (line.rfind("DIST_REAR:", 0) == 0) {
      publishInteger(line, "DIST_REAR:", rear_distance_pub_);
      return;
    }
    if (line.rfind("DIST:", 0) == 0) {
      publishInteger(line, "DIST:", front_distance_pub_);
      return;
    }

    if (line == "HALL:0" || line == "HALL:1") {
      std_msgs::msg::Bool msg;
      msg.data = (line == "HALL:1");
      payload_pub_->publish(msg);
      RCLCPP_DEBUG(get_logger(), "적재 감지: %s", msg.data ? "적재됨" : "비어 있음");
      return;
    }

    if (line.rfind("LED:", 0) == 0) {
      publishStatus("LED_ACK:" + line.substr(4));
      RCLCPP_INFO(get_logger(), "LED 상태 응답: %s", line.substr(4).c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "알 수 없는 시리얼 입력: %s", line.c_str());
  }

  void publishInteger(
    const std::string & line, const std::string & prefix,
    const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr & publisher)
  {
    int32_t value = 0;
    if (!parseInt(line.substr(prefix.size()), value)) {
      RCLCPP_WARN(get_logger(), "잘못된 센서 값: %s", line.c_str());
      return;
    }
    std_msgs::msg::Int32 msg;
    msg.data = value;
    publisher->publish(msg);
    RCLCPP_DEBUG(get_logger(), "%s %d cm", prefix.c_str(), value);
  }

  std::string port_;
  int baud_ = 9600;
  int fd_ = -1;
  std::string rx_buffer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr front_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rear_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr buzzer_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<SerialBridge>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_bridge"), "치명적 오류로 종료: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
