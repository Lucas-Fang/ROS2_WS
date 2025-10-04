#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
// #include <cmath>
#include <rclcpp/logger.hpp>
// #include <string>
#include "crc16.h"

struct IMUData {
    float roll;
    float pitch;
    float yaw;
};

class Serial_Node : public rclcpp::Node
{
public:
  Serial_Node() : Node("serial_node_cpp")
  {
    // 等设备准备好再初始化
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 串口设备名（根据实际设备调整）
    const std::string device_name = "/dev/ttyACM0";

    RCLCPP_INFO(this->get_logger(), "Serial port Node Open!");
    // 创建串口配置对象
    // 波特率115200；不开启流控制；无奇偶效验；停止位1。
    drivers::serial_driver::SerialPortConfig config(
        921600,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);
    
    // 初始化串口
    try
    {
      io_context_ = std::make_shared<drivers::common::IoContext>(1);
      // 初始化 serial_driver_
      serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
      serial_driver_->init_port(device_name, config);
      serial_driver_->port()->open();
      
      RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
      RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
      RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
      return;
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    //         send_imu_restart();

    async_receive_message();
  }

private:
    void send_imu_restart()
    {
        std::vector<uint8_t> restart_cmd = {0xAA, 0x00, 0x00, 0x0D};
        try {
            serial_driver_->port()->send(restart_cmd);
            RCLCPP_INFO(this->get_logger(), "Sent IMU restart command");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send IMU restart command: %s", ex.what());
        }
    }
  //接收函数
  void async_receive_message()  //创建一个函数更方便重新调用
  {//注册回调函数
    auto port = serial_driver_->port();

    // 设置接收回调函数
    port->async_receive([this](const std::vector<uint8_t> &data,const size_t &size) 
    {
        // if (size > 0)
        // {
        //     // 处理接收到的数据
        //     // std::string received_message(data.begin(), data.begin() + size);

        //     std::ostringstream oss;
        //     oss << "Received data (" << size << " bytes): ";
        //     for (size_t i = 0; i < size; i++) {
        //         oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        //             << static_cast<int>(data[i]) << " ";
        //     }
        //     RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        //   }
            recv_buffer_.insert(recv_buffer_.end(), data.begin(), data.begin() + size);
            parse_buffer();
            async_receive_message();
        // 继续监听新的数据
        async_receive_message();
    }
    );
  }


  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  std::vector<uint8_t> receive_data_buffer = std::vector<uint8_t>(1024); // 接收缓冲区
  std::vector<uint8_t> recv_buffer_;


  void parse_buffer()
    {
        while (recv_buffer_.size() >= 19) { // 至少一帧
            // 找帧头
            if (recv_buffer_[0] != 0x55 || recv_buffer_[1] != 0xAA) {
                recv_buffer_.erase(recv_buffer_.begin());
                continue;
            }

            // 取一帧
            std::vector<uint8_t> frame(recv_buffer_.begin(), recv_buffer_.begin() + 19);

            // 校验 CRC16
            uint16_t crc_in_frame = (uint16_t)frame[16] | ((uint16_t)frame[17] << 8);
            uint16_t crc_calc = Get_CRC16(frame.data(), 16);
            if (crc_in_frame == crc_calc) {
                // 只处理 index=0x03 的帧
                if (frame[3] == 0x03) {
                    IMUData imu{};
                    std::memcpy(&imu.roll,  &frame[4], 4);
                    std::memcpy(&imu.pitch, &frame[8], 4);
                    std::memcpy(&imu.yaw,   &frame[12],4);

                    RCLCPP_INFO(this->get_logger(),
                        "IMU RPY: roll=%.2f, pitch=%.2f, yaw=%.2f",
                        imu.roll, imu.pitch, imu.yaw);
                }
            } else {
                // RCLCPP_WARN(this->get_logger(), "CRC mismatch, drop frame");
            }

            // 删除已处理帧
            recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + 19);
        }
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<Serial_Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

