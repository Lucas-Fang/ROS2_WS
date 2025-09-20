#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <cmath>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <string>
#include "crc16.h"

uint16_t Get_CRC16(const uint8_t *ptr, uint16_t len);
float combine_bytes(const uint8_t *data);
bool parse_data(const uint8_t *data, size_t length);
float uint_to_float(int x_int, float x_min, float x_max, int bits);


class Serial_Node : public rclcpp::Node
{
public:
  Serial_Node() : Node("serial_node_cpp")
  {
    // 等设备准备好再初始化
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

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

    async_receive_message();
  }

private:
  void async_receive_message()  //创建一个函数更方便重新调用
  {
    auto port = serial_driver_->port();

    // 设置接收回调函数
    port->async_receive([this](const std::vector<uint8_t> &data,const size_t &size) 
    {
        if (size > 0)
        {
            // 处理接收到的数据
            std::string received_message(data.begin(), data.begin() + size);
            // RCLCPP_INFO(this->get_logger(), "Received: %s (%ld bytes)", received_message.c_str(),size);
          parse_data(data.data(),19);
          }
        // 继续监听新的数据
        async_receive_message();
    }
    );
  }

  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  std::vector<uint8_t> receive_data_buffer = std::vector<uint8_t>(1024); // 接收缓冲区
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<Serial_Node>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


uint16_t Get_CRC16(const uint8_t *ptr, uint16_t len)
{
uint16_t crc = 0xFFFF;
for (size_t i = 0; i < len; ++i)
{
uint8_t index = (crc >> 8 ^ ptr[i]);
crc = ((crc << 1) ^ CRC16_table[index]);
}
return crc;
}



float combine_bytes(const uint8_t *data)
{
    // 将 4 字节拼成一个 int32_t，低位在前（小端）
    
    
    int out = (
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] >> 8) |
        ((uint32_t)data[2] >> 16) |
        ((uint32_t)data[3] >> 24));
      return  uint_to_float(out,-pow(2.0, 32.0)/2,pow(2.0, 32.0)/2,32);
}

bool parse_data(const uint8_t *data, size_t length)
{
    if (length != 19)
    {
        std::cerr << "长度错误\n";
        return false;
    }

    if (data[0] == 0x55 || data[1] == 0xAA || data[18] == 0x0A)
    {
        // std::cerr << "帧头或帧尾错误\n";
        // return false;
           uint16_t crc_received = (data[17] << 8) | data[16];
    uint16_t crc_calc = Get_CRC16(data, 16); // 不含 CRC 和结尾

    if (crc_received != crc_calc)
    {
        std::cerr << "CRC 校验失败\n";
        return false;
    }

    float roll  = combine_bytes(&data[4]);
    float pitch = combine_bytes(&data[8]);
    float yaw   = combine_bytes(&data[12]);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "roll:%.2f,pitch:%2f,yaw:%.2f",roll,pitch,yaw);

    return true;
    }
    else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "zhen tou zhen wei cuo wu!\n");
    return false;
    }

 
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/* converts unsigned int to float, given range and number of bits */
float span = x_max - x_min;
float offset = x_min;
return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
