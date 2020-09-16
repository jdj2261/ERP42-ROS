#ifndef SERIAL_NODE_H
#define SERIAL_NODE_H

#include <SerialPort/serial_port.h>

// command
typedef struct _pc_to_erp42
{
  unsigned char S = 0x53;
  unsigned char T = 0x54;
  unsigned char X = 0x58;
  uint8_t MorA = 0x00;  // Manual : 0x00, Auto : 0x01
  uint8_t E_stop = 0x00;
  uint8_t gear = 0x01;
  union speed{uint8_t speed[2]; uint16_t _speed;};  union speed speed;
  union steer{char steer[2]; short _steer;};  union steer steer;
  uint8_t brake;
  uint8_t alive;
  unsigned char ETX0 = 0x0D;
  unsigned char ETX1 = 0x0A;
}PC2ERP;

const uint8_t SPEED_FACTOR=10;
const int8_t STEER_FACTOR=71;

namespace unmansol
{
namespace erp42
{
class ERP42Serial
{
public:
  ERP42Serial(const char *device_name, int serial_baudrate);

  ~ERP42Serial()
  {
    // close the device
//    delete [] m_read_data;
    serial_port_.Close();
    std::cout << " Serial Communication Finished... " << std::endl;
  }

  void Init_node();
  void Init_data();
  void Read();
  void Write();
  void Update(unsigned char (&buffer)[14]);
  void Update();

  // Callback (ROS)
//  void CmdCtrlMsgCallback(const erp42_msgs::CmdControl::Ptr &msg);
  void DriveCallback(const erp42_msgs::DriveCmd::Ptr &msg);
  void ModeCallback(const erp42_msgs::ModeCmd::Ptr &msg);

private:
  SerialPort serial_port_;
  ros::NodeHandle m_nh;
  ros::Publisher m_pub_feedback;
  ros::Publisher m_pub_cmdcontrol;
  ros::Subscriber m_sub_mode;
  ros::Subscriber m_sub_drive;
  erp42_msgs::SerialFeedBack m_feedback_msg;
  erp42_msgs::CmdControl m_cmdcontrol_msg;

  unsigned char m_read_data[18];


protected:
  PC2ERP m_pc2erp;

  uint8_t m_AlvCnt;

}; // class SerialPort
} // namespace serial
} // namespace unmansol

#endif // SERIAL_NODE_H
