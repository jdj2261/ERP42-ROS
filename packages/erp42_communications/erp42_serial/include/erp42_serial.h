#pragma once

#include "SerialPort/serial_port.h"
#include <memory>
#include <thread>


namespace unmansol
{
namespace erp42
{
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

    enum class MorA
    {
        MANUAL=0,
        AUTO=1,
    };

    constexpr uint8_t SPEED_FACTOR{10};
    constexpr uint8_t STEER_FACTOR{71};
    constexpr int8_t MINUS_STEER_FACTOR{-71};
    constexpr uint8_t NUM_WRITE{14};
    constexpr uint8_t NUM_READ{18};

    class ERP42Serial
    {
    public:
        explicit ERP42Serial() = default;
        explicit ERP42Serial(const char *device_name, int serial_baudrate);
        ~ERP42Serial() noexcept
        {
            serial_port_->Close();
            std::cout << "Serial Communication Finished... " << std::endl;
        }

        bool Open();
        bool Read();
        void Write();
        void Run();

    private:
        std::shared_ptr<SerialPort> serial_port_;
        ros::NodeHandle m_nh;
        ros::Rate m_loop;
        ros::Publisher m_pub_feedback;
        ros::Publisher m_pub_cmdcontrol;
        ros::Subscriber m_sub_mode;
        ros::Subscriber m_sub_drive;
        erp42_msgs::SerialFeedBack m_feedback_msg;
        erp42_msgs::CmdControl m_cmdcontrol_msg;

        unsigned char m_read_data[NUM_READ];
        PC2ERP m_pc2erp;
        uint8_t m_AlvCnt;

        void InitData();
        void UpdateNode();
        void writeUpdate(unsigned char (&buffer)[NUM_WRITE]);
        void readUpdate();

        // Callback (ROS)
        void DriveCallback(const erp42_msgs::DriveCmd::Ptr &msg);
        void ModeCallback(const erp42_msgs::ModeCmd::Ptr &msg);


    }; // class SerialPort
} // namespace serial
} // namespace unmansol
