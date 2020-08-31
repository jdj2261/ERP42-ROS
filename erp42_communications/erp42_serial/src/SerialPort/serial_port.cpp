#include <SerialPort/serial_port.h>

using namespace std;
using namespace unmansol::erp42;

SerialPort::SerialPort(const char *device_name, int baudrate){
  readBuffer.reserve(defaultReadBufferSize);
  Open(device_name, baudrate);
}


void SerialPort::Open(const char *device_name, int baudrate){
  if (!strlen(device_name)){
    cout << "Device path has not been assigned" << endl;
    return;
  }
  fd = open(device_name,  O_RDWR| O_NOCTTY);
  //O_RDWR-Read/Write access to Serial port,  O_NOCTTY -No terminal control
  if (fd == -1)
    cout << "\n Error in Opening device -"<< strerror(errno) << endl;
  else
    cout << "\n Device Opened Successfully" <<endl;

  Configure(baudrate);
}

void SerialPort::Close(){
  if(fd != -1){
    auto retVal = close(fd);
    if(retVal != 0){
      cout << "Tried to close serial port, but close() failed " << endl;
      return;
    }
    fd = -1;
  }
}

void SerialPort::Configure(int baudrate){
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if(tcgetattr(fd, &tty) != 0)
    cout << "Could not get terminal attributes - " << strerror(errno) << endl;


  tty.c_cflag &= ~PARENB;   //  No Parity  bit
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size
  tty.c_cflag |=  CS8;      // Set the data bits = 8
  tty.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
  tty.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
  tty.c_iflag &= ~(BRKINT | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

  tty.c_oflag &= ~OPOST;//No Output Processing

  switch (baudrate)
  {
  case 9600:
    cfsetispeed(&tty,B9600); // Set Read  Speed as 9600                       */
    cfsetospeed(&tty,B9600); // Set Write Speed as 9600
    break;

  case 115200:
    cfsetispeed(&tty,B115200); // Set Read  Speed as 9600                       */
    cfsetospeed(&tty,B115200); // Set Write Speed as 9600
    break;

  default:
    std::cout << " None.. " << std::endl;
    break;
  }

  tty.c_cc[VMIN] = 36; // Read at least 36 characters
  tty.c_cc[VTIME] = 1; // Wait 0.1sec

  if((tcsetattr(fd,TCSANOW,&tty)) != 0) //Set the attributes
    cout << " Error in Setting attributes - " << strerror(errno) << endl;
  else
    cout << "BaudRate : " << baudrate << " StopBits = 1, Parity = None" << endl;
}

void SerialPort::Read(unsigned char* rpacket, int packetsize){

  if (fd == 0)
  {
    cout << "Read was called but file descriptor was 0, file has not been opened" << endl;
    return;
  }
  std::string sName(reinterpret_cast<char*>(rpacket));

  if (sName.length() == 0)
    cout << "No data!" <<endl;
  else
  {
    ssize_t n = read(fd, rpacket, packetsize);
    if(n != packetsize)
    {
      cout << "read error!" << endl;
    }
    return;
  }
}
void SerialPort::Read(string& data)
{
  data.clear();
  if (fd == 0){
    cout << "Read was called but file descriptor was 0, file has not been opened" << endl;
    return;
  }
  ssize_t n = read(fd, &readBuffer[0], defaultReadBufferSize);
  data = string(&readBuffer[0], n);
}

void SerialPort::Write(unsigned char*  wpacket, int packetsize)
{
  if (fd == 0)
  {
    cout << "Write was called but file descriptor was 0, file has not been opened" << endl;
    return;
  }
  int writeResult = write(fd, wpacket, packetsize);
  if(writeResult == -1){
    cout << "Write Packet failed" << endl;
  }
}

void SerialPort::Write(const string& data)
{
  if (fd == 0)
  {
    cout << "Write was called but file descriptor was 0, file has not been opened" << endl;
    return;
  }
  int writeResult = write(fd, data.c_str(), data.size());
  if(writeResult == -1){
    cout << "Write failed" << endl;
  }
}
