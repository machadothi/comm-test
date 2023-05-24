#include "SerialComm.h"

#include <iomanip>
#include <iostream>
#include<cstring>
#include <cerrno>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define TIOCM_LOOP      0x8000

SerialComm::SerialComm() {}

//------------------------------------------------------------------------------

SerialComm::~SerialComm() {}

//------------------------------------------------------------------------------

bool SerialComm::connect(const std::string &device, uint32_t baud, bool loop)
{
    m_dev = device;
    m_baud_rate = baud_rate(baud);
    m_sw_loop = loop;

    m_file_descriptor = ::open(m_dev.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

    fcntl(m_file_descriptor, F_SETFL, 0);

    if (m_file_descriptor == -1) {
        return false;
    } else {
        if (ioctl(m_file_descriptor, TIOCEXCL) != 0) {
            return false;
        }
        if (tcflush(m_file_descriptor, TCIOFLUSH) != 0) {
            return false;
        }
        
        int pins;
        ioctl(m_file_descriptor, TIOCMGET, &pins);
        
        if (m_sw_loop)
            pins |= TIOCM_LOOP;
        else
            pins &= ~TIOCM_LOOP;
        
        ioctl(m_file_descriptor, TIOCMSET, &pins);
        
        
        struct termios st;
        if (tcgetattr(m_file_descriptor, &st) != 0) {
            return false;
        }
        
        st.c_cflag &= ~PARENB;
        st.c_cflag &= ~CSTOPB;
        st.c_cflag &= ~CSIZE;
        st.c_cflag |= CS8;
        
        // ignores modem lines and enables receiver
        st.c_cflag |= (CLOCAL | CREAD);
        
        // disable software flow control
        st.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        // disable conversao do LF e CR
        st.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        
        // disable canonical input
        st.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        // raw output
        st.c_oflag &= ~OPOST;
        
        st.c_cc[VMIN] = 1;
        st.c_cc[VTIME] = 1;
        
        // One-shot autobauding enabled by default on both modules.
        // If disabled, it default back to 115200, see TOBY-L2 SysIntegrManual and
        // SATA-G3-U2_SysIntegrManual
        if (cfsetispeed(&st, m_baud_rate) != 0) {
            //   return Err(err_str("cfsetispeed() error", errno));
            return false;
        }
        if (cfsetospeed(&st, m_baud_rate) != 0) {
            //   return Err(err_str("cfsetospeed() error", errno));
            return false;
        }
        if (tcsetattr(m_file_descriptor, TCSANOW, &st) != 0) {
            //   return Err(err_str("tcsetattr() error", errno));
            return false;
        }
    }

    return true;

}

//------------------------------------------------------------------------------

void SerialComm::disconnect()
{
   if (m_file_descriptor >= 0) {
      // Close port
      close(m_file_descriptor);
      m_file_descriptor = -1;
   }
}

//------------------------------------------------------------------------------

bool SerialComm::is_connected()
{
   return (m_file_descriptor != -1);
}

//------------------------------------------------------------------------------

template <SerialComm::PollType poll_type>
bool SerialComm::is_rw_avail(long timeout_ms)
{
  auto pfd = ::pollfd{
         m_file_descriptor, poll_type | POLLPRI | POLLERR | POLLHUP, 0
      };

   switch (::poll(&pfd, 1, timeout_ms)) {
      case -1: // poll failure
         break;
      case 0: // no new data but can be EOF or timeout
         break;
      default: // some data or error flag available
      {
         if (pfd.revents & (POLLERR | POLLNVAL | POLLHUP)) {
            break;
         } else if (pfd.revents & poll_type) {
            return true;
         } else {
            // will never end up here
         }
      }
   }
  return false;
}

//------------------------------------------------------------------------------

ssize_t SerialComm::read(char *data, size_t max_len, long timeout) {
   if (is_rw_avail<PollType::PT_READ>(timeout)) {
      return ::read(m_file_descriptor, data, max_len);
   } else {
      return 0;
  }
}

//------------------------------------------------------------------------------

ssize_t SerialComm::write(const char *data, size_t len, long timeout_ms) {
   if (is_rw_avail<PollType::PT_WRITE>(timeout_ms)) {
      ssize_t bytes_written =  ::write(m_file_descriptor, data, len);

      if (bytes_written < 0) {
         std::cout << __PRETTY_FUNCTION__ << "|" << __LINE__  
            << "| Error sendBinary m_fd_serial " << m_file_descriptor 
            << " errno = " << errno << " " << std::to_string(errno);
      }

      return bytes_written;
   }

   return 0;
}

//------------------------------------------------------------------------------

speed_t SerialComm::baud_rate(uint32_t baud_rate)
{
   switch (baud_rate) {
      case 4800:
         m_baud_rate = B4800;
         break;
      case 9600:
         m_baud_rate = B9600;
         break;
      case 19200:
         m_baud_rate = B19200;
         break;
      case 38400:
         m_baud_rate = B38400;
         break;
      case 57600:
         m_baud_rate = B57600;
         break;
      case 115200:
         m_baud_rate = B115200;
         break;
   }

   return m_baud_rate;
}

//------------------------------------------------------------------------------