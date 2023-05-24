#include <fcntl.h>
#include <sys/ioctl.h>

#include "SerialModule.h"

#define TIOCM_LOOP      0x8000

SerialModule::SerialModule(const std::string &device, 
  speed_t baudrate, bool sw_loop)
    : m_dev(device),
      m_bd(baudrate),
      m_sw_loop(sw_loop) 
{
  m_fd = -1;
}

SerialModule::~SerialModule() { close(); }

bool SerialModule::open() {
    if (close()) {
        
        int pins;
        
        m_fd = ::open(m_dev.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
        
        fcntl(m_fd, F_SETFL, 0);
        
        if (m_fd == -1) {
            return false;
        } else {
            if (ioctl(m_fd, TIOCEXCL) != 0) {
                return false;
            }
            if (tcflush(m_fd, TCIOFLUSH) != 0) {
                return false;
            }
            
            
            ioctl(m_fd, TIOCMGET, &pins);
            
            if (m_sw_loop)
                pins |= TIOCM_LOOP;
            else
                pins &= ~TIOCM_LOOP;
            
            ioctl(m_fd, TIOCMSET, &pins);
            
            
            struct termios st;
            if (tcgetattr(m_fd, &st) != 0) {
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
            if (cfsetispeed(&st, m_bd) != 0) {
                //   return Err(err_str("cfsetispeed() error", errno));
                return false;
            }
            if (cfsetospeed(&st, m_bd) != 0) {
                //   return Err(err_str("cfsetospeed() error", errno));
                return false;
            }
            if (tcsetattr(m_fd, TCSANOW, &st) != 0) {
                //   return Err(err_str("tcsetattr() error", errno));
                return false;
            }
        }
        
        return true;
    }
    return false;
}

bool SerialModule::close() {

  if (m_fd > 0) {
    if (::close(m_fd) != 0)
      return false;
  }
  return true;
}

ssize_t SerialModule::write(const char *data, size_t len) {
  return ::write(m_fd, data, len);
}

ssize_t SerialModule::write(const char *data, size_t len, long timeout) {
  if (is_rw_avail<PollType::PT_WRITE>(timeout)) {
    return ::write(m_fd, data, len);
  } else {
    return 0;
  }
}

template <typename T> ssize_t SerialModule::write(T &data) {
  return ::write(m_fd, data.c_str(), data.size());
}

template <typename T> ssize_t SerialModule::write(T &data, long timeout) {
  if (is_rw_avail<PollType::PT_WRITE>(timeout)) {
    return ::write(m_fd, data.c_str(), data.size());
  } else {
    return 0;
  }
}

ssize_t SerialModule::read(char *data, size_t max_len) {
  unsigned iOffs = 0;
  int iTemp = 0;

  do // le serial ate sair por timeout ou por num de bytes lidos
  {
    iTemp = ::read(m_fd, data + iOffs, max_len - iOffs);

    if (iTemp)
      iOffs += iTemp;

    if (iOffs >= max_len)
      break;

  } while (iTemp);

  return iOffs;
//   return ::read(m_fd, data, max_len);
}

ssize_t SerialModule::read(char *data, size_t max_len, long timeout) {
  if (is_rw_avail<PollType::PT_READ>(timeout)) {
    return ::read(m_fd, data, max_len);
  } else {
    return 0;
  }
}

template <SerialModule::PollType poll_type>
bool SerialModule::is_rw_avail(long timeout_ms) {
  auto pfd = ::pollfd{m_fd, poll_type | POLLPRI | POLLERR | POLLHUP, 0};

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