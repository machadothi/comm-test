#ifndef SERIAL_MODULE_H_
#define SERIAL_MODULE_H_

#include <iostream>
#include <string>
#include <vector>

#include <termios.h>
#include <unistd.h>
#include <poll.h>

class SerialModule {
public:
  enum PollType : short int { PT_READ = POLLIN, PT_WRITE = POLLOUT };

  SerialModule(const std::string &device, speed_t baudrate, bool sw_loop);
  ~SerialModule();

  bool open();
  bool close();

  ssize_t write(const char *data, size_t len);
  ssize_t write(const char *data, size_t len, long timeout);

  template <typename T>
  ssize_t write(T &data);

  template <typename T>
  ssize_t write(T &data, long timeout);

  // document possible returns
  ssize_t read(char *data, size_t max_len);
  ssize_t read(char *data, size_t max_len, long timeout);

  template <SerialModule::PollType poll_type> 
  bool is_rw_avail(long timeout_ms);

private:
  int m_fd;
  std::string m_dev;
  speed_t m_bd;
  bool m_sw_loop;
};
#endif // SERIAL_MODULE_H_