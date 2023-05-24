
#ifndef SRC_ICOMM_H_
#define SRC_ICOMM_H_

#include <string>
#include <memory>

#include <poll.h>
#include <string.h>
#include <termios.h>

/**
 * @brief This is an interface to communication devices. In the GNSS project 
 * will be used to create the UART, USB and SPI concrete classes that 
 * communicates with the GNSS module.
 * 
 */
class IComm
{
public:

   IComm() :
      m_file_descriptor(-1),
      m_baud_rate(B38400) {}

   virtual ~IComm() {}

   /**
    * @brief opens a given communication device and stores its file descriptor
    * in the corresponding member variable.
    * 
    * @param device_name device to the opened. e.g. /dev/ttyUSB0
    * @return true if the device is successfully initializated.
    * @return false otherwise.
    */
   virtual bool connect(const std::string &device, uint32_t baud, 
      bool loop) = 0;

   /**
    * @brief closes the file descriptor stored in the m_file_descriptor variable
    * 
    */
   virtual void disconnect() = 0;

   /**
    * @brief reads data from the stored file descriptor into a given buffer
    * depending on the device availability.
    * 
    * @param data input buffer
    * @param max_len input buffer size
    * @param timeout time to wait device availability
    * @return ssize_t number of bytes read. -1 if error occur
    */
   virtual ssize_t read(char *data, size_t max_len, long timeout) = 0;

   /**
    * @brief writes data into the stored file descriptor from a given buffer
    * depending on the device availability.
    * 
    * @param data output buffer
    * @param max_len output buffer size
    * @param timeout time to wait device availability
    * @return ssize_t number of bytes written. -1 if error occur
    */
   virtual ssize_t write(const char *data, size_t len, long timeout_ms) = 0;

   /**
    * @brief verifies if there's any opened device.
    * 
    * @return false if m_file_descriptor equals -1
    * @return true otherwise
    */
   virtual bool is_connected() = 0;

   /**
    * @brief stores a given baud rate.
    * 
    * @param baud_rate baud rate value in decimal representation.
    * @return speed_t baud rate value in the termios.h representation.
    */
   virtual speed_t baud_rate(uint32_t baud_rate) = 0;

   typedef std::unique_ptr<IComm> ptr_t;

protected:
   /**
    * @brief possible devices operations. Used for verifying device 
    * availability.
    * 
    */
   enum PollType : short int { PT_READ = POLLIN, PT_WRITE = POLLOUT };

   int m_file_descriptor;

   speed_t m_baud_rate;

   std::string m_dev;

   bool m_sw_loop;
};

typedef IComm::ptr_t IComm_ptr;

#endif /* SRC_ICOMM_H_ */