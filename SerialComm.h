#ifndef SRC_SERIAL_COMM_H_
#define SRC_SERIAL_COMM_H_

#include "IComm.h"

#include <string.h>

/**
 * @brief 
 * 
 */
class SerialComm : public IComm
{
public:
   SerialComm();

   virtual ~SerialComm();

   bool connect(const std::string &device, uint32_t baud, bool loop) override;
   
   void disconnect() override;

   ssize_t read(char *data, size_t max_len, long timeout) override;

   ssize_t write(const char *data, size_t len, long timeout_ms) override;

   bool is_connected() override;

   // ------------------------------------ //

   speed_t baud_rate(uint32_t baud_rate) override;

protected:
   /**
    * @brief Verifies if if possible to read of write into the stored file
    * descriptor.
    * 
    * @tparam poll_type intended operation
    * @param timeout_ms time to wait device to be available
    * @return true operation is available
    * @return false otherwise
    */
   template <PollType poll_type> 
   bool is_rw_avail(long timeout_ms);
};

#endif /* SRC_SERIAL_COMM_H_ */