#ifndef SRC_COMM_FACTORY_H_
#define SRC_COMM_FACTORY_H_

#include <memory>

#include "IComm.h"

/**
 * @brief List of available communication devices. Should be used to instantiate
 * a new device.
 * 
 * 
 */
enum class CommDevice
{
   UNKNOWN,
   UART,
   USB,
   SPI,
};

/**
 * @brief Create a comm device object.
 * 
 * @param comm_type desired communication device to be instatiated.
 * @return IComm_ptr IComm interface instance.
 */
IComm_ptr create_comm_device(CommDevice comm_type);

#endif /* SRC_COMM_FACTORY_H_ */