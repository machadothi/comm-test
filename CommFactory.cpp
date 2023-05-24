#include <iostream>
#include <string>

#include "SerialComm.h"
#include "CommFactory.h"

#include <termios.h>

IComm_ptr create_comm_device(CommDevice comm_type)
{
   switch (comm_type)
   {
      case CommDevice::UNKNOWN:
         break;

      case CommDevice::UART:
         {
            return std::unique_ptr<IComm>{new SerialComm()};
         }
      case CommDevice::USB:
         // add case for USB
         break;

      case CommDevice::SPI:
         // add case for SPI
         break;
   }

   return nullptr;
}