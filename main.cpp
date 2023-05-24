#include <mutex>
#include <string>
#include <thread>

#include "CommFactory.h"
#include "LoopBack.h"

namespace {

    bool sw_loop = false;
    std::string device{};
    uint32_t bd = 0;

    void parse_args(int &argc, char **argv) {
        device = argv[1];
        bd = std::stoi(argv[2]);
        if (argc >= 4) {
            sw_loop = bool(std::stoi(argv[3]));
        }
    }
}

//---------------------------------------------------------------------------------
// argv[1] = device name e.g. /dev/ttyUSB0
// argv[2] = baud rate e.g. 115200
// argv[3] = enable/disable software loop. Disable=(0 or no param) | Enable=1

int main(int argc, char **argv) {
    parse_args(argc, argv);

    auto serial = create_comm_device(CommDevice::UART);
    serial->connect(device.c_str(), bd, sw_loop);

    std::shared_ptr<LoopBack> loop = std::make_shared<LoopBack>();

    std::mutex mon_mtx;

    auto sender = [&serial, &loop]() {
        loop->sender(serial);
    };
    auto receiver = [&serial, &loop, &mon_mtx]() {
        loop->receiver(serial, mon_mtx);
    };
    auto mon = [&loop, &mon_mtx]() {
        loop->verbose(mon_mtx);
    };

    std::thread writer(sender);
    std::thread reader(receiver);
    std::thread monitor(mon);

    if (writer.joinable())
        writer.join();

    if (reader.joinable())
        reader.join();

    if (monitor.joinable())
        monitor.join();

    return 0;
}