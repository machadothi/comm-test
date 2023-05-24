#ifndef SRC_LOOP_BACK_H_
#define SRC_LOOP_BACK_H_

#include <mutex>

#include "IComm.h"

class LoopBack {
public:
    LoopBack();

    ~LoopBack();

    void sender(IComm_ptr &serial);

    void receiver(IComm_ptr &serial, std::mutex &mtx_mon);

    void verbose(std::mutex &mtx_mon);

private:
    bool compare(char right, char left);

    unsigned int calc_byte_rate(unsigned int current_byte_cout,
                                unsigned int last_byte_count, int time_rate);

    float calc_fail_rate(unsigned int fail_bytes, unsigned int total_bytes);


    struct bus_monitor {
        unsigned char first = 0;
        unsigned char last = 0;
        unsigned int fail_counter = 0;
        unsigned int rcv_counter = 0;
        unsigned int tx_counter = 0;
        unsigned int rx_buff_size = 0;
        unsigned int tx_buff_size = 0;
        bool begin = true;
        std::chrono::milliseconds writer_duration;
        std::chrono::milliseconds reader_duration;
        std::chrono::milliseconds monitor_duration;
    } bus_monitor_flags;

};

#endif //SRC_LOOP_BACK_H_