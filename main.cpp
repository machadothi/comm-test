#include <iomanip>
#include <iostream>
#include <string>

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "SerialComm.h"
#include "CommFactory.h"

//---------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
bool compare(char right, char left) {
  if ((char)(right - left) != (char)1) {
    return false;
  }
  return true;
}
//------------------------------------------------------------------------------
unsigned int calc_byte_rate(unsigned int current_byte_cout,
                            unsigned int last_byte_count, int time_rate) {

  return (current_byte_cout - last_byte_count) / time_rate;
}
//------------------------------------------------------------------------------
float calc_fail_rate(unsigned int fail_bytes, unsigned int total_bytes) {

  return (static_cast<float>(fail_bytes) / static_cast<float>(total_bytes)) * 100;
}

//------------------------------------------------------------------------------
void write_port(IComm_ptr &serial) {
  char data[1024];
  ssize_t write_size = 0;
  auto writer_timer_start = std::chrono::steady_clock::now();
  auto writer_timer_end = std::chrono::steady_clock::now();

  for (unsigned int i = 0; i < sizeof(data); i++) {
    data[i] = i;
  }

  auto safe_write = [&serial, &data]() {
    return serial->write(data, sizeof(data), 1000);
  };

  while (true) {
    if ((write_size = safe_write()) > 0) {
      bus_monitor_flags.tx_counter += write_size;
      bus_monitor_flags.tx_buff_size = write_size;
    } else {
      std::cout << "write timeout" << std::endl;
    }
    writer_timer_end = std::chrono::steady_clock::now();

    bus_monitor_flags.writer_duration = 
    std::chrono::duration_cast<std::chrono::milliseconds>(
      writer_timer_end - writer_timer_start
      );
      
    writer_timer_start =  writer_timer_end;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

//------------------------------------------------------------------------------
void read_port(IComm_ptr &serial, std::mutex &mtx_mon) {

  char buf[1024];
  static ssize_t buff_size;
  auto reader_timer_start = std::chrono::steady_clock::now();
  auto reader_timer_end = std::chrono::steady_clock::now();

  auto safe_read = [&serial, &buf]() {
    // return serial.read(buf, sizeof(buf), 1000);
    return serial->read(buf, sizeof(buf), 1000);
  };

  while (true) {
    buff_size = safe_read();

    if (buff_size <= 0) {
      continue;
    }
    {
      std::lock_guard<std::mutex> guard(mtx_mon);

      bus_monitor_flags.rcv_counter += buff_size;
      bus_monitor_flags.rx_buff_size = buff_size;
      bus_monitor_flags.first = buf[0];

      if (!bus_monitor_flags.begin &&
          !compare(buf[0], bus_monitor_flags.last)) {
        bus_monitor_flags.fail_counter++;
      }

      bus_monitor_flags.begin = false;
      for (int i = 1; i < buff_size; i++) {
        if (!compare(buf[i], buf[i - 1])) {
          bus_monitor_flags.fail_counter++;
        }
      }
      bus_monitor_flags.last = buf[buff_size - 1];
    
      reader_timer_end = std::chrono::steady_clock::now();

      bus_monitor_flags.reader_duration = 
      std::chrono::duration_cast<std::chrono::milliseconds>(
            reader_timer_end - reader_timer_start
            );
    
      reader_timer_start =  reader_timer_end;
    }
  }
}

//------------------------------------------------------------------------------
void bus_monitor(std::mutex &mtx_mon) {

  decltype(bus_monitor_flags.rcv_counter) rcv_counter_last = 0;

  decltype(bus_monitor_flags.tx_counter) tx_counter_last = 0;

  int sleep_time = 1; // seconds


  while (true) {
    {
      std::lock_guard<std::mutex> guard(mtx_mon);

      if (bus_monitor_flags.begin) {
        rcv_counter_last = bus_monitor_flags.rcv_counter;
        tx_counter_last = bus_monitor_flags.tx_counter;
        continue;
      }
      system("clear");
      std::cout << "\nMonitor Thread " << std::endl;

      std::cout << std::left << std::setw(12) << "| Fail Counter " 
                << std::left << std::setw(20) << "| Writer Loop Time (ms)"
                << std::left << std::setw(20) << "| Reader Loop Time (ms)"                      
                << std::left << std::setw(12) << "| Rx Buf Size " 
                << std::left << std::setw(12) << "| Tx Buf Size " 
                << std::left << std::setw(12) << "| Tx Avg Rate " 
                << std::left << std::setw(12) << "| Rx Avg Rate " 
                << std::left << std::setw(12) << "| Lose Byte Rate (%) " 
                << std::left << std::setw(20) << "| Rx Total Counter " 
                << std::endl;

      std::cout << std::right << std::setw(12) << bus_monitor_flags.fail_counter
                << std::right << std::setw(20) << bus_monitor_flags.writer_duration.count()
                << std::right << std::setw(25) << bus_monitor_flags.reader_duration.count()
                << std::right << std::setw(12) << bus_monitor_flags.rx_buff_size
                << std::right << std::setw(15) << bus_monitor_flags.tx_buff_size 
                << std::right << std::setw(15) << calc_byte_rate(
                                                    bus_monitor_flags.tx_counter, 
                                                    tx_counter_last, 
                                                    sleep_time
                                                    )
                << std::right << std::setw(15) << calc_byte_rate(
                                                    bus_monitor_flags.rcv_counter, 
                                                    rcv_counter_last, 
                                                    sleep_time
                                                    )
                << std::right << std::setw(20) << calc_fail_rate(
                                                    bus_monitor_flags.fail_counter,
                                                    bus_monitor_flags.rcv_counter
                                                    )
                << std::right << std::setw(12) << bus_monitor_flags.rcv_counter 
                << std::endl;

      rcv_counter_last = bus_monitor_flags.rcv_counter;
      tx_counter_last = bus_monitor_flags.tx_counter;
    }
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time));
  }
}

//---------------------------------------------------------------------------------
int main(int argc, char **argv) {

  // argv[1] = device name e.g. /dev/ttyUSB0
  // argv[2] = baud rate e.g. 115200
  // argv[3] = enable/disable software loop. Disable=(0 or no param) | Enable=1
  
  bool sw_loop = false;

  std::string my_device = argv[1];

  uint32_t bd = std::stoi(argv[2]);

  if (argc >= 4) {
      sw_loop = bool(std::stoi(argv[3]));
  }

  auto serial = create_comm_device(CommDevice::UART);

  serial->connect(my_device.c_str(), bd, sw_loop);

  std::mutex my_bus_mtx;

  std::mutex my_mon_mtx;

  std::thread writer(write_port, std::ref(serial));

  std::thread reader(read_port, std::ref(serial), std::ref(my_mon_mtx));

  std::thread monitor(bus_monitor, std::ref(my_mon_mtx));

  if (writer.joinable())
    writer.join();

  if (reader.joinable())
    reader.join();

  if (monitor.joinable())
    monitor.join();

  return 0;
}