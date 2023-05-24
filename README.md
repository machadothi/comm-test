# Communication bus loopback tester

Application to test a communication bus by writing a sequence of bytes and verifying the integrity when receiving it back. The loopback can be performed in some different ways like:

1. short-circuiting the RX-TX in case of a serial bus;
2. software loop back, performed by the linux driver
3. external device does the loopback by returning the data it receives.

## Building
To build the project in a Linux distribution, follow the bellow steps.

```c++
$ mkdir build && cd build

$ cmake ..

$ make
```

## Running
To run the application

```c++
$ ./comm_test <device> <baud_rate> <sw_loopback_en>
```

For instance

```c++
$ ./comm_test /dev/ttyUSB0 9600 1
```