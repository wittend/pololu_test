# pololu_test
This is a simple utility meant to test the integrity of an I2C bus created using a Pololu "Pololu Isolated USB-to-I²C Adapter with Isolated Power" interface (**Pololu item #: 5397**)  

It probably works with the confusingly similar product: "Pololu Isolated USB-to-I²C" Adapter (**Pololu item #: 5396**). I don't know. 

This utility reads the device information and reports the interface onformation and then enumerates the addresses of devices found on the bus:
The utility currently devfaults to using the port "/dev/ttyACM2" which appears in my /dev folder when the device is plugged in.  YMMV.  

No facility short of rebuilding the utility with an alternative path string for the port is currently available.

### To build, do the usual CMake thing:
```
Clone this repo to ./pololu_test
$ cd pololu_test
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### To run, cd to:

- pololu_test/cmake-build-debug/
- (or pololu_test/cmake-build-release/)

### then:

$ ./pololu_test
```
Connecting to /dev/ttyACM2...
Connected.
  Device Info:
     Vendor ID: 0x1FFB
     Product ID: 0x2503
     Firmware Version: 1.00
     Serial Number: 7400-4F00-0150-3642-483

  Scanning for I2C devices...
    Found 2 device(s):
    Address: 0x1F
    Address: 0x23
```
You may need to use sudo.  NOt sure...
