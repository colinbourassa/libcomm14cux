# libcomm14cux
https://github.com/colinbourassa/libcomm14cux

**libcomm14cux** is a cross-platform library that is capable of communicating with the Lucas 14CUX ECU via its diagnostic serial port. The library itself has no user-interface and requires a front-end application to display data. Included with this code is a simple command-line utility (named "read14cux") which uses the library to read and display bytes from the 14CUX's memory.

The [RoverGauge](https://github.com/colinbourassa/rovergauge) project provides a graphical front-end to this library. Special hardware is required to match the ECU's signaling scheme. The hardware is beyond the scope of this document and detailed in the RoverGauge README.

## License
libcomm14cux is licensed under the GPL v3. See the file "LICENSE" for details.

## Disclaimer
While this software has been shown to be reliable and is provided in good faith, it is provided with ABSOLUTELY NO WARRANTY.

## Compatibility
libcomm14cux has been tested with the following operating systems at some point in its history:

* Linux (AMD64)
* OpenBSD 5.2 (AMD64)
* FreeBSD 9.0 (AMD64)
* Windows XP, 7, 10 (i686, AMD64)
* Macintosh OS X 10.6

Note that installation packages are not available for some of the OSs listed above, but it is possible to build from source for those platforms.

## Using read14cux
The libcomm14cux library must be visible to the read14cux executable for it to run. (In Windows, the most straightforward way to do this is to ensure that the DLL and read14cux.exe are in the same directory.)

The read14cux utility reads a specified number of bytes from a specified memory location in the ECU, and will either display those bytes at the console (16 bytes per line), or, if an output filename is given, write the bytes to a file.

The read14cux command line parameters are as follows:

`read14cux <serial-device> [-b baud-rate] <address> <length> [output-file]`

* serial-device:
    The name of the serial port device, which is probably an FTDI USB converter. In recent Linux distributions, it's probably "/dev/ttyUSB0". In Windows, it's a COM port (such as "COM2") -- the specific COM port can be found by locating the serial device in the Windows Device Manager.
* baud-rate:
    Optional parameter, prefixed with '-b' if present. Sets the baud rate used when communicating with the ECU. This is only useful if your ECU is running customized firmware that supports a higher baud rate.
* address:
    Memory address at which to start reading data. This should be specified in hex (using the "0x" prefix, as in "0xC000".)
* length:
    Number of bytes to read.
* output-file:
    Optional. If given, the file to which the bytes will be written (rather than to the console.)

## Issues with writing to memory in the 14CUX
Write operations are of limited usefulness; fuel maps cannot be changed in this way because they are read from the ROM, and sensor readings are continually updated by the ECU (meaning that any change to the memory location of a sensor value would almost immediately be overwritten.)

Care should be taken when writing memory while the engine is running; it's possible to affect the operation of the fuel pump or idle-control stepper motor in this way.

## Notes for those developing frontends to libcomm14cux
Because Windows still uses legacy DOS names for certain devices (such as COM ports), there are restrictions on how such devices may be manipulated by code. In order to open COM ports beyond COM9, Windows I/O calls (such as CreateFile()) require use of the prefix "\\.\" on the device name, as in "\\.\COM10". (Also note that the backslashes must be properly escaped where appropriate.)

