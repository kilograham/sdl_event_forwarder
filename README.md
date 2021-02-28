This is a simple tool to forward raw keyboard, mouse or gamepad
events to an RP2040 connected via UART. The events are sent via escape sequences, so the program on the RP2040 needs to be expecting them.

Build

```shell
mkdir build
cd build
cmake ..
make
```
Usage:

```
./sdl_event_forwarder <tty_device>
```

e.g.

```shell
./sdl_event_forwarder /dev/ttyUSB0
```

This will open a window, whose input is forwarded