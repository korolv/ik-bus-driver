# ik-bus-driver
## Introduction
The I and K buses are a serial communications bus in which all connected control units can
send as well as receive information over one wire. I/K-bus is part of the bus system BMW.

In order to support I/K-bus communication protocol you need modify linux:
- patch linux kernel;
- a special driver for the network device;
- a special network protocol driver.

This repository contains everything you need for support I/K-bus.

## Patch linux kernel
Patches will add a I/K-bus protocol family to the Linux kernel, enabling the user to communicate over I/K-bus using the simple and well-established socket API. This approach is very similar to the SocketCAN implementation which is used to enable CAN-communication for Linux.

## Network device driver
It is slibus. It implements network device which connect to serial device over special tty line discipline. Slibus receive and transmit data between UART and networking subsystem Linux. When we attach slibus tty line discipline, we get network device (ibus0, ibus1, ibus3...) in system.

## Network protocol driver
pf_ibus_raw is linux kernel module which dispatches I/K-bus frames between network devices and socket layer. It based only on RAW socket.
