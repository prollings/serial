# serial
Single header serial library for Windows and Linux.

# API

## The `SerialPort` type

A POD type that holds information about the serial port.
An object of this type is produced by `open` and must be passed through to all other functions.

## The `Settings` type

A POD type that holds:

* Baud rate
* Character size
* Flow control
* Parity
* Stop bits

An object of this type should be provided to the `configure` to apply the settings to an open serial port.
The type has sane defaults that apply to most devices.
Usually the only thing that needs changing is the baud rate.

## Open a port with `open`

Provide a valid path to a serial port device and a `SerialPort` object will be returned.

## `configure` an open port

After opening a port, it should be configured.
Usually the only setting that needs to be changed is the baud rate.

## `in_waiting`

Return the number of bytes waiting to be read off the serial port.

## `clean_input`

Discard all data that is currently waiting to be read.

## `read`

Read bytes from the serial port.

Args:

1. port: `SerialPort&`
1. buffer: `char*`
1. length: `int`
1. timeout: `int`

The <buffer> should be at least <length> in size.

Timeout:

* `0`: the read will not block but return instantly with the number of bytes read. The buffer will be filled by any bytes read.
* `-1`: the read will block until <length> bytes have been read.
* `> 0`: the read will block until either <length> bytes have been read, or <timeout> milliseconds have passed.

## `write`

Write bytes to serial port

Args:

1. port: `SerialPort&`
1. buffer: `char*`
1. length: `int`
1. timeout: `int`

Again, the <buffer> should be at least <length> in size.

The rules for <timeout> are the same as with `read`.
