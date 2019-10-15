/*
MIT License

Copyright (c) 2019 Patrick Rollings

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef SERIAL_HPP
#define SERIAL_HPP

#if __linux__
#define SERIAL_OS_LINUX 1
#elif __APPLE__
#define SERIAL_OS_APPLE 1
#endif

#ifdef _WIN32
#define SERIAL_OS_WINDOWS 1
#endif

#if SERIAL_OS_WINDOWS
extern "C" {
#include <windows.h>
}
#elif SERIAL_OS_LINUX
#include <termios.h>
#include <sys/ioctl.h>
#endif

namespace serial {
	namespace settings {
		enum class CharSize {
			CS5,
			CS6,
			CS7,
			CS8,
		};

		enum class FlowControl {
			NONE,
			SOFTWARE,
			HARDWARE,
		};

		enum class Parity {
			NONE,
			ODD,
			EVEN,
		};

		enum class StopBits {
			ONE,
			ONE_POINT_FIVE,
			TWO,
		};

		struct Settings {
			uint64_t    baud_rate    = 9600;
			CharSize    char_size    = CharSize::CS8;
			FlowControl flow_control = FlowControl::NONE;
			Parity      parity       = Parity::NONE;
			StopBits    stop_bits    = StopBits::ONE;
		};

#if SERIAL_OS_WINDOWS
	using NativeHandle = int;
#elif SERIAL_OS_LINUX
	using NativeHandle = int;
#endif

	struct SerialPort {
		NativeHandle handle;
		Settings settings;
	};

	SerialPort open(char* device) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
		int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (fd < 0) {
			// error out
		}
		return { 
			.handle: fd,
			.settings: Settings(),
		};
#endif
	}

	void configure(SerialPort& sp, Settings settings) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
		termios tty;
		auto fd = sp.handle;

		if (tcgetattr(fd, &tty) < 0) {
			// error out
		}

		// set defaults
		tty.c_cflag |= (CLOCAL | CREAD);
		tty.c_iflag |= IGNPAR;

		// raw mode
		cfmakeraw(&tty);

		// baud
		int br = 0;
		switch (settings.baud_rate) {
			case 0:       br = B0;       break;
			case 50:      br = B50;      break;
			case 75:      br = B75;      break;
			case 110:     br = B110;     break;
			case 134:     br = B134;     break;
			case 150:     br = B150;     break;
			case 200:     br = B200;     break;
			case 300:     br = B300;     break;
			case 600:     br = B600;     break;
			case 1200:    br = B1200;    break;
			case 1800:    br = B1800;    break;
			case 2400:    br = B2400;    break;
			case 4800:    br = B4800;    break;
			case 9600:    br = B9600;    break;
			case 19200:   br = B19200;   break;
			case 38400:   br = B38400;   break;
			case 57600:   br = B57600;   break;
			case 115200:  br = B115200;  break;
			case 230400:  br = B230400;  break;
			case 460800:  br = B460800;  break;
			case 500000:  br = B500000;  break;
			case 576000:  br = B576000;  break;
			case 921600:  br = B921600;  break;
			case 1000000: br = B1000000; break;
			case 1152000: br = B1152000; break;
			case 1500000: br = B1500000; break;
			case 2000000: br = B2000000; break;
			case 2500000: br = B2500000; break;
			case 3000000: br = B3000000; break;
			case 3500000: br = B3500000; break;
			case 4000000: br = B4000000; break;
			default:
				//error
				break;
		}
		cfsetospeed(&tty, br);
		cfsetispeed(&tty, br);

		// char size
		switch (settings.char_size) {
			case CharSize::CS5:
				tty.c_cflag |= CS5;
			case CharSize::CS6:
				tty.c_cflag |= CS6;
			case CharSize::CS7:
				tty.c_cflag |= CS7;
			case CharSize::CS8:
				tty.c_cflag |= CS8;
		}

		// flow control
		switch (settings.flow_control) {
			case FlowControl::NONE:
				tty.c_iflag &= ~(IXOFF | IXON);
				tty.c_cflag &= ~CRTSCTS;
				break;
			case FlowControl::SOFTWARE:
				tty.c_iflag |= IXOFF | IXON;
				tty.c_cflag &= ~CRTSCTS;
				break;
			case FlowControl::HARDWARE:
				tty.c_iflag &= ~(IXOFF | IXON);
				tty.c_cflag |= CRTSCTS;
				break;
		}

		// parity
		switch (settings.parity) {
			case Parity::NONE:
				tty.c_iflag |= IGNPAR;
				tty.c_cflag &= ~(PARENB | PARODD);
				break;
			case Parity::EVEN:
				tty.c_iflag &= ~(IGNPAR | PARMRK);
				tty.c_iflag |= INPCK;
				tty.c_cflag |= PARENB;
				tty.c_cflag &= ~PARODD;
				break;
			case Parity::ODD:
				tty.c_iflag &= ~(IGNPAR | PARMRK);
				tty.c_iflag |= INPCK;
				tty.c_cflag |= (PARENB | PARODD);
				break;
		}

		// stop bits
		switch (settings.stop_bits) {
			case StopBits::ONE:
				tty.c_cflag &= ~CSTOPB;
				break;
			case StopBits::TWO:
				tty.c_cflag |= CSTOPB;
				break;
			case StopBits::ONE_POINT_FIVE:
				// linux no supporty
				break;
		}

		if (tcsetattr(fd, TCSANOW, &tty) != 0) {
			// error out
		}
#endif
	}

	int in_waiting(SerialPort& sp) {
#if SERIAL_OS_WINDOWS
		DWORD flags;
		COMSTAT comstat;
		auto err = ClearCommError(sp.handle, &flags, &comstat);
		return comstat.cbInQue;
#elif SERIAL_OS_LINUX
		int ready = 0;
		ioctl(sp.handle, FIONREAD, &ready);
		return ready;
#endif
	}

	void clean_input(SerialPort& sp) {
#if SERIAL_OS_WINDOWS
		PurgeComm(sp.handle, PURGE_RXCLEAR | PURGE_RXABORT);
#elif SERIAL_OS_LINUX
		tcflush(sp.handle, TCIFLUSH);
#endif
	}

	int read(SerialPort& sp, char* buf, int length, int timeout) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
		if (timeout == 0) {
			// non-blocking
			return ::read(sp.handle, buf, length);
		}

		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(sp.handle, &rfds);

		timeval tv;
		tv.tv_sec = timeout / 1000;
		tv.tv_usec = (timeout % 1000) / 1000;

		timeval* tvp = &tv;
		if (timeout == -1) {
			tvp = nullptr;
		}

		int read_count = 0;

		while (read_count < length) {
			int ready = select(sp.handle + 1, &rfds, nullptr, nullptr, tvp);
			if (ready == -1) {
				// error out
			} else if (ready == 0) {
				// timeout occured
				break;
			}
			if (FD_ISSET(sp.handle, &rfds)) {
				int bytes_read = read(sp.handle, buf, length - read_count);
				if (bytes_read < 0) {
					// error out
				}
				read_count += bytes_read;
			} else {
				// select finished, but the sp fd wasn't part of it
				// useful for cancelling, but impossible without a
				// special fd for cancellation.
				break;
			}
		}
		return read_count;
#endif
	}

	void write(SerialPort& sp, char* buf, int length, int timeout) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
		if (timeout == 0) {
			int wlen = ::write(sp.handle, buf, length);
			if (wlen != length) {
				// error out
			}
			return;
		}
		fd_set wfds;
		FD_ZERO(&wfds);
		FD_SET(sp.handle, &wfds);

		timeval tv;
		tv.tv_sec = timeout / 1000;
		tv.tv_usec = (timeout % 1000) / 1000;

		timeval* tvp = &tv;
		if (timeout == -1) {
			tvp = nullptr;
		}

		int written_count = 0;

		while (written_count < length) {
			int bytes_written = ::write(
				sp.handle,
				&buf[written_count],
				length - written_count
			);

			int ready = select(sp.handle + 1, nullptr, &wfds, nullptr, tvp);
			if (ready == -1) {
				// error
			} else if (ready == 0) {
				// timeout
				break;
			}
			if (FD_ISSET(sp.handle, &wfds)) {
				written_count += bytes_written;
			} else {
				// cancelled
			}
		}
#endif
	}
} // serial

#endif
