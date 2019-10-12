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
		enum class BaudRate {
			B0,
			B50,
			B75,
			B110,
			B134,
			B150,
			B200,
			B300,
			B600,
			B1200,
			B1800,
			B2400,
			B4800,
			B9600,
			B19200,
			B38400,
			B57600,
			B115200,
			B230400,
		};

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
			BaudRate    baud_rate    = BaudRate::B9600;
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
		int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
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
		// tty.c_cflag &= ~(CSIZE | PARENB);
		// tty.c_iflag &= ~(
		// 	IGNBRK | BRKINT | PARMRK | ISTRIP |
		// 	INLCR  | IGNCR  | ICRNL  | IXON
		// );
		// tty.c_oflag &= ~OPOST;
		// tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		// tty.c_cflag |= CS8;

		// baud
		cfsetospeed(&tty, settings.baud_rate);
		cfsetispeed(&tty, settings.baud_rate);

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
		tcflush(handle, TCIOFLUSH);
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
			int ready = select(sp.handle + 1, nullptr, &wfds, nullptr, tvp);
			if (ready == -1) {
				// error
			} else if (ready == 0) {
				// timeout
				break;
			}
			if (FD_ISSET(sp.handle, &wfds)) {
				int bytes_written = write(
					sp.handle,
					buf[written_count],
					length - written_count
				);
			} else {
				// cancelled
			}
		}
#endif
	}
} // serial

#endif
