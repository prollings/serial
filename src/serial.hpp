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
		using SysSettings = DCB;
#elif SERIAL_OS_LINUX
		using SysSettings = termios;
#endif

		void set_baud_rate(SysSettings& ss, BaudRate br) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
			cfsetospeed(&ss, br);
			cfsetispeed(&ss, br);
#endif
		}

		void set_char_size(SysSettings& ss, CharSize cs) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
			switch (cs) {
				case CharSize::CS5:
					ss.c_cflag |= CS5;
				case CharSize::CS6:
					ss.c_cflag |= CS6;
				case CharSize::CS7:
					ss.c_cflag |= CS7;
				case CharSize::CS8:
					ss.c_cflag |= CS8;
			}
#endif
		}

		void set_flow_control(SysSettings& ss, FlowControl fc) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
			switch (fc) {
				case FlowControl::NONE:
					ss.c_iflag &= ~(IXOFF | IXON);
					ss.c_cflag &= ~CRTSCTS;
					break;
				case FlowControl::SOFTWARE:
					ss.c_iflag |= IXOFF | IXON;
					ss.c_cflag &= ~CRTSCTS;
					break;
				case FlowControl::HARDWARE:
					ss.c_iflag &= ~(IXOFF | IXON);
					ss.c_cflag |= CRTSCTS;
					break;
			}
#endif
		}

		void set_parity(SysSettings& ss, Parity p) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
			switch (p) {
				case Parity::NONE:
					ss.c_iflag |= IGNPAR;
					ss.c_cflag &= ~(PARENB | PARODD);
					break;
				case Parity::EVEN:
					ss.c_iflag &= ~(IGNPAR | PARMRK);
					ss.c_iflag |= INPCK;
					ss.c_cflag |= PARENB;
					ss.c_cflag &= ~PARODD;
					break;
				case Parity::ODD:
					ss.c_iflag &= ~(IGNPAR | PARMRK);
					ss.c_iflag |= INPCK;
					ss.c_cflag |= (PARENB | PARODD);
					break;
			}
#endif
		}

		void set_stop_bits(SysSettings& ss, StopBits sb) {
#if SERIAL_OS_WINDOWS
#elif SERIAL_OS_LINUX
			switch (sb) {
				case StopBits::ONE:
					ss.c_cflag &= ~CSTOPB;
					break;
				case StopBits::TWO:
					ss.c_cflag |= CSTOPB;
					break;
				case StopBits::ONE_POINT_FIVE:
					// linux no supporty
					break;
			}
#endif
		}
	} // settings

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

		cfsetospeed(&tty, (speed_t)settings.baud_rate);
		cfsetispeed(&tty, (speed_t)settings.baud_rate);

		// set defaults
		tty.c_cflag |= (CLOCAL | CREAD);
		tty.c_iflag |= IGNPAR;

		cfmakeraw(&tty);
		// tty.c_cflag &= ~(CSIZE | PARENB);
		// tty.c_iflag &= ~(
		// 	IGNBRK | BRKINT | PARMRK | ISTRIP |
		// 	INLCR  | IGNCR  | ICRNL  | IXON
		// );
		// tty.c_oflag &= ~OPOST;
		// tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		// tty.c_cflag |= CS8;

		set_baud_rate(tty, settings.baud_rate);
		set_char_size(tty, settings.char_size);
		set_partiy(tty, settings.parity);
		set_stop_bits(tty, settings.stop_bits);
		set_flow_control(tty, settings.flow_control);

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
