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
#include <cstdio>
}
#elif SERIAL_OS_LINUX
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#endif

namespace serial {
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
		int         baud_rate    = 9600;
		int         char_size    = 8;
		FlowControl flow_control = FlowControl::NONE;
		Parity      parity       = Parity::NONE;
		StopBits    stop_bits    = StopBits::ONE;
	};

	enum class Err {
		NONE,
		DISCONNECT,
		INVALID_DEVICE,
		INVALID_BAUD,
		INVALID_CHAR_SIZE,
		INVALID_STOP_BITS,
		INVALID_TIMEOUT,
		TIMEOUT,
		LATENCY,
	};

	template <typename T>
	struct Result {
		Err err = Err::NONE;
		T val;
	};

#if SERIAL_OS_WINDOWS
	using NativeHandle = HANDLE;
#elif SERIAL_OS_LINUX
	using NativeHandle = int;
#endif

#if SERIAL_OS_WINDOWS
	namespace detail {
		int get_all_device_subkeys(char*** subkeys) {
			TCHAR* key_path = (TCHAR*)"SYSTEM\\CurrentControlSet\\Services\\FTDIBUS\\Enum";
			HKEY key;
			auto r = RegOpenKeyEx(HKEY_LOCAL_MACHINE, key_path, 0, KEY_READ, &key);
			if (r)
			{
				return 0; // or -1?
			}

			DWORD type;
			DWORD count;
			DWORD count_size = sizeof(DWORD);
			r = RegQueryValueEx(key, "Count", NULL, &type, (LPBYTE)&count, &count_size);
			*subkeys = new char*[count];
			DWORD port_info_size = 140;
			TCHAR port_info[port_info_size];
			for (int iii = 0; iii < count; iii++)
			{
				char num[10];
				sprintf(num, "%d", iii);
				RegQueryValueEx(
					key, num, NULL, &type, (LPBYTE)&port_info, &port_info_size
				);

				int port_info_len = strlen(port_info) - 4;
				char* subkey = new char[port_info_len + 1];
				strcpy(subkey, &port_info[4]);
				subkey[port_info_len] = 'A';
				subkey[strcspn(subkey, "&")] = '+';
				subkey[strcspn(subkey, "\\")] = '+';
				subkeys[iii] = &subkey;
			}
			RegCloseKey(key);
			return count;
		}

		HKEY open_device_params(char* port_name) {
			char** device_subkeys;
			auto device_count = get_all_device_subkeys(&device_subkeys);
			HKEY key;
			TCHAR* keypath = (TCHAR*)"SYSTEM\\CurrentControlSet\\Enum\\FTDIBUS";
			long r = RegOpenKeyEx(HKEY_LOCAL_MACHINE, keypath, 0, KEY_READ, &key);
			if (r) {
				// no drives
			}

			int index = 0;
			HKEY read_only_key = 0, param_key = 0;
			for (int iii = 0; iii < device_count; iii++) {
				char* subkey = device_subkeys[iii];
				TCHAR param_path[100];
				snprintf(param_path, 100, "%s\\0000\\Device Parameters", subkey);
				r = RegOpenKeyEx(key, param_path, 0, KEY_READ, &read_only_key);
				if (r) {
					continue;
				}
				DWORD reg_port_name_size = 10;
				TCHAR reg_port_name[reg_port_name_size];
				DWORD type;
				r = RegQueryValueEx(
					read_only_key, "PortName", NULL, &type, (LPBYTE)&reg_port_name, &reg_port_name_size
				);
				bool names_match = strcmp(port_name, reg_port_name) == 0;
				if (!r && names_match) {
					r = RegOpenKeyEx(read_only_key, NULL, 0, KEY_READ | KEY_SET_VALUE, &param_key);
					if (r) {
						// admin priv error
					}
				}
				RegCloseKey(read_only_key);
				if (param_key) {
					break;
				}
			}
			RegCloseKey(key);
			return param_key;
		}
	}
#endif

	struct SerialPort {
		char* path;
		NativeHandle handle;
		Settings settings;
	};

	SerialPort open(char* device) {
#if SERIAL_OS_WINDOWS
		HANDLE handle = CreateFileA(
			device, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0
		);

		if (handle == INVALID_HANDLE_VALUE) {
			DWORD err = GetLastError();
			// error out
		}
		DCB dcb;
		dcb.DCBlength = sizeof(dcb);
		if (!GetCommState(handle, &dcb)) {
			DWORD last_error = GetLastError();
			CloseHandle(handle);
			// error out
		}

		// defaults
		dcb.fBinary = true;
		dcb.fNull = false;
		dcb.fAbortOnError = false;
		dcb.BaudRate = 0;
		dcb.ByteSize = 8;
		dcb.fOutxCtsFlow = false;
		dcb.fOutxDsrFlow = false;
		dcb.fDsrSensitivity = false;
		dcb.fOutX = false;
		dcb.fInX= false;
		dcb.fRtsControl = DTR_CONTROL_DISABLE;
		dcb.fParity = false;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		if (!SetCommState(handle, &dcb)) {
			DWORD last_error = GetLastError();
			CloseHandle(handle);
			// error out
		}

		return SerialPort {
			.path = device,
			.handle = handle,
			.settings = Settings(),
		};
#elif SERIAL_OS_LINUX
		int fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (fd < 0) {
			// error out
		}
		return SerialPort {
			.path = device,
			.handle = fd,
			.settings = Settings(),
		};
#endif
	}

	void configure(SerialPort& sp, Settings settings) {
#if SERIAL_OS_WINDOWS
		DCB dcb;
		dcb.DCBlength = sizeof(dcb);
		if (!GetCommState(sp.handle, &dcb)) {
			DWORD last_error = GetLastError();
			CloseHandle(sp.handle);
			// error out
		}

		// baud
		dcb.BaudRate = settings.baud_rate;

		// flow
		dcb.fOutxCtsFlow = false;
		dcb.fOutxDsrFlow = false;
		dcb.fTXContinueOnXoff = true;
		dcb.fDtrControl = DTR_CONTROL_ENABLE;
		dcb.fDsrSensitivity = false;
		dcb.fOutX = false;
		dcb.fInX = false;
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		if (settings.flow_control == FlowControl::SOFTWARE) {
			dcb.fOutX = true;
			dcb.fInX = true;
		} else if (settings.flow_control == FlowControl::HARDWARE) {
			dcb.fOutxCtsFlow = true;
			dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
		}

		// parity
		switch (settings.parity) {
			case Parity::NONE:
				dcb.fParity = false;
				dcb.Parity = NOPARITY;
				break;
			case Parity::ODD:
				dcb.fParity = true;
				dcb.Parity = ODDPARITY;
				break;
			case Parity::EVEN:
				dcb.fParity = true;
				dcb.Parity = EVENPARITY;
				break;
		}

		// stop bits
		switch (settings.stop_bits) {
			case StopBits::ONE:
				dcb.StopBits = ONESTOPBIT;
				break;
			case StopBits::ONE_POINT_FIVE:
				dcb.StopBits = ONE5STOPBITS;
				break;
			case StopBits::TWO:
				dcb.StopBits = TWOSTOPBITS;
				break;
		}

		// char size
		dcb.ByteSize = settings.char_size;

		if (!SetCommState(sp.handle, &dcb)) {
			DWORD last_error = GetLastError();
			CloseHandle(sp.handle);
			// error out
		}
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
		if (settings.char_size < 5 || settings.char_size > 8) {
			// error out
		}
		switch (settings.char_size) {
			case 5:
				tty.c_cflag |= CS5;
				break;
			case 6:
				tty.c_cflag |= CS6;
				break;
			case 7:
				tty.c_cflag |= CS7;
				break;
			case 8:
				tty.c_cflag |= CS8;
				break;
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

	void set_low_latency(SerialPort& sp, bool ll) {
#if SERIAL_OS_WINDOWS
		int latency = 16;
		if (ll) {
			latency = 1;
		}
		HKEY key = detail::open_device_params(sp.path);
		int r = RegSetValueEx(
			key, "LatencyTimer", 0, REG_DWORD, (LPBYTE)&latency, sizeof(latency)
		);
		if (r)
		{
			// error out
		}
		RegCloseKey(key);
#elif SERIAL_OS_LINUX
		serial_struct ser;
		ioctl(sp.handle, TIOCGSERIAL, &ser);
		ser.flags |= ll ? ASYNC_LOW_LATENCY : ~ASYNC_LOW_LATENCY;
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

	void clear_input(SerialPort& sp) {
#if SERIAL_OS_WINDOWS
		PurgeComm(sp.handle, PURGE_RXCLEAR | PURGE_RXABORT);
#elif SERIAL_OS_LINUX
		tcflush(sp.handle, TCIFLUSH);
#endif
	}

	void clear_output(SerialPort& sp) {
#if SERIAL_OS_WINDOWS
		PurgeComm(sp.handle, PURGE_TXCLEAR | PURGE_TXABORT);
#elif SERIAL_OS_LINUX
		tcflush(sp.handle, TCOFLUSH);
#endif
	}

	int read(SerialPort& sp, char* buf, int length, int timeout) {
#if SERIAL_OS_WINDOWS
		COMMTIMEOUTS timeouts;
		if (timeout == -1) {
			timeouts.ReadIntervalTimeout = 0;
			timeouts.ReadTotalTimeoutConstant = 0;
			timeouts.ReadTotalTimeoutMultiplier = 0;
		} else if (timeout == 0) {
			timeouts.ReadIntervalTimeout = MAXDWORD;
			timeouts.ReadTotalTimeoutConstant = 0;
			timeouts.ReadTotalTimeoutMultiplier = 0;
		} else if (timeout > 0) {
			timeouts.ReadIntervalTimeout = 0;
			timeouts.ReadTotalTimeoutConstant = (DWORD)timeout;
			timeouts.ReadTotalTimeoutMultiplier = 0;
		} else {
			// invalid timeout error out
		}
		if (!SetCommTimeouts(sp.handle, &timeouts)) {
			DWORD last_error = GetLastError();
			CloseHandle(sp.handle);
			// error out
		}
		DWORD bytes_read = 0;
		ReadFile(sp.handle, buf, length, &bytes_read, NULL);
		return bytes_read;
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
				int bytes_read = ::read(sp.handle, buf, length - read_count);
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
		COMMTIMEOUTS timeouts;
		if (timeout == -1) {
			timeouts.WriteTotalTimeoutConstant = MAXDWORD;
			timeouts.WriteTotalTimeoutMultiplier = 0;
		} else if (timeout == 0) {
			timeouts.WriteTotalTimeoutConstant = 0;
			timeouts.WriteTotalTimeoutMultiplier = 0;
		} else if (timeout > 0) {
			timeouts.WriteTotalTimeoutConstant = (DWORD)timeout;
			timeouts.WriteTotalTimeoutMultiplier = 0;
		} else {
			// invalid timeout error out
		}
		if (!SetCommTimeouts(sp.handle, &timeouts)) {
			DWORD last_error = GetLastError();
			CloseHandle(sp.handle);
			// error out
		}
		DWORD written = 0;
		bool status = WriteFile(sp.handle, buf, length, &written, NULL);
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
