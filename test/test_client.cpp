#include <string>
#include <iostream>
#include "../include/serial.hpp"

int main() {
	auto sp = serial::open("/dev/pst/3");
	serial::Settings settings;
	settings.baud_rate = 19200;

	serial::configure(sp, settings);

	std::string rx_buf;
	rx_buf.reserve(8);

	serial::write(sp, "test_1", 6, -1);
	rx_buf.clear();
	serial::read(sp, &rx_buf[0], 8, -1);
	std::cout << "Client: " << rx_buf << "\n"; 

	serial::write(sp, "test_2", 6, -1);
	rx_buf.clear();
	serial::read(sp, &rx_buf[0], 8, -1);
	std::cout << "Client: " << rx_buf << "\n"; 

	serial::write(sp, "test_3", 6, -1);
	rx_buf.clear();
	serial::read(sp, &rx_buf[0], 8, -1);
	std::cout << "Client: " << rx_buf << "\n"; 
}
