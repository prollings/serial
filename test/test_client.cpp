#include <string>
#include <iostream>
#include "../include/serial.hpp"

int main() {
	auto sp = serial::open("/dev/pts/3");
	serial::Settings settings;
	settings.baud_rate = 19200;

	serial::configure(sp, settings);

	std::string rx_buf;

	int write_timeout = 100;
	int read_timeout = -1;

	for (auto num_char : {'1', '2', '3'}) {
		rx_buf.clear();
		rx_buf.resize(8, '\0');
		std::string tx_buf = "test_";
		tx_buf.push_back(num_char);
		serial::write(sp, tx_buf.data(), tx_buf.length(), write_timeout);
		auto res = serial::read(sp, &rx_buf[0], rx_buf.length(), read_timeout);
		std::cout << "client rx: '" << rx_buf << "' with err: " << (int)res.err << "\n"; 
	}

	std::cout << "client finished\n";
}
