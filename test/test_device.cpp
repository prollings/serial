#include <iostream>
#include <string>
#include "../include/serial.hpp"

int main() {
	auto sp = serial::open("/dev/pts/2");

	serial::Settings settings;
	settings.baud_rate = 19200;
	serial::configure(sp, settings);

	std::string buf;
	char character = 0;
	bool running = true;
	while (running) {
		auto res = serial::read(sp, &character, 1, 100);
		if (res.val == 1) {
			buf.push_back(character);
			std::cout << "dev buf: " << buf << "\n";
			std::string ret_buf;
			bool buffer_complete = false;
			if (buf == "test_1") {
				buffer_complete = true;
				ret_buf = "result_1";
			} else if (buf == "test_2") {
				buffer_complete = true;
				ret_buf = "result_2";
			} else if (buf == "test_3") {
				buffer_complete = true;
				ret_buf = "result_3";
				running = false;
			}
			if (buffer_complete) {
				buf.clear();
				auto res = serial::write(sp, ret_buf.data(), ret_buf.length(), 0);
				std::cout << "device tx err: " << (int)res.err << "\n";
			}
		}
	}
	std::cout << "device finished\n";
}
