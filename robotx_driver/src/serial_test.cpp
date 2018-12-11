#include <boost/asio.hpp>
#include <iostream>

using namespace boost::asio;

int main(int argc, char *argv[])
{
	std::string wbuf = "a";
	char rbuf[32];
	std::size_t length;

	const char *PORT = "/dev/serial/by-id/usb-mbed_Microcontroller_101000000000000000000002F7F1F5C4-if01";
	io_service io;
	serial_port port( io, PORT );
	port.set_option(serial_port_base::baud_rate(9600));
	port.set_option(serial_port_base::character_size(8));
	port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
	port.set_option(serial_port_base::parity(serial_port_base::parity::none));
	port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

	port.write_some( buffer( wbuf ) );
	/*
    length = port.read_some( buffer( rbuf ) );

	std::cout.write(rbuf, length);
    */
	return 0;
}