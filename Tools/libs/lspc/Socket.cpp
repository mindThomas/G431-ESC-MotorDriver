#include "LSPC.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>
#include <mutex>
#include <iostream>
#include <termios.h>
#include <sys/ioctl.h>
//#include <asm/termios.h>

#include <boost/optional.hpp>

namespace lspc
{
	Socket::Socket() : serial_is_sending(false), controller_port(ioservice) {}

	Socket::Socket(const std::string &com_port_name, const size_t baudRate) : serial_is_sending(false), controller_port(ioservice) {
		open(com_port_name, baudRate);
	}

	Socket::~Socket() {
		std::lock_guard<std::mutex> lock(resourceMutex_);
        controller_port.close();

		ioservice.stop();

		if (ioservice_thread.joinable()) {
			ioservice_thread.join();
		}        
	}

	// Process incoming data on serial link
	//
	// @brief Reads the serial buffer and dispatches the received payload to the
	// relevant message handling callback function.
	void Socket::processSerial(const boost::system::error_code &error,
							   std::size_t bytes_transferred) {
		if (error == boost::system::errc::operation_canceled) {
			return;
		} else if (error) {
            std::cout << "LSPC: " + error.message() << std::endl;
			controller_port.close();
			return;
			//throw std::runtime_error("processSerial: " + error.message());
		}

		for (size_t i = 0; i < bytes_transferred; i++) {
            try {
                processIncomingByte(read_buffer[i]);
            }
            catch (std::exception &e) {
                std::cout << "LSPC: " << e.what() << std::endl;
                controller_port.close();
                return;
            }
            catch (...) {
                std::cout << "LSPC: Uncaught error. Closing port." << std::endl;
                controller_port.close();
                return;
            }
        }

		// READ THE NEXT PACKET
		// Our job here is done. Queue another read.
		/*boost::asio::async_read(
				controller_port, boost::asio::buffer(read_buffer),
				std::bind(&Socket::processSerial, this, std::placeholders::_1,
						  std::placeholders::_2));*/
        controller_port.async_read_some(boost::asio::buffer(read_buffer),
                std::bind(&Socket::processSerial, this, std::placeholders::_1,
                          std::placeholders::_2));
		return;
	}

	void Socket::serialWriteCallback(const boost::system::error_code &error,
									 size_t bytes_transferred) {
		serial_is_sending = false;
	}

	template <typename SyncReadStream, typename MutableBufferSequence>
	void Socket::readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time)
	{
		boost::optional<boost::system::error_code> timer_result;
		boost::asio::deadline_timer timer(s.get_io_service());
		timer.expires_from_now(expiry_time);
		timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });

		boost::optional<boost::system::error_code> read_result;
		boost::asio::async_read(s, buffers, [&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });

		s.get_io_service().reset();
		while (s.get_io_service().run_one())
		{
			if (read_result)
				timer.cancel();
			else if (timer_result)
				s.cancel();
		}
        s.get_io_service().reset();

		if (*read_result)
			throw boost::system::system_error(*read_result);
	}

	boost::system::error_code Socket::Flush() {
    	bool continueReading = true;
    	while (continueReading) {
    	    try
    	    {
                readWithTimeout(controller_port, boost::asio::buffer(read_buffer), boost::posix_time::milliseconds(1));
    	    }
			catch (boost::system::system_error& e)
			{
    	        continueReading = false;
    	    }
    	}
	}

    /// From https://stackoverflow.com/questions/22581315/how-to-discard-data-as-it-is-sent-with-boostasio/22598329#22598329
    /// @brief Flush a serial port's buffers.
    ///
    /// @param serial_port Port to flush.
    /// @param what Determines the buffers to flush.
    /// @param error Set to indicate what error occurred, if any.
    boost::system::error_code Socket::flush_serial_port(
            boost::asio::serial_port& serial_port,
            flush_type what)
    {
        if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what))
        {
            return boost::system::error_code();
        }
        else
        {
            return boost::system::error_code(errno,
                                             boost::asio::error::get_system_category());
        }

        // From http://mnb.ociweb.com/mnb/MiddlewareNewsBrief-201303.html
#if 0
        boost::system::error_code ec;
#if !defined(BOOST_WINDOWS) && !defined(__CYGWIN__)
		const bool isFlushed =! ::tcflush(controller_port.native(), TCIOFLUSH);
		if (!isFlushed)
			ec = boost::system::error_code(errno,
										   boost::asio::error::get_system_category());
#else
		const bool isFlushed = ::PurgeComm(controller_port.native(),
        PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    if (!isFlushed)
        ec = boost::system::error_code(::GetLastError(),
            boost::asio::error::get_system_category());
#endif
    	return ec;
#endif
    }

    void set_speed(boost::asio::serial_port& p, const size_t speed)
    {
        termios t;
        int fd = p.native_handle();

        // This can only be used to change to one of the default baud rate
        // e.g. B115200 or B921600
        if (tcgetattr(fd, &t) < 0) { throw boost::system::system_error(boost::asio::error::invalid_argument); }
        if (cfsetspeed(&t, speed) < 0) { throw boost::system::system_error(boost::asio::error::invalid_argument); }
        if (tcsetattr(fd, TCSANOW, &t) < 0) { throw boost::system::system_error(boost::asio::error::invalid_argument); }
    }

    void setCustomBaudRate(boost::asio::serial_port& p, const size_t speed)
    {
        // See https://stackoverflow.com/questions/12646324/how-can-i-set-a-custom-baud-rate-on-linux
        // And https://gist.github.com/kennethryerson/f7d1abcf2633b7c03cf0
        // And http://www.hce-engineering.com/setting-custom-baud-rate-on-serial-port/

        /* extended termios struct for custom baud rate */
        struct termios2 {
            tcflag_t c_iflag;		/* input mode flags */
            tcflag_t c_oflag;		/* output mode flags */
            tcflag_t c_cflag;		/* control mode flags */
            tcflag_t c_lflag;		/* local mode flags */
            cc_t c_line;			/* line discipline */
            cc_t c_cc[NCCS];		/* control characters */
            speed_t c_ispeed;		/* input speed */
            speed_t c_ospeed;		/* output speed */
        };

        struct termios2 tio;
        int fd = p.native_handle();

        ioctl(fd, TCGETS2, &tio);
        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= CBAUDEX;//BOTHER;
        tio.c_ispeed = speed;
        tio.c_ospeed = speed;
        ioctl(fd, TCSETS2, &tio);
    }

	void Socket::open(const std::string &com_port_name, const size_t baudRate) {
		std::lock_guard<std::mutex> lock(resourceMutex_);
		if (controller_port.is_open()) {
			return;
		}

        // From https://github.com/mavlink/mavros/blob/master/libmavconn/src/serial.cpp
        controller_port.open(com_port_name);

        try {
            // Set baudrate and 8N1 mode
            //controller_port.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
            setCustomBaudRate(controller_port, baudRate);
            controller_port.set_option(boost::asio::serial_port_base::character_size(8));
            controller_port.set_option(
              boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            controller_port.set_option(
              boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        } catch (boost::system::system_error &e) {
            controller_port.close();
            throw e;
        }

#if BOOST_ASIO_VERSION >= 101200 || !defined(__linux__)
        // Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
        serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
#elif BOOST_ASIO_VERSION < 101200 && defined(__linux__)
        // Workaround to set some options for the port manually. This is done in
        // Boost.ASIO, but until v1.12.0 (Boost 1.66) there was a bug which doesn't enable relevant
        // code. Fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
        {
            int fd = controller_port.native_handle();

            termios tio;
            tcgetattr(fd, &tio);

            // Set hardware flow control settings
            bool hwflow_enabled = false;
            if (hwflow_enabled) {
                tio.c_iflag &= ~(IXOFF | IXON);
                tio.c_cflag |= CRTSCTS;
            } else {
                tio.c_iflag &= ~(IXOFF | IXON);
                tio.c_cflag &= ~CRTSCTS;
            }

            // Set serial port to "raw" mode to prevent EOF exit.
            cfmakeraw(&tio);

            // Commit settings
            tcsetattr(fd, TCSANOW, &tio);
        }
#endif

#if defined(__linux__)
        // Enable low latency mode on Linux
        {
            int fd = controller_port.native_handle();

            struct serial_struct ser_info;
            ioctl(fd, TIOCGSERIAL, &ser_info);

            ser_info.flags |= ASYNC_LOW_LATENCY;

            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
#endif

		if (!controller_port.is_open()) return;

		Flush();

        /*boost::asio::async_read(
                controller_port, boost::asio::buffer(read_buffer),
                std::bind(&Socket::processSerial, this, std::placeholders::_1,
                          std::placeholders::_2));*/
        controller_port.async_read_some(boost::asio::buffer(read_buffer),
                                        std::bind(&Socket::processSerial, this, std::placeholders::_1,
                                                  std::placeholders::_2));

		// Start the I/O service in its own thread.
		ioservice_thread = std::thread([&] { ioservice.run(); });
	}

	bool Socket::isOpen() {
		std::lock_guard<std::mutex> lock(resourceMutex_);
		return controller_port.is_open();
	}

	// Send a package with lspc
	//
	// @brief Sends a packaged buffer over the USB serial link.
	//
	// @param type The message type. This is user specific; any type between
	// 1-255.
	// @param payload A vector with the serialized payload to be sent.
	//
	// @return True if the packet was sent.
	bool Socket::send(uint8_t type, const std::vector<uint8_t> &payload) {
		std::lock_guard<std::mutex> lock(resourceMutex_);
		Packet outPacket(type, payload);

		if (!serial_is_sending) {
			serial_is_sending = true;
			boost::asio::async_write(
					controller_port, boost::asio::buffer(outPacket.encodedBuffer()),
					std::bind(&Socket::serialWriteCallback, this, std::placeholders::_1,
							  std::placeholders::_2));
		} else {
			return false;
		}

		return true;
	}

} // namespace end
