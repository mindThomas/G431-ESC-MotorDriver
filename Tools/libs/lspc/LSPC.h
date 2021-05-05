#ifndef LSPC_H
#define LSPC_H

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

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include "lspc/Packet.hpp"
#include "lspc/SocketBase.hpp"

namespace lspc {

  class Socket : public SocketBase {
    // Buffer for receiving data on the serial link.
    std::array<uint8_t, 1> read_buffer;

    // Serial port
    boost::asio::io_service ioservice;
    boost::asio::serial_port controller_port;
    std::thread ioservice_thread;
    std::atomic<bool> serial_is_sending;

    // Lock guard mutex
    std::mutex resourceMutex_;

    /// @brief Different ways a serial port may be flushed.
    enum flush_type
    {
        flush_receive = TCIFLUSH,
        flush_send = TCOFLUSH,
        flush_both = TCIOFLUSH
    };

    // Process incoming data on serial link
    //
    // @brief Reads the serial buffer and dispatches the received payload to the
    // relevant message handling callback function.
    void processSerial(const boost::system::error_code &error, std::size_t bytes_transferred);

    void serialWriteCallback(const boost::system::error_code &error, size_t bytes_transferred);

    template <typename SyncReadStream, typename MutableBufferSequence>
    void readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time);

    boost::system::error_code Flush();
    boost::system::error_code flush_serial_port(boost::asio::serial_port& serial_port, flush_type what);

   public:
    Socket();
    Socket(const std::string &com_port_name, const size_t baudRate = 115200);
    ~Socket();

    void open(const std::string &com_port_name, const size_t baudRate = 115200);

    bool send(uint8_t type, const std::vector<uint8_t> &payload) override;

    bool isOpen();
  };

}  // namespace lspc

#endif  // LSPC_H
