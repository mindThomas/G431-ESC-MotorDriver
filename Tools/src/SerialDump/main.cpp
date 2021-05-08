#include <iostream>
#include <thread> // for ioservice thread
#include <string.h> // for memchr
#include <fstream>      // for file output (std::ofstream)
#include <iomanip>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/optional.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <utils.hpp>

// OBS! You can change between async_read and async_read_some depending on whether to block 
// and wait for the buffer to be filled up or run the callback with whatever data 
// that is available at the time of reading
// See https://stackoverflow.com/a/28509826

// Boost::asio serial examples:
// - https://github.com/anton-matosov/SerialPort
// - https://objectcomputing.com/resources/publications/mnb/multi-platform-serial-interfacing-using-boost-a-gps-sensor-and-opendds-part-i

// Serial port
boost::asio::io_service ioservice;
boost::asio::serial_port port = boost::asio::serial_port(ioservice);
std::thread ioservice_thread;
const bool hwflow{false};
std::array<uint8_t, 100> read_buffer;

void ProcessIncomingBytes(uint8_t * buffer, size_t bytes_received);

void ReadCallback(const boost::system::error_code &error, std::size_t bytes_transferred)
{    
    if (error == boost::system::errc::operation_canceled || error == boost::asio::error::eof) {
        // EOF error
        //std::cout << "EOF error" << std::endl;
        //port.async_read_some(boost::asio::buffer(read_buffer),
        //                      std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));        
        boost::asio::async_read(port, boost::asio::buffer(read_buffer),
                                std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));        
        return;
    } else if (error) {
        //throw std::runtime_error("serial asio: " + error.message());
        std::cout << "Serial Callback error: " << error.message() << std::endl;
        return;
    }

    ProcessIncomingBytes(&read_buffer[0], bytes_transferred);

    // READ THE NEXT PACKET
    port.async_read_some(boost::asio::buffer(read_buffer),
                          std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));
    //boost::asio::async_read(port, boost::asio::buffer(read_buffer),
    //                        std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));

    return;
}

void WriteCallback(const boost::system::error_code &error, size_t bytes_transferred) {    
}

template <typename SyncReadStream, typename MutableBufferSequence>
void readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time)
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


boost::system::error_code Flush() {
    // From http://mnb.ociweb.com/mnb/MiddlewareNewsBrief-201303.html
#if 0
    boost::system::error_code ec;
#if !defined(BOOST_WINDOWS) && !defined(__CYGWIN__)
    const bool isFlushed =! ::tcflush(controller_port.native(), TCIOFLUSH);
    if (!isFlushed)
        ec = boost::system::error_code(errno,
                                       boost::asio::error::get_system_category());
#else
    const bool isFlushed = ::PurgeComm(port.native(),
    PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
if (!isFlushed)
    ec = boost::system::error_code(::GetLastError(),
        boost::asio::error::get_system_category());
#endif
    return ec;
#else
    bool continueReading = true;
    while (continueReading) {
        try
        {
            readWithTimeout(port, boost::asio::buffer(read_buffer), boost::posix_time::milliseconds(1));
        }
        catch (boost::system::system_error& e)
        {
            continueReading = false;
        }
    }
#endif
}

/* extended termios struct for custom baud rate */
// Taken from https://code.woboq.org/qt5/qtserialport/src/serialport/qserialport_unix.cpp.html#_ZL15qt_set_databitsP7termiosN11QSerialPort8DataBitsE
struct termios2 {
    tcflag_t c_iflag;       /* input mode flags */
    tcflag_t c_oflag;       /* output mode flags */
    tcflag_t c_cflag;       /* control mode flags */
    tcflag_t c_lflag;       /* local mode flags */
    cc_t c_line;            /* line discipline */
    cc_t c_cc[19];          /* control characters */
    //cc_t c_cc[NCCS];		/* control characters */
    speed_t c_ispeed;       /* input speed */
    speed_t c_ospeed;       /* output speed */
};
#ifndef TCGETS2
#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#endif
#ifndef BOTHER
#define BOTHER      0010000
#endif

void setCustomBaudRate(boost::asio::serial_port& p, const size_t speed)
{
    // See https://stackoverflow.com/questions/12646324/how-can-i-set-a-custom-baud-rate-on-linux
    // And https://gist.github.com/kennethryerson/f7d1abcf2633b7c03cf0
    // And http://www.hce-engineering.com/setting-custom-baud-rate-on-serial-port/
    int fd = p.native_handle();

    struct termios2 tio2;
    ioctl(fd, TCGETS2, &tio2);
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;
    tio2.c_ispeed = speed;
    tio2.c_ospeed = speed;
    ioctl(fd, TCSETS2, &tio2);
}

#if 0
static inline void qt_set_common_props(termios *tio, QIODevice::OpenMode m)
{
#ifdef Q_OS_SOLARIS
    tio->c_iflag &= ~(IMAXBEL|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    tio->c_oflag &= ~OPOST;
    tio->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    tio->c_cflag &= ~(CSIZE|PARENB);
    tio->c_cflag |= CS8;
#else
    ::cfmakeraw(tio);
#endif
    tio->c_cflag |= CLOCAL;
    tio->c_cc[VTIME] = 0;
    tio->c_cc[VMIN] = 0;
    if (m & QIODevice::ReadOnly)
        tio->c_cflag |= CREAD;
}
static inline void qt_set_databits(termios *tio, QSerialPort::DataBits databits)
{
    tio->c_cflag &= ~CSIZE;
    switch (databits) {
        case QSerialPort::Data5:
            tio->c_cflag |= CS5;
            break;
        case QSerialPort::Data6:
            tio->c_cflag |= CS6;
            break;
        case QSerialPort::Data7:
            tio->c_cflag |= CS7;
            break;
        case QSerialPort::Data8:
            tio->c_cflag |= CS8;
            break;
        default:
            tio->c_cflag |= CS8;
            break;
    }
}
static inline void qt_set_parity(termios *tio, QSerialPort::Parity parity)
{
    tio->c_iflag &= ~(PARMRK | INPCK);
    tio->c_iflag |= IGNPAR;
    switch (parity) {
#ifdef CMSPAR
        // Here Installation parity only for GNU/Linux where the macro CMSPAR.
        case QSerialPort::SpaceParity:
            tio->c_cflag &= ~PARODD;
            tio->c_cflag |= PARENB | CMSPAR;
            break;
        case QSerialPort::MarkParity:
            tio->c_cflag |= PARENB | CMSPAR | PARODD;
            break;
#endif
        case QSerialPort::NoParity:
            tio->c_cflag &= ~PARENB;
            break;
        case QSerialPort::EvenParity:
            tio->c_cflag &= ~PARODD;
            tio->c_cflag |= PARENB;
            break;
        case QSerialPort::OddParity:
            tio->c_cflag |= PARENB | PARODD;
            break;
        default:
            tio->c_cflag |= PARENB;
            tio->c_iflag |= PARMRK | INPCK;
            tio->c_iflag &= ~IGNPAR;
            break;
    }
}
static inline void qt_set_stopbits(termios *tio, QSerialPort::StopBits stopbits)
{
    switch (stopbits) {
        case QSerialPort::OneStop:
            tio->c_cflag &= ~CSTOPB;
            break;
        case QSerialPort::TwoStop:
            tio->c_cflag |= CSTOPB;
            break;
        default:
            tio->c_cflag &= ~CSTOPB;
            break;
    }
}
static inline void qt_set_flowcontrol(termios *tio, QSerialPort::FlowControl flowcontrol)
{
    switch (flowcontrol) {
        case QSerialPort::NoFlowControl:
            tio->c_cflag &= ~CRTSCTS;
            tio->c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
        case QSerialPort::HardwareControl:
            tio->c_cflag |= CRTSCTS;
            tio->c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
        case QSerialPort::SoftwareControl:
            tio->c_cflag &= ~CRTSCTS;
            tio->c_iflag |= IXON | IXOFF | IXANY;
            break;
        default:
            tio->c_cflag &= ~CRTSCTS;
            tio->c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
    }
}

bool QSerialPortPrivate::setStandardBaudRate(qint32 baudRate, QSerialPort::Directions directions)
{
#ifdef Q_OS_LINUX
    // try to clear custom baud rate, using termios v2
    struct termios2 tio2;
    if (::ioctl(descriptor, TCGETS2, &tio2) != -1) {
        if (tio2.c_cflag & BOTHER) {
            tio2.c_cflag &= ~BOTHER;
            tio2.c_cflag |= CBAUD;
            ::ioctl(descriptor, TCSETS2, &tio2);
        }
    }
    // try to clear custom baud rate, using serial_struct (old way)
    struct serial_struct serial;
    ::memset(&serial, 0, sizeof(serial));
    if (::ioctl(descriptor, TIOCGSERIAL, &serial) != -1) {
        if (serial.flags & ASYNC_SPD_CUST) {
            serial.flags &= ~ASYNC_SPD_CUST;
            serial.custom_divisor = 0;
            // we don't check on errors because a driver can has not this feature
            ::ioctl(descriptor, TIOCSSERIAL, &serial);
        }
    }
#endif
    termios tio;
    if (!getTermios(&tio))
        return false;
    if ((directions & QSerialPort::Input) && ::cfsetispeed(&tio, baudRate) < 0) {
        setError(getSystemError());
        return false;
    }
    if ((directions & QSerialPort::Output) && ::cfsetospeed(&tio, baudRate) < 0) {
        setError(getSystemError());
        return false;
    }
    return setTermios(&tio);
}

bool setCustomBaudRate(boost::asio::serial_port& p, int32_t baudRate)
{
    int fd = p.native_handle();

    struct termios2 tio2;
    if (::ioctl(fd, TCGETS2, &tio2) != -1) {
        tio2.c_cflag &= ~CBAUD;
        tio2.c_cflag |= BOTHER;
        tio2.c_ispeed = baudRate;
        tio2.c_ospeed = baudRate;
        if (::ioctl(fd, TCSETS2, &tio2) != -1
                && ::ioctl(fd, TCGETS2, &tio2) != -1) {
            return true;
        }
    }
    struct serial_struct serial;
    if (::ioctl(fd, TIOCGSERIAL, &serial) == -1) {
        setError(getSystemError());
        return false;
    }
    serial.flags &= ~ASYNC_SPD_MASK;
    serial.flags |= (ASYNC_SPD_CUST /* | ASYNC_LOW_LATENCY*/);
    serial.custom_divisor = serial.baud_base / baudRate;
    if (serial.custom_divisor == 0) {
        setError(QSerialPortErrorInfo(QSerialPort::UnsupportedOperationError,
                                      QSerialPort::tr("No suitable custom baud rate divisor")));
        return false;
    }
    if (serial.custom_divisor * baudRate != serial.baud_base) {
        qWarning("Baud rate of serial port %s is set to %f instead of %d: divisor %f unsupported",
            qPrintable(systemLocation),
            float(serial.baud_base) / serial.custom_divisor,
            baudRate, float(serial.baud_base) / baudRate);
    }
    if (::ioctl(fd, TIOCSSERIAL, &serial) == -1) {
        setError(getSystemError());
        return false;
    }
    return setCustomBaudRate(p, B38400);
}

bool QSerialPortPrivate::setBaudRate(qint32 baudRate, QSerialPort::Directions directions)
{
    if (baudRate <= 0) {
        setError(QSerialPortErrorInfo(QSerialPort::UnsupportedOperationError, QSerialPort::tr("Invalid baud rate value")));
        return false;
    }
    const qint32 unixBaudRate = QSerialPortPrivate::settingFromBaudRate(baudRate);
    return (unixBaudRate > 0)
           ? setStandardBaudRate(unixBaudRate, directions)
           : setCustomBaudRate(baudRate, directions);
}
bool QSerialPortPrivate::setDataBits(QSerialPort::DataBits dataBits)
{
    termios tio;
    if (!getTermios(&tio))
        return false;
    qt_set_databits(&tio, dataBits);
    return setTermios(&tio);
}
bool QSerialPortPrivate::setParity(QSerialPort::Parity parity)
{
    termios tio;
    if (!getTermios(&tio))
        return false;
    qt_set_parity(&tio, parity);
    return setTermios(&tio);
}
bool QSerialPortPrivate::setStopBits(QSerialPort::StopBits stopBits)
{
    termios tio;
    if (!getTermios(&tio))
        return false;
    qt_set_stopbits(&tio, stopBits);
    return setTermios(&tio);
}
bool QSerialPortPrivate::setFlowControl(QSerialPort::FlowControl flowControl)
{
    termios tio;
    if (!getTermios(&tio))
        return false;
    qt_set_flowcontrol(&tio, flowControl);
    return setTermios(&tio);
}
#endif

void Connect(std::string port_name, uint32_t baud_rate)
{
    // From https://github.com/mavlink/mavros/blob/master/libmavconn/src/serial.cpp
    try {
        port.open(port_name);

        // Set baudrate and 8N1 mode
        //port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        setCustomBaudRate(port, baud_rate);
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

#if BOOST_ASIO_VERSION >= 101200 || !defined(__linux__)
        // Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
        serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
#elif BOOST_ASIO_VERSION < 101200 && defined(__linux__)
        // Workaround to set some options for the port manually. This is done in
        // Boost.ASIO, but until v1.12.0 (Boost 1.66) there was a bug which doesn't enable relevant
        // code. Fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
        {
            int fd = port.native_handle();

            termios tio;
            tcgetattr(fd, &tio);

            // Set hardware flow control settings
            if (hwflow) {
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
            int fd = port.native_handle();

            struct serial_struct ser_info;
            ioctl(fd, TIOCGSERIAL, &ser_info);

            ser_info.flags |= ASYNC_LOW_LATENCY;

            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
#endif
    }
    catch (boost::system::system_error &err) {
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "Error: " << std::string(err.what()) << std::endl;
        exit(-1);        
    }

    if (!port.is_open()) return;

    Flush();

    port.async_read_some(boost::asio::buffer(read_buffer),
                         std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));
    //boost::asio::async_read(port, boost::asio::buffer(read_buffer),
    //                    std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));
    // Start the I/O service in its own thread.
    ioservice_thread = std::thread([&] { ioservice.run(); });

    // Wait for start
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Disconnect()
{
    ioservice.stop();

    if (ioservice_thread.joinable()) {
        ioservice_thread.join();
    }

    try {
        port.close();
    }
    catch (boost::system::system_error &err) {
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "Error: " << std::string(err.what()) << std::endl;
        exit(-1);
    }
}

bool isOpen() {
    return port.is_open();
}

void send(std::vector<uint8_t> buffer) {
    // Consider adding a mutex
    boost::asio::async_write(
            port, boost::asio::buffer(buffer),
            std::bind(&WriteCallback, std::placeholders::_1,
                      std::placeholders::_2));
}

// Should exit signals
utils::EventSignal exit_signal;
void exitHandler(int s) {
    std::cout << "Exiting..." << std::endl;
    //should_exit = true;
    exit_signal.trigger();
}

int main(int argc, char* argv[]) 
{ 
    signal(SIGINT, exitHandler);

    Connect("/dev/ttyACM0", 1612800); // 1612800
    exit_signal.waitForEvent();
    Disconnect();    
} 

void ProcessIncomingBytes(uint8_t * buffer, size_t bytes_received)
{
#if 0
    uint8_t * start = (uint8_t*)memchr((void*)buffer, 0xFE, bytes_received);
    if (start > 0) { 
        if ((start - buffer) + 9 < bytes_received) {
            if (start[8] == 0xAA) {
                for (unsigned int i = 0; i < 9; i++) {
                    printf("0x%x\t", *start++);
                }
                printf("\n");
            }
        }
    }
#endif
    std::stringstream ss;
    ss << std::setfill('0');
    for (unsigned int i = 0; i < bytes_received; i++) {
        ss << " 0x" << std::setw(2) << std::uppercase << std::hex << static_cast<int>(*buffer++);
    }
    std::cout << ss.str();
    std::cout << std::endl;
}