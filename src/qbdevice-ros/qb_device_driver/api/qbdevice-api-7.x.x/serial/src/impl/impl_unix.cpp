/***
 *  MIT License
 *
 *  Copyright (c) 2023 qbrobotics®
 *  Copyright (c) 2020 Alessandro Tondo
 *  Copyright (c) 2012 William Woodall, John Harrison
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 *  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 *  to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 *  the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 *  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#if !defined(_WIN32)

#include <serial/impl/impl.h>

using namespace serial;

Serial::SerialImpl::SerialImpl(std::string port, unsigned long baudrate, Timeout timeout, bytesize_t bytesize,
                               parity_t parity, stopbits_t stopbits, flowcontrol_t flowcontrol)
    : port_(std::move(port)),
      fd_(-1),
      is_open_(false),
      timeout_(timeout),
      baudrate_(baudrate),
      parity_(parity),
      bytesize_(bytesize),
      stopbits_(stopbits),
      flowcontrol_(flowcontrol) {
  if (!port_.empty()) {
    open();
  }
}

Serial::SerialImpl::~SerialImpl() {
  close();
}

void Serial::SerialImpl::open() {
  if (port_.empty()) {
    throw SerialInvalidArgumentException("serial port is empty.");
  }
  if (is_open_) {
    return;
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ == -1) {
    if (errno == EINTR) {
      open();  // Recurse because this is a recoverable error
      return;
    }
    throw SerialIOException("failure during ::open()", errno);
  }

  // prevent multiple openings
  if (::ioctl(fd_, TIOCEXCL) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }

  // set communication as BLOCKING

  if(::fcntl(fd_, F_SETFL, 0) == -1) {
      throw SerialIOException("failure during ::fcntl()", errno);
  }

  reconfigurePort();
  is_open_ = true;
}

void Serial::SerialImpl::reconfigurePort() {
  if (fd_ == -1) {
    throw SerialInvalidArgumentException("invalid file descriptor");
  }

  struct termios options{0};
  if (::tcgetattr(fd_, &options) == -1) {
    throw SerialIOException("failure during ::tcgetattr()", errno);
  }

  // input modes
  options.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
#ifdef IUCLC
  options.c_iflag &= ~IUCLC;
#endif
#ifdef IMAXBEL
  options.c_iflag &= ~IMAXBEL;
#endif

  // output modes
  options.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL);
#ifdef OLCUC
  options.c_oflag &= ~OLCUC;
#endif
#ifdef NLDLY
  options.c_oflag &= ~NLDLY;
#endif
#ifdef CRDLY
  options.c_oflag &= ~CRDLY;
#endif
#ifdef TABDLY
  options.c_oflag &= ~TABDLY;
#endif
#ifdef BSDLY
  options.c_oflag &= ~BSDLY;
#endif
#ifdef VTDLY
  options.c_oflag &= ~VTDLY;
#endif
#ifdef FFDLY
  options.c_oflag &= ~FFDLY;
#endif

  // control modes
  options.c_cflag &= (CSIZE | CSTOPB | PARENB | PARODD);
  options.c_cflag |= (CREAD | HUPCL | CLOCAL);
#ifdef CMSPAR
  options.c_cflag &= ~CMSPAR;
#endif
#ifdef CRTSCTS
  options.c_cflag &= ~CRTSCTS;
#endif

  // local modes
  options.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);
#ifdef ECHOCTL
  options.c_lflag &= ~ECHOCTL;
#endif
#ifdef ECHOPRT
  options.c_lflag &= ~ECHOPRT;
#endif
#ifdef ECHOKE
  options.c_lflag &= ~ECHOKE;
#endif

  // control characters
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  struct serial_struct serinfo;

  ::ioctl(fd_, TIOCGSERIAL, &serinfo);
  serinfo.flags |= ASYNC_LOW_LATENCY;
  ::ioctl(fd_, TIOCSSERIAL, &serinfo);

  bool use_custom_baudrate = false;
  speed_t baudrate = 0;
  switch (baudrate_) {
#ifdef B0
    case 0: baudrate = B0; break;
#endif
#ifdef B50
    case 50: baudrate = B50; break;
#endif
#ifdef B75
    case 75: baudrate = B75; break;
#endif
#ifdef B110
    case 110: baudrate = B110; break;
#endif
#ifdef B134
    case 134: baudrate = B134; break;
#endif
#ifdef B150
    case 150: baudrate = B150; break;
#endif
#ifdef B200
    case 200: baudrate = B200; break;
#endif
#ifdef B300
    case 300: baudrate = B300; break;
#endif
#ifdef B600
    case 600: baudrate = B600; break;
#endif
#ifdef B1200
    case 1200: baudrate = B1200; break;
#endif
#ifdef B1800
    case 1800: baudrate = B1800; break;
#endif
#ifdef B2400
    case 2400: baudrate = B2400; break;
#endif
#ifdef B4800
    case 4800: baudrate = B4800; break;
#endif
#ifdef B7200
    case 7200: baudrate = B7200; break;
#endif
#ifdef B9600
    case 9600: baudrate = B9600; break;
#endif
#ifdef B14400
    case 14400: baudrate = B14400; break;
#endif
#ifdef B19200
    case 19200: baudrate = B19200; break;
#endif
#ifdef B28800
    case 28800: baudrate = B28800; break;
#endif
#ifdef B57600
    case 57600: baudrate = B57600; break;
#endif
#ifdef B76800
    case 76800: baudrate = B76800; break;
#endif
#ifdef B38400
    case 38400: baudrate = B38400; break;
#endif
#ifdef B115200
    case 115200: baudrate = B115200; break;
#endif
#ifdef B128000
    case 128000: baudrate = B128000; break;
#endif
#ifdef B153600
    case 153600: baudrate = B153600; break;
#endif
#ifdef B230400
    case 230400: baudrate = B230400; break;
#endif
#ifdef B256000
    case 256000: baudrate = B256000; break;
#endif
#ifdef B460800
    case 460800: baudrate = B460800; break;
#endif
#ifdef B500000
    case 500000: baudrate = B500000; break;
#endif
#ifdef B576000
    case 576000: baudrate = B576000; break;
#endif
#ifdef B921600
    case 921600: baudrate = B921600; break;
#endif
#ifdef B1000000
    case 1000000: baudrate = B1000000; break;
#endif
#ifdef B1152000
    case 1152000: baudrate = B1152000; break;
#endif
#ifdef B1500000
    case 1500000: baudrate = B1500000; break;
#endif
#ifdef B2000000
    case 2000000: baudrate = B2000000; break;
#endif
#ifdef B2500000
    case 2500000: baudrate = B2500000; break;
#endif
#ifdef B3000000
    case 3000000: baudrate = B3000000; break;
#endif
#ifdef B3500000
    case 3500000: baudrate = B3500000; break;
#endif
#ifdef B4000000
    case 4000000: baudrate = B4000000; break;
#endif
    default:
      baudrate = baudrate_;
      use_custom_baudrate = true;
  }

  if (!use_custom_baudrate) {
    if (::cfsetispeed(&options, baudrate) == -1) {
      throw SerialIOException("failure during ::cfsetispeed()", errno);
    }
    if (::cfsetospeed(&options, baudrate) == -1) {
      throw SerialIOException("failure during ::cfsetospeed()", errno);
    }
  }

  switch (bytesize_) {
    case eightbits: options.c_cflag |= CS8; break;
    case sevenbits: options.c_cflag |= CS7; break;
    case sixbits: options.c_cflag |= CS6; break;
    case fivebits: options.c_cflag |= CS5; break;
    default:
      throw SerialInvalidArgumentException("invalid byte size");
  }

  switch (stopbits_) {
    case stopbits_one: options.c_cflag &= ~CSTOPB; break;
    case stopbits_one_point_five:  // there is no POSIX support for 1.5 stop bit
    case stopbits_two: options.c_cflag |= CSTOPB; break;
    default:
      throw SerialInvalidArgumentException("invalid stop bit");
  }

  switch (parity_) {
    case parity_none: options.c_iflag |= IGNPAR; break;
    case parity_even: options.c_cflag |= PARENB; break;
    case parity_odd: options.c_cflag |= (PARENB | PARODD); break;
#ifdef CMSPAR
    case parity_mark: options.c_cflag |= (PARENB | PARODD | CMSPAR); break;
    case parity_space: options.c_cflag |= (PARENB | CMSPAR); break;
#endif
    default:
      throw SerialInvalidArgumentException("invalid parity");
  }

  switch (flowcontrol_) {
    case flowcontrol_none: break;
    case flowcontrol_software: options.c_iflag |= (IXON | IXOFF | IXANY); break;
#ifdef CRTSCTS
    case flowcontrol_hardware: options.c_cflag |= (CRTSCTS); break;
#endif
    default:
      throw SerialInvalidArgumentException("invalid flow control");
  }

  // activate settings
  if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
    throw SerialIOException("failure during ::tcsetattr()", errno);
  }

  if (use_custom_baudrate) {
#if defined(__APPLE__)
    if (::ioctl(fd_, IOSSIOSPEED, &baudrate) == -1) {
      throw SerialIOException("failure during ::ioctl(IOSSIOSPEED)", errno);
    }
#elif defined(__linux__) && defined (TIOCSSERIAL)  //TODO: maybe termios2 could be an alternative
    struct serial_struct serial {0};
    if (::ioctl(fd_, TIOCGSERIAL, &serial) == -1) {
      throw SerialIOException("failure during ::ioctl()", errno);
    }

    //TODO: check ASYNC_LOW_LATENCY
    serial.flags &= ~ASYNC_SPD_MASK;
    serial.flags |= ASYNC_SPD_CUST;
    serial.custom_divisor = static_cast<int>((serial.baud_base + baudrate_ / 2) / baudrate_);
    int closest_baudrate = serial.baud_base / serial.custom_divisor;
    if (std::abs(static_cast<int>(closest_baudrate - baudrate_)) < 0.02) {
      throw SerialIOException("cannot set baudrate to " + std::to_string(baudrate_) + " (the closest possible value is " + std::to_string(closest_baudrate) + ")");
    }

    if (::ioctl(fd_, TIOCSSERIAL, &serial) == -1) {
      throw SerialIOException("failure during ::ioctl()", errno);
    }
#else
    throw SerialInvalidArgumentException("custom baudrates are not supported on current OS");
#endif
  }
}

void Serial::SerialImpl::close() {
  if (is_open_ && fd_ != -1 && ::close(fd_) != 0) {
    throw SerialIOException("failure during ::close()", errno);
  }
  fd_ = -1;
  is_open_ = false;
}

bool Serial::SerialImpl::isOpen() const {
  return is_open_;
}

size_t Serial::SerialImpl::available() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  int count = 0;
  if (::ioctl(fd_, TIOCINQ, &count) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
  return static_cast<size_t>(count);
}

bool waitOnPoll(std::chrono::milliseconds timeout_ms, std::unique_ptr<pollfd> fds) {
  int r = ::poll(fds.get(), 1, timeout_ms.count());
  if (r < 0 && errno != EINTR) {
    throw SerialIOException("failure during ::poll()", errno);
  }
  if (fds->revents == POLLERR || fds->revents == POLLHUP || fds->revents == POLLNVAL) {
    throw SerialIOException("failure during ::poll(), revents has been set to '" + std::to_string(fds->revents) + "'.");
  }
  return r > 0;
}

bool Serial::SerialImpl::waitReadable(std::chrono::milliseconds timeout_ms) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  return waitOnPoll(timeout_ms, std::unique_ptr<pollfd>(new pollfd{fd_, POLLIN, 0}));
}

bool Serial::SerialImpl::waitWritable(std::chrono::milliseconds timeout_ms) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  return waitOnPoll(timeout_ms, std::unique_ptr<pollfd>(new pollfd{fd_, POLLOUT, 0}));
}

void Serial::SerialImpl::waitByteTimes(size_t count) const {
  auto start = std::chrono::steady_clock::now();
  uint32_t bit_time_ns = 1e9 / baudrate_;
  uint32_t byte_time_ns = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);
  byte_time_ns += stopbits_ == stopbits_one_point_five ? -1.5*bit_time_ns : 0;  // stopbits_one_point_five is 3
  std::this_thread::sleep_until(start + std::chrono::nanoseconds(byte_time_ns * count));
}

size_t Serial::SerialImpl::read(uint8_t *buf, size_t size) {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  auto read_deadline = timeout_.getReadDeadline(size);
  size_t total_bytes_read = ::read(fd_, buf, size);
  if (total_bytes_read == -1) {
    throw SerialIOException("failure during ::read()", errno);
  }
  while (total_bytes_read < size) {
    auto remaining_time = Timeout::remainingMicroseconds(read_deadline);
    if (remaining_time.count() <= 0) {
      break;
    }
    if (waitReadable(std::min(std::chrono::duration_cast<std::chrono::milliseconds>(remaining_time), timeout_.getInterByte()))) {
      if (size - total_bytes_read > 1 && timeout_.getInterByteMilliseconds() == std::numeric_limits<uint32_t>::max()) {
        size_t bytes_available = available();
        if (bytes_available + total_bytes_read < size) {
          waitByteTimes(size - (bytes_available + total_bytes_read));
        }
      }
      size_t bytes_read = ::read(fd_, buf + total_bytes_read, size - total_bytes_read);
      if (bytes_read < 1) {  // at least one byte is for sure available
        throw SerialIOException("failure during ::read()", errno);
      }
      total_bytes_read += bytes_read;
    }
  }
  return total_bytes_read;
}

size_t Serial::SerialImpl::write(const uint8_t *data, size_t size) {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  auto write_deadline = timeout_.getWriteDeadline(size);
  size_t total_bytes_written = ::write(fd_, data, size);
  if (total_bytes_written == -1) {
    throw SerialIOException("failure during ::write()", errno);
  }
  while (total_bytes_written < size) {
    auto remaining_time = Timeout::remainingMicroseconds(write_deadline);
    if (remaining_time.count() <= 0) {
      break;
    }
    if (waitWritable(std::chrono::duration_cast<std::chrono::milliseconds>(remaining_time))) {
      size_t bytes_written = ::write(fd_, data + total_bytes_written, size - total_bytes_written);
      if (bytes_written < 1) {  // at least one byte is for sure available
        throw SerialIOException("failure during ::write()", errno);
      }
      total_bytes_written += bytes_written;
    }
  }
  return total_bytes_written;
}

void Serial::SerialImpl::setPort(const std::string &port) {
  port_ = port;
}

std::string Serial::SerialImpl::getPort() const {
  return port_;
}

void Serial::SerialImpl::setTimeout(const Timeout &timeout) {
  timeout_ = timeout;  // timeout is used directly inside read() and write(): there is no need to call reconfigurePort()
}

Serial::Timeout Serial::SerialImpl::getTimeout() const {
  return timeout_;
}

void Serial::SerialImpl::setBaudrate(unsigned long baudrate) {
  baudrate_ = baudrate;
  if (is_open_) {
    reconfigurePort();
  }
}

unsigned long Serial::SerialImpl::getBaudrate() const {
  return baudrate_;
}

void Serial::SerialImpl::setBytesize(serial::bytesize_t bytesize) {
  bytesize_ = bytesize;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::bytesize_t Serial::SerialImpl::getBytesize() const {
  return bytesize_;
}

void Serial::SerialImpl::setParity(serial::parity_t parity) {
  parity_ = parity;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::parity_t Serial::SerialImpl::getParity() const {
  return parity_;
}

void Serial::SerialImpl::setStopbits(serial::stopbits_t stopbits) {
  stopbits_ = stopbits;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::stopbits_t Serial::SerialImpl::getStopbits() const {
  return stopbits_;
}

void Serial::SerialImpl::setFlowcontrol(serial::flowcontrol_t flowcontrol) {
  flowcontrol_ = flowcontrol;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::flowcontrol_t Serial::SerialImpl::getFlowcontrol() const {
  return flowcontrol_;
}

void Serial::SerialImpl::flush() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcflush(fd_, TCIOFLUSH) == -1) {
    throw SerialIOException("failure during ::tcflush()", errno);
  }
}

void Serial::SerialImpl::flushInput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcflush(fd_, TCIFLUSH) == -1) {
    throw SerialIOException("failure during ::tcflush()", errno);
  }
}

void Serial::SerialImpl::flushOutput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcflush(fd_, TCOFLUSH) == -1) {
    throw SerialIOException("failure during ::tcflush()", errno);
  }
}

void Serial::SerialImpl::sendBreak(int duration_ms) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcsendbreak(fd_, duration_ms) == -1) {
    throw SerialIOException("failure during ::tcsendbreak()", errno);
  }
}

void Serial::SerialImpl::setBreak(bool level) const {
  setModemStatus(level ? TIOCSBRK : TIOCCBRK);
}

void Serial::SerialImpl::setModemStatus(uint32_t request, uint32_t command) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::ioctl(fd_, request, &command) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
}

void Serial::SerialImpl::setRTS(bool level) const {
  setModemStatus(level ? TIOCMBIS : TIOCMBIC, TIOCM_RTS);
}

void Serial::SerialImpl::setDTR(bool level) const {
  setModemStatus(level ? TIOCMBIS : TIOCMBIC, TIOCM_DTR);
}

void Serial::SerialImpl::waitForModemChanges() const {
#ifndef TIOCMIWAIT
  throw SerialException("TIOCMIWAIT is not defined");
#else
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  // cannot use setModemStatus(): TIOCMIWAIT requires arg by value (not by pointer)
  if (::ioctl(fd_, TIOCMIWAIT, TIOCM_CTS | TIOCM_DSR | TIOCM_RI | TIOCM_CD) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
#endif
}

uint32_t Serial::SerialImpl::getModemStatus() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  uint32_t modem_status;
  if (::ioctl(fd_, TIOCMGET, &modem_status) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
  return modem_status;
}

bool Serial::SerialImpl::getCTS() const {
  return getModemStatus() & TIOCM_CTS;
}

bool Serial::SerialImpl::getDSR() const {
  return getModemStatus() & TIOCM_DSR;
}

bool Serial::SerialImpl::getRI() const {
  return getModemStatus() & TIOCM_RI;
}

bool Serial::SerialImpl::getCD() const {
  return getModemStatus() & TIOCM_CD;
}

#endif  // !defined(_WIN32)
