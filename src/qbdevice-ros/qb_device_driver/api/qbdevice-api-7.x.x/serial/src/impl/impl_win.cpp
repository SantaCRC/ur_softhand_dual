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

#if defined(_WIN32)

#include <serial/impl/impl.h>

using namespace serial;

Serial::SerialImpl::SerialImpl(std::string port, unsigned long baudrate, Timeout timeout, bytesize_t bytesize,
                               parity_t parity, stopbits_t stopbits, flowcontrol_t flowcontrol)
    : port_(std::move(port)),
      fd_(INVALID_HANDLE_VALUE),
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

  std::string escaped_port = escape(port_);
  fd_ = ::CreateFileW(std::wstring(escaped_port.begin(), escaped_port.end()).c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (fd_ == INVALID_HANDLE_VALUE) {
    throw SerialIOException("failure during ::CreateFileW()", ::GetLastError());
  }

  reconfigurePort();
  is_open_ = true;
}

void Serial::SerialImpl::reconfigurePort() {
  if (fd_ == INVALID_HANDLE_VALUE) {
    throw SerialInvalidArgumentException("invalid file descriptor");
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!::GetCommState(fd_, &dcbSerialParams)) {
    throw SerialIOException("failure during ::GetCommState()", ::GetLastError());
  }

  dcbSerialParams.BaudRate = baudrate_;

  switch (bytesize_) {
    case eightbits: dcbSerialParams.ByteSize = 8; break;
    case sevenbits: dcbSerialParams.ByteSize = 7; break;
    case sixbits: dcbSerialParams.ByteSize = 6; break;
    case fivebits: dcbSerialParams.ByteSize = 5; break;
    default:
      throw SerialInvalidArgumentException("invalid byte size");
  }

  switch (stopbits_) {
    case stopbits_one: dcbSerialParams.StopBits = ONESTOPBIT; break;
    case stopbits_one_point_five: dcbSerialParams.StopBits = ONE5STOPBITS; break;
    case stopbits_two: dcbSerialParams.StopBits = TWOSTOPBITS; break;
    default:
      throw SerialInvalidArgumentException("invalid stop bit");
  }

  switch (parity_) {
    case parity_none: dcbSerialParams.Parity = NOPARITY; break;
    case parity_even: dcbSerialParams.Parity = EVENPARITY; break;
    case parity_odd: dcbSerialParams.Parity = ODDPARITY; break;
    case parity_mark: dcbSerialParams.Parity = MARKPARITY; break;
    case parity_space: dcbSerialParams.Parity = SPACEPARITY; break;
    default:
      throw SerialInvalidArgumentException("invalid parity");
  }

  //TODO: missing fOutxDsrFlow and fDtrControl (and many others)
  switch (flowcontrol_) {
    case flowcontrol_none:
      dcbSerialParams.fOutxCtsFlow = false;
      dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
      dcbSerialParams.fOutX = false;
      dcbSerialParams.fInX = false;
      break;
    case flowcontrol_software:
      dcbSerialParams.fOutxCtsFlow = false;
      dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
      dcbSerialParams.fOutX = true;
      dcbSerialParams.fInX = true;
      break;
    case flowcontrol_hardware:
      dcbSerialParams.fOutxCtsFlow = true;
      dcbSerialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
      dcbSerialParams.fOutX = false;
      dcbSerialParams.fInX = false;
      break;
    default:
      throw SerialInvalidArgumentException("invalid flow control");
  }

  if (!::SetCommState(fd_, &dcbSerialParams)) {
    throw SerialIOException("failure during ::SetCommState()", ::GetLastError());
  }

  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = timeout_.getInterByteMilliseconds();
  timeouts.ReadTotalTimeoutConstant = timeout_.getReadConstantMilliseconds();
  timeouts.ReadTotalTimeoutMultiplier = timeout_.getReadMultiplierMilliseconds();
  timeouts.WriteTotalTimeoutConstant = timeout_.getWriteConstantMilliseconds();
  timeouts.WriteTotalTimeoutMultiplier = timeout_.getWriteMultiplierMilliseconds();
  if (!::SetCommTimeouts(fd_, &timeouts)) {
    throw SerialIOException("failure during ::SetCommTimeouts()", ::GetLastError());
  }
}

void Serial::SerialImpl::close() {
  if (is_open_ && fd_ != INVALID_HANDLE_VALUE && !::CloseHandle(fd_)) {
    throw SerialIOException("failure during ::CloseHandle()", ::GetLastError());
  }
  fd_ = INVALID_HANDLE_VALUE;
  is_open_ = false;
}

bool Serial::SerialImpl::isOpen() const {
  return is_open_;
}

size_t Serial::SerialImpl::available() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  COMSTAT stat;
  if (!::ClearCommError(fd_, nullptr, &stat)) {
    throw SerialIOException("during ::ClearCommError()", ::GetLastError());
  }
  return static_cast<size_t>(stat.cbInQue);
}

bool Serial::SerialImpl::waitReadable(std::chrono::milliseconds timeout_ms) const {
  throw SerialException("waitReadable() is not implemented on Windows");
  //TODO: have a deeper look at WaitForSingleObject
}

bool Serial::SerialImpl::waitWritable(std::chrono::milliseconds timeout_ms) const {
  throw SerialException("waitWritable() is not implemented on Windows");
  //TODO: have a deeper look at WaitForSingleObject
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
  DWORD bytes_read = 0;
  if (!::ReadFile(fd_, buf, static_cast<DWORD>(size), &bytes_read, nullptr)) {
    throw SerialIOException("during ::ReadFile()", ::GetLastError());
  }
  return static_cast<size_t>(bytes_read);
}

size_t Serial::SerialImpl::write(const uint8_t *data, size_t size) {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD bytes_written = 0;
  if (!::WriteFile(fd_, data, static_cast<DWORD>(size), &bytes_written, nullptr)) {
    throw SerialIOException("during ::WriteFile()", ::GetLastError());
  }
  return static_cast<size_t>(bytes_written);
}

void Serial::SerialImpl::setPort(const std::string &port) {
  port_ = port;
}

std::string Serial::SerialImpl::getPort() const {
  return port_;
}

void Serial::SerialImpl::setTimeout(const Timeout &timeout) {
  timeout_ = timeout;
  if (is_open_) {
    reconfigurePort();
  }
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
  if (!::PurgeComm(fd_, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
    throw SerialIOException("failure during ::EscapeCommFunction()");
  }
}

void Serial::SerialImpl::flushInput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (!::PurgeComm(fd_, PURGE_RXCLEAR)) {
    throw SerialIOException("failure during ::EscapeCommFunction()");
  }
}

void Serial::SerialImpl::flushOutput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (!::PurgeComm(fd_, PURGE_TXCLEAR)) {
    throw SerialIOException("failure during ::EscapeCommFunction()");
  }
}

void Serial::SerialImpl::sendBreak(int duration_ms) const {
  throw SerialException("sendBreak() is not implemented on Windows");
}

void Serial::SerialImpl::setBreak(bool level) const {
  setModemStatus(level ? SETBREAK : CLRBREAK);
}

void Serial::SerialImpl::setModemStatus(uint32_t request, uint32_t command) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (!::EscapeCommFunction(fd_, request)) {
    throw SerialIOException("failure during ::EscapeCommFunction()");
  }
}

void Serial::SerialImpl::setRTS(bool level) const {
  setModemStatus(level ? SETRTS : CLRRTS);
}

void Serial::SerialImpl::setDTR(bool level) const {
  setModemStatus(level ? SETDTR : CLRDTR);
}

void Serial::SerialImpl::waitForModemChanges() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (!::SetCommMask(fd_, EV_CTS | EV_DSR | EV_RING | EV_RLSD)) {
    throw SerialIOException("failure during ::SetCommMask()");
  }
  DWORD event;
  if (!::WaitCommEvent(fd_, &event, nullptr)) {
    throw SerialIOException("failure during ::WaitCommEvent()");
  }
}

uint32_t Serial::SerialImpl::getModemStatus() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD modem_status;
  if (!::GetCommModemStatus(fd_, &modem_status)) {
    throw SerialIOException("failure during ::GetCommModemStatus()");
  }
  return modem_status;
}

bool Serial::SerialImpl::getCTS() const {
  return getModemStatus() & MS_CTS_ON;
}

bool Serial::SerialImpl::getDSR() const {
  return getModemStatus() & MS_DSR_ON;
}

bool Serial::SerialImpl::getRI() const {
  return getModemStatus() & MS_RING_ON;
}

bool Serial::SerialImpl::getCD() const {
  return getModemStatus() & MS_RLSD_ON;
}

#endif  // defined(_WIN32)
