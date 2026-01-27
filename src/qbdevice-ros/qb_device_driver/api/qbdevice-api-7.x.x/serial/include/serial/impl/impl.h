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

#ifndef SERIAL_IMPL_H
#define SERIAL_IMPL_H

#include <serial/serial.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

#if defined(__linux__)
#include <linux/serial.h>  //TODO: maybe termios2 could be an alternative
#elif defined(__MACH__)
#include <AvailabilityMacros.h>
#include <mach/clock.h>
#include <mach/mach.h>
#if defined(MAC_OS_X_VERSION_10_3) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_3)
#include <IOKit/serial/ioss.h>
#endif
//TODO: check if it works properly on macOS
#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif
#endif
#endif

namespace serial {

#if defined(_WIN32)
inline std::string escape(const std::string &str) {
  std::smatch match;
  std::regex_search(str, match, std::regex(R"(^\\\\.\\)"));
  if (match.empty()) {
    return R"(\\.\)" + str;
  }
  return str;
}
#endif

class Serial::SerialImpl {
 public:
  SerialImpl(std::string port, unsigned long baudrate, Timeout timeout, bytesize_t bytesize, parity_t parity,
             stopbits_t stopbits, flowcontrol_t flowcontrol);

  virtual ~SerialImpl();

  void open();

  void close();

  bool isOpen() const;

  size_t available() const;

  bool waitReadable(std::chrono::milliseconds timeout_ms) const;

  bool waitWritable(std::chrono::milliseconds timeout_ms) const;

  void waitByteTimes(size_t count) const;

  size_t read(uint8_t *buf, size_t size = 1);

  size_t write(const uint8_t *data, size_t size);

  void flush() const;

  void flushInput() const;

  void flushOutput() const;

  void sendBreak(int duration_ms) const;

  void setBreak(bool level) const;

  void setModemStatus(uint32_t request, uint32_t command = 0) const;

  void setRTS(bool level) const;

  void setDTR(bool level) const;

  void waitForModemChanges() const;

  uint32_t getModemStatus() const;

  bool getCTS() const;

  bool getDSR() const;

  bool getRI() const;

  bool getCD() const;

  void setPort(const std::string &port);

  std::string getPort() const;

  void setTimeout(const Timeout &timeout);

  Timeout getTimeout() const;

  void setBaudrate(unsigned long baudrate);

  unsigned long getBaudrate() const;

  void setBytesize(bytesize_t bytesize);

  bytesize_t getBytesize() const;

  void setParity(parity_t parity);

  parity_t getParity() const;

  void setStopbits(stopbits_t stopbits);

  stopbits_t getStopbits() const;

  void setFlowcontrol(flowcontrol_t flowcontrol);

  flowcontrol_t getFlowcontrol() const;

 protected:
  void reconfigurePort();

 private:
#if !defined(_WIN32)
  int fd_;
#else
  HANDLE fd_;
#endif
  std::string port_;
  bool is_open_;
  Timeout timeout_;
  unsigned long baudrate_;
  parity_t parity_;
  bytesize_t bytesize_;
  stopbits_t stopbits_;
  flowcontrol_t flowcontrol_;
};
}  // namespace serial

#endif  // SERIAL_IMPL_H
