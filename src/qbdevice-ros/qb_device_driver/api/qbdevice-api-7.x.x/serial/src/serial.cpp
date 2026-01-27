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

#include <serial/serial.h>
#include <serial/impl/impl.h>

using namespace serial;

Serial::Serial(const std::string &port_name, uint32_t baudrate, Serial::Timeout timeout, bytesize_t bytesize, parity_t parity,
               stopbits_t stopbits, flowcontrol_t flowcontrol)
    : pimpl_(new SerialImpl(port_name, baudrate, timeout, bytesize, parity, stopbits, flowcontrol)) {

}

Serial::~Serial() {
// pimpl_ is automatically destroyed and close() is called inside pimpl_ destructor
}

void Serial::open() {
  pimpl_->open();
}

void Serial::close() {
  pimpl_->close();
}

bool Serial::isOpen() const {
  return pimpl_->isOpen();
}

size_t Serial::available() {
  return pimpl_->available();
}

bool Serial::waitReadable() {
  return pimpl_->waitReadable(pimpl_->getTimeout().getReadConstant());
}

bool Serial::waitWritable() {
  return pimpl_->waitWritable(pimpl_->getTimeout().getWriteConstant());
}

void Serial::waitByteTimes(size_t count) {
  pimpl_->waitByteTimes(count);
}

size_t Serial::read(uint8_t *buffer, size_t size) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  return pimpl_->read(buffer, size);
}

size_t Serial::read(std::vector<uint8_t> &buffer, size_t size) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  std::unique_ptr<uint8_t[]> buf(new uint8_t[size]);
  size_t bytes_read = pimpl_->read(buf.get(), size);
  buffer.insert(buffer.end(), buf.get(), buf.get() + bytes_read);
  return bytes_read;
}

size_t Serial::read(std::string &buffer, size_t size) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  std::unique_ptr<uint8_t[]> buf(new uint8_t[size]);
  size_t bytes_read = pimpl_->read(buf.get(), size);
  buffer.append(reinterpret_cast<const char *>(buf.get()), bytes_read);
  return bytes_read;
}

std::string Serial::read(size_t size) {
  std::string buffer;
  read(buffer, size);
  return buffer;
}

size_t Serial::readline_(std::string &line, size_t size, const std::string &eol) {
  std::unique_ptr<uint8_t[]> buffer(new uint8_t[size]);
  size_t eol_len = eol.length();
  size_t read_so_far = 0;
  while (read_so_far < size) {
    size_t bytes_read = pimpl_->read(buffer.get() + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break; // Timeout occurred on reading 1 byte
    }
    if (read_so_far >= eol_len && std::string(reinterpret_cast<const char *>(buffer.get() + read_so_far - eol_len), eol_len) == eol) {
      break; // EOL found
    }
  }
  line.append(reinterpret_cast<const char *>(buffer.get()), read_so_far);
  return read_so_far;
}

size_t Serial::readline(std::string &line, size_t size, const std::string &eol) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  return readline_(line, size, eol);
}

std::string Serial::readline(size_t size, const std::string &eol) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  std::string line;
  readline_(line, size, eol);
  return line;
}

std::vector<std::string> Serial::readlines(size_t size, const std::string &eol) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  std::vector<std::string> lines;
  std::string line;
  while (readline_(line, size, eol) > 0) {
    lines.push_back(line);
    size -= line.size();
    line.clear();
  }
  return lines;
}

size_t Serial::write(const std::string &data) {
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  return pimpl_->write(reinterpret_cast<const uint8_t *>(data.c_str()), data.length());
}

size_t Serial::write(const std::vector<uint8_t> &data) {
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  return pimpl_->write(&data[0], data.size());
}

size_t Serial::write(const uint8_t *data, size_t size) {
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  return pimpl_->write(data, size);
}

void Serial::setPort(const std::string &port_name) {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  bool was_open = pimpl_->isOpen();
  if (was_open) {
    close();
  }
  pimpl_->setPort(port_name);
  if (was_open) {
    open();
  }
}

std::string Serial::getPort() const {
  return pimpl_->getPort();
}

void Serial::setTimeout(const Timeout &timeout) {
  pimpl_->setTimeout(timeout);
}

void Serial::setTimeout(uint32_t inter_byte, uint32_t read_constant, uint32_t read_multiplier, uint32_t write_constant, uint32_t write_multiplier) {
  setTimeout(Timeout(inter_byte, read_constant, read_multiplier, write_constant, write_multiplier));
}

Serial::Timeout Serial::getTimeout() const {
  return pimpl_->getTimeout();
}

void Serial::setBaudrate(uint32_t baudrate) {
  pimpl_->setBaudrate(baudrate);
}

uint32_t Serial::getBaudrate() const {
  return uint32_t(pimpl_->getBaudrate());
}

void Serial::setBytesize(bytesize_t bytesize) {
  pimpl_->setBytesize(bytesize);
}

bytesize_t Serial::getBytesize() const {
  return pimpl_->getBytesize();
}

void Serial::setParity(parity_t parity) {
  pimpl_->setParity(parity);
}

parity_t Serial::getParity() const {
  return pimpl_->getParity();
}

void Serial::setStopbits(stopbits_t stopbits) {
  pimpl_->setStopbits(stopbits);
}

stopbits_t Serial::getStopbits() const {
  return pimpl_->getStopbits();
}

void Serial::setFlowcontrol(flowcontrol_t flowcontrol) {
  pimpl_->setFlowcontrol(flowcontrol);
}

flowcontrol_t Serial::getFlowcontrol() const {
  return pimpl_->getFlowcontrol();
}

void Serial::flush() {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  pimpl_->flush();
}

void Serial::flushInput() {
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  pimpl_->flushInput();
}

void Serial::flushOutput() {
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  pimpl_->flushOutput();
}

void Serial::sendBreak(int duration) {
  pimpl_->sendBreak(duration);
}

void Serial::setBreak(bool level) {
  pimpl_->setBreak(level);
}

void Serial::setRTS(bool level) {
  pimpl_->setRTS(level);
}

void Serial::setDTR(bool level) {
  pimpl_->setDTR(level);
}

void Serial::waitForModemChanges() {
  pimpl_->waitForModemChanges();
}

bool Serial::getCTS() {
  return pimpl_->getCTS();
}

bool Serial::getDSR() {
  return pimpl_->getDSR();
}

bool Serial::getRI() {
  return pimpl_->getRI();
}

bool Serial::getCD() {
  return pimpl_->getCD();
}
