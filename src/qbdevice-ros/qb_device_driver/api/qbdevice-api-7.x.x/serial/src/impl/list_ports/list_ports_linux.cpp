/***
 *  MIT License
 *
 *  Copyright (c) 2023 qbrobotics®
 *  Copyright (c) 2020 Alessandro Tondo
 *  Copyright (c) 2014 Craig Lilley
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

#if defined(__linux__)

#include <climits>
#include <fstream>

#include <fcntl.h>
#include <glob.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>

#include <serial/serial.h>

using namespace serial;

int getLinkPath(std::string system_path, std::string &link_path) {
  struct stat serial_port_stat{0};
  if (::lstat(system_path.c_str(), &serial_port_stat) == -1) {
    return -1;
  }
  if (!S_ISLNK(serial_port_stat.st_mode)) {
    system_path += "/device";
  }
  char link_path_buf[PATH_MAX];
  ssize_t link_length = ::readlink(system_path.c_str(), link_path_buf, sizeof(link_path_buf) - 1);
  if (link_length == -1) {
    return -1;
  }
  link_path_buf[link_length] = '\0';
  link_path = std::string(link_path_buf);
  return 0;
}

int PortInfo::getPortInfo(const std::string &serial_port_name) {
  std::smatch serial_port_match;
  std::regex_match(serial_port_name, serial_port_match, std::regex("^/dev/([^/]+)/?$"));
  if (serial_port_match.size() != 2) {
    //TODO: wrong serial port name
    return -1;
  }
  std::string serial_port_substr(serial_port_match[1]);

  std::string link_path;
  std::string system_path("/sys/class/tty/" + serial_port_substr);
  if (getLinkPath(system_path, link_path)) {
    //TODO: device not found in sys or link read error
    return -1;
  }

  if (std::strstr(link_path.c_str(), "usb")) {
    system_path = "/sys/class/tty/" + serial_port_substr + "/device";
    for (int i=0; i<3; i++) {
      system_path += "/..";
      //FIXME: should check the existence at least for the first four files
      struct stat serial_port_stat{0};
      if (::stat((system_path + "/busnum").c_str(), &serial_port_stat) == -1) {
        continue;
      }
      std::ifstream(system_path + "/busnum") >> busnum;
      std::ifstream(system_path + "/devnum") >> devnum;
      std::ifstream(system_path + "/idProduct") >> std::hex >> id_product;
      std::ifstream(system_path + "/idVendor") >> std::hex >> id_vendor;
      std::getline(std::ifstream(system_path + "/manufacturer"), manufacturer);
      std::getline(std::ifstream(system_path + "/product"), product);
      std::getline(std::ifstream(system_path + "/serial"), serial_number);
      break;
    }
  }
  serial_port = serial_port_name;
  return 0;
}

int serial::getPortsInfo(std::vector<PortInfo> &serial_ports) {
  serial_ports.clear();
  std::vector<std::string> serial_port_names;
  if (getPortsList(serial_port_names) < 0) {
    return -1;
  }
  for (auto const &serial_port_name : serial_port_names) {
    PortInfo serial_port;
    if (!serial_port.getPortInfo(serial_port_name)) {
      serial_ports.push_back(serial_port);
    }
  }
  return serial_ports.size();
}

int serial::getPortsList(std::vector<std::string> &serial_port_names) {
  serial_port_names.clear();

  glob_t glob_results;
  std::string glob_pattern("/sys/class/tty/*");
  if (::glob(glob_pattern.c_str(), 0, nullptr, &glob_results)) {
    // 'serial_port_names' is cleared
    globfree(&glob_results);
    return -1;
  }

  for (int i=0; i<glob_results.gl_pathc; i++) {
    std::string link_path;
    std::string system_path(glob_results.gl_pathv[i]);
    if (getLinkPath(system_path, link_path)) {
      continue;
    }
    if (std::strstr(link_path.c_str(), "virtual")) {
      continue;
    }
    std::string serial_port_name("/dev/" + std::string(system_path.c_str() + std::strlen("/sys/class/tty/")));
    if (std::strstr(link_path.c_str(), "serial8250")) {
      int file_descriptor = ::open(serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (file_descriptor < 0) {
        continue;
      }
      struct serial_struct serial_info{0};
      if (::ioctl(file_descriptor, TIOCGSERIAL, &serial_info) || serial_info.type == PORT_UNKNOWN) {
        ::close(file_descriptor);
        continue;
      }
      ::close(file_descriptor);
    }
    serial_port_names.push_back(serial_port_name);
  }

  globfree(&glob_results);
  return serial_port_names.size();
}

#endif // defined(__linux__)
