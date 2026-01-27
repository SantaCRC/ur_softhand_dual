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

#if defined(__APPLE__)

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>

#include <serial/serial.h>

using namespace serial;

int toInt(CFNumberRef cf_num) {
  int num;
  if (cf_num && ::CFNumberGetValue(cf_num, kCFNumberIntType, &num)) {
    return num;
  }
  return 0;
}

std::string toString(CFStringRef cf_str) {
  char str[PATH_MAX] = {};
  if (cf_str && ::CFStringGetCString(cf_str, str, sizeof(str), kCFStringEncodingASCII)) {
    return std::string(str);
  }
  return "";
}

std::string getDevicePath(const io_object_t &serial_port_obj) {
  CFTypeRef path = ::IORegistryEntryCreateCFProperty(serial_port_obj, CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);
  std::string device_path = toString(reinterpret_cast<CFStringRef>(path));
  if (path) {
    ::CFRelease(path);
  }
  return device_path;
}

std::string getPortPropertyString(const io_object_t &serial_port_obj, const std::string &property_name) {
  CFStringRef cf_property_name = ::CFStringCreateWithCString(kCFAllocatorDefault, property_name.c_str(), kCFStringEncodingASCII);
  CFTypeRef cf_property = ::IORegistryEntrySearchCFProperty(serial_port_obj, kIOServicePlane, cf_property_name, kCFAllocatorDefault, kIORegistryIterateRecursively | kIORegistryIterateParents);
  std::string property_value = toString(reinterpret_cast<CFStringRef>(cf_property));
  if (cf_property) {
    ::CFRelease(cf_property);
  }
  return property_value;
}

uint16_t getPortPropertyValue(const io_object_t &serial_port_obj, const std::string &property_name) {
  CFStringRef cf_property_name = ::CFStringCreateWithCString(kCFAllocatorDefault, property_name.c_str(), kCFStringEncodingASCII);
  CFTypeRef cf_property = ::IORegistryEntrySearchCFProperty(serial_port_obj, kIOServicePlane, cf_property_name, kCFAllocatorDefault, kIORegistryIterateRecursively | kIORegistryIterateParents);
  int property_value = toInt(reinterpret_cast<CFNumberRef>(cf_property));
  if (cf_property) {
    ::CFRelease(cf_property);
  }
  return property_value;
}

std::string getClassName(const io_object_t &device_obj) {
  io_name_t class_name;
  return ::IOObjectGetClass(device_obj, class_name) == KERN_SUCCESS ? class_name : "";
}

io_registry_entry_t getParent(io_object_t &serial_port_obj) {
  io_object_t parent = serial_port_obj;
  while (getClassName(parent) != "IOUSBDevice") {
    if (::IORegistryEntryGetParentEntry(parent, kIOServicePlane, &parent) != KERN_SUCCESS) {
      return 0;
    }
  }
  return parent;
}

int PortInfo::getPortInfo(const std::string &serial_port_name) {
  CFMutableDictionaryRef classes;
  if (!(classes = ::IOServiceMatching(kIOSerialBSDServiceValue))) {
    return -1;
  }

  io_iterator_t serial_port_iterator;
  if (::IOServiceGetMatchingServices(kIOMasterPortDefault, classes, &serial_port_iterator) != KERN_SUCCESS) {
    return -1;
  }

  io_object_t serial_port_obj;
  while ((serial_port_obj = ::IOIteratorNext(serial_port_iterator))) {
    std::string serial_port_name_found = getDevicePath(serial_port_obj);
    if (serial_port_name_found != serial_port_name) {
      continue;
    }
    serial_port = serial_port_name_found;

    io_registry_entry_t parent = getParent(serial_port_obj);
    if (parent == 0) {
      continue;
    }
    ::IOObjectRelease(parent);

    devnum = getPortPropertyValue(serial_port_obj, "USB Address");
    id_product = getPortPropertyValue(serial_port_obj, "idProduct");
    id_vendor = getPortPropertyValue(serial_port_obj, "idVendor");
    manufacturer = getPortPropertyString(serial_port_obj, "USB Vendor Name");
    product = getPortPropertyString(serial_port_obj, "USB Product Name");
    serial_number = getPortPropertyString(serial_port_obj, "USB Serial Number");
    ::IOObjectRelease(serial_port_obj);
  }

  ::IOObjectRelease(serial_port_iterator);
  return serial_port != serial_port_name ? -1 : 0;
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

  CFMutableDictionaryRef classes;
  if (!(classes = ::IOServiceMatching(kIOSerialBSDServiceValue))) {
    return -1;
  }

  io_iterator_t serial_port_iterator;
  if (::IOServiceGetMatchingServices(kIOMasterPortDefault, classes, &serial_port_iterator) != KERN_SUCCESS) {
    return -1;
  }

  io_object_t serial_port_obj;
  while ((serial_port_obj = ::IOIteratorNext(serial_port_iterator))) {
    serial_port_names.push_back(getDevicePath(serial_port_obj));
    ::IOObjectRelease(serial_port_obj);
  }

  ::IOObjectRelease(serial_port_iterator);
  return serial_port_names.size();
}

#endif // defined(__APPLE__)
