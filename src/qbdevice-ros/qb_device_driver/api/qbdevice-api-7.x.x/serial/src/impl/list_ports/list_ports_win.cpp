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

#if defined(_WIN32)

#include <tchar.h>
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <cfgmgr32.h>
#include <usbioctl.h>
#include <usbiodef.h>
#include <devpkey.h>

#include <codecvt>

#include <serial/impl/impl.h>

using namespace serial;

template <typename T>
std::shared_ptr<T> sharedPtr(size_t size = 0) {
  std::shared_ptr<T> ptr(static_cast<T*>(::malloc(sizeof(T) + size)), ::free);
  ::memset(ptr.get(), 0, sizeof(T)+size);
  return ptr;
}

std::wstring toWString(const std::string& str) {
  return std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>>().from_bytes(str);
}

std::string toString(const std::wstring& wstr) {
  return std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>>().to_bytes(wstr);
}

std::string getDeviceDescriptor(HANDLE handle, DWORD connection_index, UCHAR descriptor_index) {
  if (!descriptor_index) {
    return "";
  }

  char buffer[sizeof(USB_DESCRIPTOR_REQUEST) + MAXIMUM_USB_STRING_LENGTH] = {0};
  auto request = reinterpret_cast<PUSB_DESCRIPTOR_REQUEST>(buffer);
  auto descriptor = reinterpret_cast<PUSB_STRING_DESCRIPTOR>(request+1);
  DWORD size = sizeof(buffer);

  request->ConnectionIndex = connection_index;
  request->SetupPacket.bmRequest = 0x80;
  request->SetupPacket.bRequest = USB_REQUEST_GET_DESCRIPTOR;
  request->SetupPacket.wIndex = 0;
  request->SetupPacket.wLength = MAXIMUM_USB_STRING_LENGTH;
  request->SetupPacket.wValue = (USB_STRING_DESCRIPTOR_TYPE << 8) | descriptor_index;
  if (!::DeviceIoControl(handle, IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION, request, size, request, size, &size, nullptr)) {
    return "";
  }

  return toString(descriptor->bString);
}

std::string getDevicePath(const GUID &guid, DEVINST instance, DWORD &busnum) {
  std::string device_path;
  HDEVINFO device_info = ::SetupDiGetClassDevsA(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (device_info != INVALID_HANDLE_VALUE) {
    auto device_info_data = sharedPtr<SP_DEVINFO_DATA>();
    device_info_data->cbSize = sizeof(SP_DEVINFO_DATA);
    for (int i=0; ::SetupDiEnumDeviceInfo(device_info, i, device_info_data.get()); i++) {
      DEVINST instance_actual = instance;
      auto device_interface_data = sharedPtr<SP_DEVICE_INTERFACE_DATA>();
      device_interface_data->cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
      for (int j=0; ::SetupDiEnumDeviceInterfaces(device_info, device_info_data.get(), &guid, j, device_interface_data.get()); j++) {
        DWORD required_size = 0;
        if (!::SetupDiGetDeviceInterfaceDetailA(device_info, device_interface_data.get(), nullptr, 0, &required_size, nullptr) && GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
          continue;
        }

        auto device_detail_data = sharedPtr<SP_DEVICE_INTERFACE_DETAIL_DATA_A>(required_size);
        device_detail_data->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
        if (!::SetupDiGetDeviceInterfaceDetailA(device_info, device_interface_data.get(), device_detail_data.get(), required_size, nullptr, device_info_data.get())) {
          continue;
        }

        while (::CM_Get_Parent(&instance_actual, instance_actual, 0) == CR_SUCCESS && instance_actual != device_info_data->DevInst) {
          ;
        }
        if (instance_actual == device_info_data->DevInst) {
          busnum = i + 1;
          device_path = std::string(device_detail_data->DevicePath);
          break;
        }
      }
      if (!device_path.empty()) {
        break;
      }
    }
  }
  ::SetupDiDestroyDeviceInfoList(device_info);
  return device_path;
}

std::string getHubName(HANDLE handle, DWORD connection_index) {
  DWORD size = sizeof(USB_NODE_CONNECTION_NAME);
  auto hub_name_tmp = sharedPtr<USB_NODE_CONNECTION_NAME>();
  hub_name_tmp->ConnectionIndex = connection_index;
  if (!::DeviceIoControl(handle, IOCTL_USB_GET_NODE_CONNECTION_NAME, hub_name_tmp.get(), size, hub_name_tmp.get(), size, &size, nullptr)) {
    return "";
  }

  size = hub_name_tmp->ActualLength;
  auto hub_name = sharedPtr<USB_NODE_CONNECTION_NAME>(size);
  hub_name->ConnectionIndex = connection_index;
  if (!::DeviceIoControl(handle, IOCTL_USB_GET_NODE_CONNECTION_NAME, hub_name.get(), size, hub_name.get(), size, &size, nullptr)) {
    return "";
  }

  return toString(hub_name->NodeName);
}

std::string getRootName(HANDLE handle) {
  DWORD size = sizeof(USB_ROOT_HUB_NAME);
  auto root_name_tmp = sharedPtr<USB_ROOT_HUB_NAME>();
  if (!::DeviceIoControl(handle, IOCTL_USB_GET_ROOT_HUB_NAME, nullptr, 0, root_name_tmp.get(), size, &size, nullptr)) {
    return "";
  }

  size = root_name_tmp->ActualLength;
  auto root_name = sharedPtr<USB_ROOT_HUB_NAME>(size);
  if (!::DeviceIoControl(handle, IOCTL_USB_GET_ROOT_HUB_NAME, nullptr, 0, root_name.get(), size, &size, nullptr)) {
    return "";
  }

  return toString(root_name->RootHubName);
}

std::string getPortNameFromFriendlyName(const std::string& friendly_name) {
  std::smatch serial_port_match;
  std::regex_match(friendly_name, serial_port_match, std::regex("^.*(COM\\d+).*$"));
  if (serial_port_match.size() != 2) {
    return "";
  }
  return std::string(serial_port_match[1]);
}

DWORD getPortPropertyValue(HDEVINFO &device_info_set, SP_DEVINFO_DATA &device_info_data, DEVPROPKEY property) {
  DEVPROPTYPE devPropType;
  DWORD length = 0;
  DWORD buffer = 0;
  if (!::SetupDiGetDeviceProperty(device_info_set, &device_info_data, &property, &devPropType, reinterpret_cast<PBYTE>(&buffer), sizeof(buffer), &length, 0)) {
    return 0;
  }
  return buffer;
}

std::string getPortPropertyString(HDEVINFO &device_info_set, SP_DEVINFO_DATA &device_info_data, DEVPROPKEY property) {
  DEVPROPTYPE devPropType;
  DWORD length = 0;
  WCHAR buffer[256] = {0};
  if (!::SetupDiGetDeviceProperty(device_info_set, &device_info_data, &property, &devPropType, reinterpret_cast<PBYTE>(buffer), sizeof(buffer), &length, 0)) {
    return "";
  }
  return toString(buffer);
}

int getCOMPorts(std::map<std::string, DWORD> &serial_port_numbers, std::map<std::string, DWORD> &serial_port_instances) {
  serial_port_numbers.clear();
  serial_port_instances.clear();
  HDEVINFO device_info = ::SetupDiGetClassDevsA(&GUID_DEVINTERFACE_COMPORT, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (device_info == INVALID_HANDLE_VALUE) {
    return -1;
  }

  SP_DEVINFO_DATA device_info_data {.cbSize = sizeof(SP_DEVINFO_DATA)};
  for (int i=0; ::SetupDiEnumDeviceInfo(device_info, i, &device_info_data); i++) {
    DWORD port_number = getPortPropertyValue(device_info, device_info_data, DEVPKEY_Device_Address);
    std::string hardware_id = getPortPropertyString(device_info, device_info_data, DEVPKEY_Device_HardwareIds);  // only for debug
    std::string friendly_name = getPortPropertyString(device_info, device_info_data, DEVPKEY_Device_FriendlyName);
    std::string serial_port_name = getPortNameFromFriendlyName(friendly_name);

    serial_port_numbers.insert(std::make_pair(serial_port_name, port_number));
    serial_port_instances.insert(std::make_pair(serial_port_name, device_info_data.DevInst));
  }

  ::SetupDiDestroyDeviceInfoList(device_info);
  return serial_port_numbers.size();
}

int getInfo(HANDLE handle, DWORD serial_port_number, HANDLE &handle_found, std::shared_ptr<USB_NODE_CONNECTION_INFORMATION_EX> &connection_info_found) {
  USB_NODE_INFORMATION node_info{};
  DWORD size = sizeof(node_info);
  if (::DeviceIoControl(handle, IOCTL_USB_GET_NODE_INFORMATION, &node_info, size, &node_info, size, &size, nullptr)) {
    for (int port_number=1; port_number <= node_info.u.HubInformation.HubDescriptor.bNumberOfPorts; port_number++) {
      auto connection_info = sharedPtr<USB_NODE_CONNECTION_INFORMATION_EX>();
      connection_info->ConnectionIndex = port_number;
      size = sizeof(*connection_info);
      if (!::DeviceIoControl(handle, IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX, connection_info.get(), size, connection_info.get(), size, &size, nullptr)) {
        continue;
      }
      if (connection_info->DeviceIsHub) {
        HANDLE hub_handle = ::CreateFileA(escape(getHubName(handle, connection_info->ConnectionIndex)).c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        if (hub_handle == INVALID_HANDLE_VALUE) {
          continue;
        }
        if (getInfo(hub_handle, serial_port_number, handle_found, connection_info_found)) {
          ::CloseHandle(hub_handle);
          continue;
        }
        return 0;
      }
      if (connection_info->ConnectionIndex != serial_port_number) {
        continue;
      }

      handle_found = handle;
      connection_info_found = connection_info;
      return 0;
    }
  }
  handle_found = INVALID_HANDLE_VALUE;
  connection_info_found = nullptr;
  return -1;
}

int PortInfo::getPortInfo(const std::string &serial_port_name) {
  std::map<std::string, DWORD> serial_port_numbers;
  std::map<std::string, DWORD> serial_port_instances;
  getCOMPorts(serial_port_numbers, serial_port_instances);
  if (!serial_port_numbers.count(serial_port_name)) {
    return -1;
  }

  DWORD bus_number = 0;
  std::string device_path = getDevicePath(GUID_DEVINTERFACE_USB_HOST_CONTROLLER, serial_port_instances.at(serial_port_name), bus_number);
  HANDLE device_handle = ::CreateFileA(device_path.c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
  if (device_handle == INVALID_HANDLE_VALUE) {
    return -1;
  }
  HANDLE root_handle = ::CreateFileA(escape(getRootName(device_handle)).c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
  if (root_handle == INVALID_HANDLE_VALUE) {
    ::CloseHandle(device_handle);
    return -1;
  }

  HANDLE handle_found = INVALID_HANDLE_VALUE;
  auto connection_info_found = sharedPtr<USB_NODE_CONNECTION_INFORMATION_EX>();
  if (!getInfo(root_handle, serial_port_numbers.at(serial_port_name), handle_found, connection_info_found)) {
    busnum = bus_number;
    devnum = connection_info_found->DeviceAddress + 1;
    id_vendor = connection_info_found->DeviceDescriptor.idVendor;
    id_product = connection_info_found->DeviceDescriptor.idProduct;
    manufacturer = getDeviceDescriptor(handle_found, connection_info_found->ConnectionIndex, connection_info_found->DeviceDescriptor.iManufacturer);
    product = getDeviceDescriptor(handle_found, connection_info_found->ConnectionIndex, connection_info_found->DeviceDescriptor.iProduct);
    serial_number = getDeviceDescriptor(handle_found, connection_info_found->ConnectionIndex, connection_info_found->DeviceDescriptor.iSerialNumber);
    ::CloseHandle(handle_found);
  }

  if (handle_found != root_handle) {
    ::CloseHandle(root_handle);
  }
  ::CloseHandle(device_handle);
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
  std::map<std::string, DWORD> serial_port_numbers;
  std::map<std::string, DWORD> serial_port_instances;
  getCOMPorts(serial_port_numbers, serial_port_instances);
  for (auto const& serial_port : serial_port_numbers) {
    serial_port_names.push_back(serial_port.first);
  }
  return serial_port_names.size();
}

#endif // #if defined(_WIN32)
