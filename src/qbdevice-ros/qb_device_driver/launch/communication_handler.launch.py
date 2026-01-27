# Software License Agreement: BSD 3-Clause License

# Copyright (c) 2018-2024, qbrobotics®
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
#   products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
  
  serial_port_name_value = LaunchConfiguration('serial_port_name')
  use_specific_serial_port_value = LaunchConfiguration('use_specific_serial_port')

  serial_port_name_launch_arg = DeclareLaunchArgument(
      'serial_port_name',
      default_value=''
  )
  use_specific_serial_port_launch_arg = DeclareLaunchArgument(
      'use_specific_serial_port',
      default_value='false'
  )

  qb_node = Node(
          package='qb_device_driver',
          executable='qb_device_communication_handler',
          name='qb_device_driver_node',
          output="log",
          parameters=[{
            'serial_port_name': LaunchConfiguration('serial_port_name'),
            'use_specific_serial_port': LaunchConfiguration('use_specific_serial_port'),
         }]
  )

  return LaunchDescription([
        serial_port_name_launch_arg,
        use_specific_serial_port_launch_arg,
        qb_node
  ])