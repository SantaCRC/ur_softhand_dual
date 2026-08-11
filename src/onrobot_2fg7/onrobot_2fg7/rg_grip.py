# Copyright 2026 David Dovrat
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

import xmlrpc.client

from rcl_interfaces.msg import ParameterDescriptor
from onrobot_2fg7_interfaces.srv import Grip

class GripService(Node):

	def __init__(self):
		super().__init__('onrobot_rg_grip_service')
		
		self.declare_parameters(
			namespace='',
			parameters=[
				('ip', '192.168.0.9', ParameterDescriptor(description='IP of the OnRobot host')),
				('port', 41414, ParameterDescriptor(description='Host port for OnRobot XML-RPC'))
			]
		)
		ip = self.get_parameter('ip').get_parameter_value().string_value
		port = self.get_parameter('port').get_parameter_value().integer_value
		
		self._xmlrpc_proxy = xmlrpc.client.ServerProxy("http://" + ip + ":" + str(port) + "/")
		self._srv = self.create_service(Grip, 'grip', self.grip_callback)
		
	def grip_callback(self, request, response):
		try:
			result = self._xmlrpc_proxy.rg_grip(request.id, request.gap, request.force*1.0)
		except xmlrpc.client.Fault as f:
			self.get_logger().error('Server error: "%s"' % f)
		except Exception as e:
			self.get_logger().error('Error: "%s"' % e)
		self.get_logger().debug('onrobot_rg_grip_service request: %s' % request)
		self.get_logger().debug('onrobot_rg_grip_service response: %s' % response)
		return response

def main():
	rclpy.init()
	gripService = GripService()
	rclpy.spin(gripService)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

