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
import rclpy.node

import xmlrpc.client

from rcl_interfaces.msg import ParameterDescriptor
from onrobot_2fg7_interfaces.msg import Status

class StatusPublisher(rclpy.node.Node):
	def __init__(self):
		super().__init__('onrobot_2fg7_status_publisher')

		self.declare_parameters(
			namespace='',
			parameters=[
				('ip', '192.168.0.9', ParameterDescriptor(description='IP of the OnRobot host')),
				('port', 41414, ParameterDescriptor(description='Host port for OnRobot XML-RPC')),
				('frequency', 2.0, ParameterDescriptor(description='Number of messages to publish per second on average')),
				('queue_msgs', 10, ParameterDescriptor(description='Message queue depth'))
			]
		)
		ip = self.get_parameter('ip').get_parameter_value().string_value
		port = self.get_parameter('port').get_parameter_value().integer_value
		frequency = self.get_parameter('frequency').get_parameter_value().double_value
		queue_msgs = self.get_parameter('queue_msgs').get_parameter_value().integer_value
		
		self._xmlrpc_proxy = xmlrpc.client.ServerProxy("http://" + ip + ":" + str(port) + "/")
		self._publisher = self.create_publisher(Status, 'onrobot_2fg7_status', queue_msgs)
		self.timer = self.create_timer(1/frequency, self.timer_callback)
	
	def timer_callback(self):
		msg = Status()
		msg.status = 1
		try:
			result = self._xmlrpc_proxy.twofg_get_all_variables(0)
			self.get_logger().debug('Result: "%s"' % result)
			msg.status = result['status']
			msg.busy = result['busy']
			msg.error_linear_sensor = result['error_linear_sensor']
			msg.error_not_calibrated = result['error_not_calibrated']
			msg.grip_detected = result['grip_detected']
			msg.finger_height = result['finger_height']
			msg.finger_length = result['finger_length']
			msg.finger_orientation_outward = result['finger_orientation_outward']
			msg.fingertip_offset = result['fingertip_offset']
			msg.force = result['force']
			msg.max_force = result['max_force']
			msg.max_internal_width = result['max_internal_width']
			msg.internal_width = result['internal_width']
			msg.min_internal_width = result['min_internal_width']
			msg.min_external_width = result['min_external_width']
			msg.external_width = result['external_width']
			msg.max_external_width = result['max_external_width']
		except KeyError as k:
			self.get_logger().warn('Key error: "%s", probably received an empty response from server, check debug log for details.' % k)			
		except xmlrpc.client.Fault as f:
			self.get_logger().error('Server error: "%s"' % f)
		except Exception as e:
			self.get_logger().error('Error: "%s"' % e)
		self._publisher.publish(msg)
			
def main(args=None):
	rclpy.init(args=args)
	status = StatusPublisher()
	rclpy.spin(status)
	status.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
    
