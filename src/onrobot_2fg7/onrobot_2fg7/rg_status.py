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
from onrobot_2fg7_interfaces.msg import RgStatus

class StatusPublisher(rclpy.node.Node):
	def __init__(self):
		super().__init__('onrobot_rg_status_publisher')

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
		self._publisher = self.create_publisher(RgStatus, 'onrobot_rg_status', queue_msgs)
		self.timer = self.create_timer(1/frequency, self.timer_callback)
		
	def timer_callback(self):
		msg = RgStatus()
		msg.status = 1
		try:
			result = self._xmlrpc_proxy.rg_get_all_variables(0)
			self.get_logger().debug('Result: "%s"' % result)
			msg.angle = result['angle']
			msg.angle_speed = result['angle_speed']
			msg.busy = result['busy']
			msg.depth = result['depth']
			msg.fingertip_offset = result['fingertip_offset']
			msg.relative_depth = result['relative_depth']
			msg.s1_pushed = result['s1_pushed']
			msg.s1_triggered = result['s1_triggered']
			msg.s2_pushed = result['s2_pushed']
			msg.s2_triggered = result['s2_triggered']
			msg.safety_failed = result['safety_failed']
			msg.speed = result['speed']
			msg.status = result['status']
			msg.width = result['width']
		except KeyError as k:
			self.get_logger().warn('Key error: "%s", probably received an empty response from server, check debug log for details.' % k)			
		except xmlrpc.client.Fault as f:
			self.get_logger().error('Server error: "%s"' % f)
		except Exception as e:
			self.get_logger().error('Error: "%s"' % e)
		try:
			result = self._xmlrpc_proxy.rg_get_grip_detected(0)
			self.get_logger().debug('Result: "%s"' % result)
			msg.grip_detected = result
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
    
