# Onrobot 2FG7

A [ROS2](https://www.ros.org/) package for [Onrobot 2FG7](https://onrobot.com/en/products/2fg7-finger-gripper).

This package includes the ROS2 nodes described below. To launch all nodes, create appropriate parameter files in a directory called *config*, build & source, then run:

```console
ros2 launch onrobot_2fg7 onrobot_2fg7_launch.xml
```

## Dependencies

* [onrobot_2fg7_interfaces](https://github.com/davedovrat/onrobot_2fg7_interfaces)

## Status Publisher

The Status Publisher periodically sends a _twofg_get_all_variables_ request to the 2FG7 XML-RPC server.

To run:

 ```console
ros2 run onrobot_2fg7 status
 ```

to echo the published topic:

```console
ros2 topic echo /onrobot_2fg7_status
```

### Parameters

The parameter file should look like this:

```yaml
/onrobot_2fg7_status_publisher:
  ros__parameters:
    frequency: 2.0
    ip: 192.168.0.9
    port: 41414
    queue_msgs: 10
    use_sim_time: false

```

| Parameter 	| Meaning		|Default Value	|	Remark	|
| ---------		| ------------	|-------		|-------	|
| frequency	| The frequency of requests to the XML-RPC server, each server response gets published to the topic _onrobot_2fg7_status_ when received | 2.0 | in Hertz (Hz)|
| ip	| The IP of the OnRobot host | 192.168.0.9 | The host is usually a robotic arm. |
| port	| Host port for OnRobot XML-RPC | 41414 | [Universal Robots](https://www.universal-robots.com/) [default port](https://forum.universal-robots.com/t/overview-of-used-ports-on-local-host/8889). |
| queue_msgs	| ROS node message queue depth | 10 | Integer value. |
| use_sim_time	| Should the time also be simulated	| false	|	See [ROS Clock](http://wiki.ros.org/Clock) |

## Grip Service

The Grip Service forwards _twofg_grip_external_ requests to the XML-RPC server.

To run:

 ```console
ros2 run onrobot_2fg7 grip
 ```

To send a request (using default variable values):

```console
ros2 service call /grip  onrobot_2fg7_interfaces/srv/Grip "{'gap': 60.0}"
```

### Request

| Variable 	| Meaning		|Default Value	|	Type	|
| ---------		| ------------	|-------		|-------	|
| gap	| The value in mm of the desired gap between the gripper's fingers. | - | float64 |
| id	| The 2FG7 ID (on the URCap table, 0 is default and will work if the 2FG7 is the only OnRobot attached to the UR). | 0 | int64. |
| force	| The force in N applied to get to the desired gap. | 50 | int64. |
| speed	| The speed (in % of maximal) in which to reach the gap.  | 50 | int64. |

### Reply

| Variable 	| Meaning		|Default Value	|	Type	|
| ---------		| ------------	|-------		|-------	|
| status	| - | 0 | int64 |

### Parameters

The parameter file should look like this:

```yaml
/onrobot_2fg7_status_publisher:
  ros__parameters:
    ip: 192.168.0.9
    port: 41414
    use_sim_time: false

```

| Parameter 	| Meaning		|Default Value	|	Remark	|
| ---------		| ------------	|-------		|-------	|
| ip	| The IP of the OnRobot host | 192.168.0.9 | The host is usually a robotic arm. |
| port	| Host port for OnRobot XML-RPC | 41414 | [Universal Robots](https://www.universal-robots.com/) [default port](https://forum.universal-robots.com/t/overview-of-used-ports-on-local-host/8889). |
| use_sim_time	| Should the time also be simulated	| false	|	See [ROS Clock](http://wiki.ros.org/Clock) |

