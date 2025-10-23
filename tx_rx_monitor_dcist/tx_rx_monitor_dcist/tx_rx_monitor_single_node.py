#!/usr/bin/env python

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from pathlib import Path
import time

#from read_tx_rx import TxRxMonitor

from std_msgs.msg import String

from ros_system_monitor_msgs.msg import NodeInfoMsg

# ============================================================
# Non ROS2-specific code
# TODO include from read_tx_rx.py, rather than duplicating here
# ------------------------------------------------------------
def timestamp() -> float:
    return time.time_ns() / 1e9

class TxRxMonitor:

    def __init__(self, iface: str):
        self.iface = iface
        base = Path(f'/sys/class/net/{self.iface}/statistics')
        self.tx_bytes_path = base / 'tx_bytes'
        self.rx_bytes_path = base / 'rx_bytes'

        assert self.tx_bytes_path.exists(), f"Path {self.tx_bytes_path} does not exist"
        assert self.rx_bytes_path.exists(), f"Path {self.rx_bytes_path} does not exist"

        self.tx_prev = self.read_tx_bytes()
        self.rx_prev = self.read_rx_bytes()
        self.timestamp_prev = timestamp()

    def read_counter(self, path: Path) -> int:
        return int(path.read_text().strip())

    def read_tx_bytes(self) -> int:
        return self.read_counter(self.tx_bytes_path)

    def read_rx_bytes(self) -> int:
        return self.read_counter(self.rx_bytes_path)

    def mk_dict(self) -> dict:
        tx_bytes = self.read_tx_bytes()
        rx_bytes = self.read_rx_bytes()
        tx_curr = tx_bytes - self.tx_prev
        rx_curr = rx_bytes - self.rx_prev
        self.tx_prev = tx_bytes
        self.rx_prev = rx_bytes
        timestamp_curr = timestamp()
        delta_t = timestamp_curr - self.timestamp_prev
        self.timestamp_prev = timestamp_curr

        return {
            'iface': self.iface,
            'timestamp': timestamp_curr,
            'delta_t': delta_t,
            'tx_bytes': tx_curr,
            'rx_bytes': rx_curr,
        }
# ============================================================



# ============================================================
# https://github.com/MIT-SPARK/ROS-System-Monitor/blob/main/ros_system_monitor_msgs/msg/NodeInfoMsg.msg
# ------------------------------------------------------------
# NodeInfoMsg.NOMINAL = 1   # The node is functioning normally
# NodeInfoMsg.WARNING = 2   # The node is partially failing but still functioning
# NodeInfoMsg.ERROR   = 3   # The node is no longer functioning
# NodeInfoMsg.NO_HB   = 4   # The node has not been seen for long enough that it is considered missing
# NodeInfoMsg.STARTUP = 5   # The node is still initializing or has not been seen yet
# ============================================================

class TxRxMonitorNode(Node):

    def __init__(self):
        super().__init__('tx_rx_monitor_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ("robot_id", 0),                # robot id for this node
                ("nickname", "tx_rx_monitor"),  # default nickname
                ("iface", "wlan0"),             # network interface to monitor
                ("poll_rate_hz", 1.0),          # polling rate in Hz
            ]
        )

        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.nickname = self.get_parameter("nickname").get_parameter_value().string_value
        self.iface = self.get_parameter("iface").get_parameter_value().string_value
        self.poll_rate_hz = self.get_parameter("poll_rate_hz").get_parameter_value().double_value

        self.monitor = TxRxMonitor(self.iface)

        self.status_pub = self.create_publisher(NodeInfoMsg, "tx_rx/status", qos_profile=QoSProfile(depth=10))
        self.usage_pub = self.create_publisher(String, "tx_rx_usage", qos_profile=QoSProfile(depth=10))
        self.timer = self.create_timer(1.0 / self.poll_rate_hz, self.timer_cb)
        
        self.status = NodeInfoMsg.STARTUP

    def log_and_send_status(self, note, status=NodeInfoMsg.NOMINAL):
        """
        Log a message and send it to the status topic.
        """
        self.get_logger().info(note)
        status_msg = NodeInfoMsg()
        status_msg.nickname = self.nickname
        status_msg.node_name = self.get_fully_qualified_name()
        status_msg.status = status
        status_msg.notes = note
        self.status_pub.publish(status_msg)

    def timer_cb(self):
        try:
            data = self.monitor.mk_dict()
            data['timestamp'] = str(self.get_clock().now().to_msg())
            self.usage_pub.publish(String(data=json.dumps(data)))

            tx_kb = data['tx_bytes'] / 1024.0
            rx_kb = data['rx_bytes'] / 1024.0
            secs = data['delta_t']
            status = NodeInfoMsg.NOMINAL
            self.log_and_send_status(f'TX (KB) = {tx_kb:.2f}, RX (KB) = {rx_kb:.2f}, Δt = {secs:.2f}s', status)
        except:
            status = NodeInfoMsg.ERROR
            self.log_and_send_status(f'Error occurred while reading network statistics', status)

def main(args=None):
    rclpy.init(args=args)
    node = TxRxMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()