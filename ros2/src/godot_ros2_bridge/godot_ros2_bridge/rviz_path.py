"""Adapt RViz LineDraw Path requests to the map navigation controller."""
import math
import time
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool, Trigger


class RvizPath(Node):
    def __init__(self):
        super().__init__('alpine_rviz_path')
        self.points = []
        self.pose = None
        self.updated = 0.
        self.pending = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/sim/path', qos)
        self.preview = self.create_publisher(Path, '/sim/path_preview', qos)
        self.plugin_command = self.create_publisher(UInt8, '/test_c/line_draw/command', 10)
        self.create_subscription(Path, '/test_c/click_path_request', self.plugin_path, 10)
        self.owner = self.create_client(SetBool, '/sim/set_remote')
        self.create_subscription(PointStamped, '/clicked_point', self.point, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal, 10)
        self.create_subscription(Odometry, '/sim/localization/odom', self.odom, qos_profile_sensor_data)
        self.create_service(Trigger, '/sim/path/clear', self.clear)
        self.get_logger().info('LineDraw Path -> /sim/path in map; /sim/path/clear cancels tracking')

    def plugin_path(self, msg):
        if msg.header.frame_id != 'map':
            return
        if not msg.poses:
            self.clear_route()
            return
        age = (self.get_clock().now().nanoseconds - msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)*1.e-9
        if age < -.5 or age > 5. or len(msg.poses) < 2:
            return
        if any(p.header.frame_id not in ('', 'map') or not all(map(math.isfinite, (p.pose.position.x, p.pose.position.y))) for p in msg.poses):
            return
        self.submit(msg)

    def odom(self, msg):
        if msg.header.frame_id == 'map':
            self.pose = msg.pose.pose
            self.updated = time.monotonic()

    def path(self, points):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.position.z = self.pose.position.z if self.pose else 0.
            p.pose.orientation.w = 1.
            msg.poses.append(p)
        return msg

    def point(self, msg):
        if msg.header.frame_id != 'map' or not all(map(math.isfinite, (msg.point.x, msg.point.y))):
            return
        self.points.append((msg.point.x, msg.point.y))
        self.preview.publish(self.path(self.points))

    def goal(self, msg):
        if (msg.header.frame_id != 'map' or self.pending is not None or
                self.pose is None or time.monotonic()-self.updated > .5):
            self.get_logger().warning('Cannot start path: frame, localization or pending request')
            return
        end = (msg.pose.position.x, msg.pose.position.y)
        if not all(map(math.isfinite, end)) or not self.owner.service_is_ready():
            return
        points = [(self.pose.position.x, self.pose.position.y)] + self.points + [end]
        samples = []
        for a, b in zip(points, points[1:]):
            count = max(1, math.ceil(math.dist(a, b)/.2))
            samples.extend((a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count) for i in range(count))
        samples.append(end)
        path = self.path(samples)
        self.submit(path)

    def submit(self, path):
        if self.pending is not None or self.pose is None or time.monotonic()-self.updated > .5 or not self.owner.service_is_ready():
            self.get_logger().warning('Path not started: localization/control service unavailable')
            return
        self.pending = self.owner.call_async(SetBool.Request(data=True))
        def ready(future):
            if self.pending is not future:
                return
            self.pending = None
            if future.result() and future.result().success:
                self.path_pub.publish(path)
                self.points.clear()
                self.preview.publish(self.path([]))
                self.get_logger().info(f'Published {len(path.poses)} path samples; waiting for ROS ownership')
            else:
                self.get_logger().warning('Simulation rejected ROS ownership request')
        self.pending.add_done_callback(ready)

    def clear_route(self):
        self.pending = None
        self.points.clear()
        self.preview.publish(self.path([]))
        self.path_pub.publish(self.path([]))
    def clear(self, request, response):
        self.clear_route()
        self.plugin_command.publish(UInt8(data=3))
        response.success = True
        response.message = 'Path cleared and tracking stopped'
        return response


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler)
    node = RvizPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
