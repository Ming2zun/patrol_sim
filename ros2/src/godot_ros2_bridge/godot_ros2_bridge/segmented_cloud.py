"""Accumulate SLAM registered scans, never the mapper's limited rolling local map."""
import json
import signal
import time
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .cloud_filter import RollingCloud


def xyz_from_cloud(msg):
    fields = {f.name:f for f in msg.fields}
    if any(k not in fields or fields[k].datatype != PointField.FLOAT32 or fields[k].count != 1 for k in ('x','y','z')):
        raise ValueError('Expected float32 x/y/z fields')
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.row_step*msg.height:
        raise ValueError('Invalid cloud layout')
    dtype = np.dtype({'names':['x','y','z'], 'formats':[('>' if msg.is_bigendian else '<')+'f4']*3,
                      'offsets':[fields[k].offset for k in ('x','y','z')], 'itemsize':msg.point_step})
    rows = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=bytes(msg.data), strides=(msg.row_step,msg.point_step))
    return np.column_stack([rows[k].ravel() for k in ('x','y','z')])


class SegmentedCloud(Node):
    def __init__(self):
        super().__init__('alpine_accumulated_cloud')
        defaults = {'min_z_m':-1000.,'radius_m':80.,'voxel_size_m':.2,'publish_hz':1.,'max_voxels':500000}
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        p = {key:self.get_parameter(key).value for key in defaults}
        self.grid = RollingCloud(p['min_z_m'],p['radius_m'],p['voxel_size_m'],p['max_voxels'])
        if not 0 < p['publish_hz'] <= 10:
            raise ValueError('publish_hz must be in (0,10]')
        self.pub = self.create_publisher(PointCloud2, '/sim/segmentation/accumulated_points', QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.center = None; self.pose_at = 0.; self.epoch = None; self.cutoff = 0.
        self.last_stamp = self.get_clock().now().to_msg(); self.dirty = False
        self.create_subscription(Odometry, '/sim/localization/odom', self.odom, 10)
        self.create_subscription(String, '/sim/status', self.status, 10)
        self.create_subscription(PointCloud2, '/sim/slam/registered_points', self.scan, 1)
        self.create_timer(1./p['publish_hz'], self.publish)
        self.get_logger().info(f'Keep map Z >= {p["min_z_m"]} m; rolling XY radius {p["radius_m"]} m; voxel {p["voxel_size_m"]} m; {p["publish_hz"]} Hz')

    def odom(self, msg):
        if msg.header.frame_id != 'map':return
        p = msg.pose.pose.position
        if not np.isfinite([p.x,p.y,p.z]).all():return
        self.center = (p.x,p.y,p.z); self.pose_at = time.monotonic()

    def status(self, msg):
        s = json.loads(msg.data)
        epoch = s.get('epoch')
        sim_time = float(s.get('sim_time',0.))
        rewound = sim_time < getattr(self,'previous_sim_time',-1.)-.01
        self.previous_sim_time = sim_time
        if epoch != self.epoch or rewound:
            self.epoch = epoch; self.cutoff = sim_time
            self.clear()

    def clear(self):
        self.grid.clear(); self.last_stamp = self.get_clock().now().to_msg(); self.dirty = True
        self.publish()

    def scan(self, msg):
        if msg.header.frame_id != 'map':return
        if not msg.width or not msg.height:
            self.clear();return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1.e-9
        if self.epoch is None or stamp < self.cutoff or self.center is None or time.monotonic()-self.pose_at > 1.0:
            return
        if abs(self.get_clock().now().nanoseconds*1.e-9-stamp) > 1.:
            return
        try:
            self.grid.add(xyz_from_cloud(msg),self.center)
        except (ValueError,TypeError) as exc:
            self.get_logger().warning(str(exc));return
        self.last_stamp = msg.header.stamp; self.dirty = True

    def publish(self):
        if self.center is not None and self.grid.prune(self.center):self.dirty = True
        if not self.dirty:return
        points = self.grid.points()
        msg = PointCloud2(); msg.header.frame_id = 'map'; msg.header.stamp = self.last_stamp
        msg.height = 1; msg.width = len(points); msg.is_bigendian = False; msg.is_dense = True
        msg.point_step = 12; msg.row_step = msg.width*12
        msg.fields = [PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
        msg.data = points.astype('<f4',copy=False).tobytes()
        self.pub.publish(msg); self.dirty = False


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler);node=SegmentedCloud()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node();rclpy.try_shutdown()
