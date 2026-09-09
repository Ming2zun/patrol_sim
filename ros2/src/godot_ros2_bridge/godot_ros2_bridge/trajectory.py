"""Bounded actual driving history in map, independent of the planned/ICP paths."""
import signal
import json, math
from copy import deepcopy
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

class Trajectory(Node):
    def __init__(self):
        super().__init__('alpine_trajectory')
        self.max_points=int(self.declare_parameter('max_points',10000).value)
        self.min_distance=float(self.declare_parameter('min_distance_m',.05).value)
        if self.max_points<2 or not math.isfinite(self.min_distance) or self.min_distance<=0:
            raise ValueError('max_points >= 2 and min_distance_m > 0 required')
        self.path=Path();self.path.header.frame_id='map'
        self.epoch=None;self.last_stamp=None;self.last_sim_time=None;self.dirty=False
        self.pub=self.create_publisher(Path,'/sim/trajectory',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'/sim/localization/odom',self.odom,20)
        self.create_subscription(String,'/sim/status',self.status,10)
        self.create_service(Trigger,'/sim/trajectory/clear',self.clear_service)
        self.create_timer(.2,self.publish)
    def clear(self):
        self.path.poses.clear();self.last_stamp=None
        self.path.header.stamp=self.get_clock().now().to_msg();self.dirty=True;self.publish()
    def clear_service(self,request,response):
        self.clear();response.success=True;response.message='Driving history cleared';return response
    def status(self,msg):
        try:
            s=json.loads(msg.data);epoch=s['epoch'];stamp=float(s['sim_time'])
        except (ValueError,TypeError,KeyError):return
        if not math.isfinite(stamp):return
        if (self.epoch is not None and epoch!=self.epoch) or (self.last_sim_time is not None and stamp<self.last_sim_time-.01):self.clear()
        self.epoch=epoch;self.last_sim_time=stamp
    def odom(self,msg):
        if msg.header.frame_id!='map':return
        p=msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):return
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.last_stamp is not None:
            if stamp<self.last_stamp:self.clear()
            elif stamp==self.last_stamp:return
        self.last_stamp=stamp
        if self.path.poses:
            previous=self.path.poses[-1].pose.position
            if math.dist((p.x,p.y,p.z),(previous.x,previous.y,previous.z))<self.min_distance:return
        pose=PoseStamped();pose.header=deepcopy(msg.header);pose.pose=deepcopy(msg.pose.pose)
        self.path.header=deepcopy(msg.header);self.path.poses.append(pose)
        if len(self.path.poses)>self.max_points:del self.path.poses[:-self.max_points]
        self.dirty=True
    def publish(self):
        if self.dirty:self.pub.publish(self.path);self.dirty=False

def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler);node=Trajectory()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
