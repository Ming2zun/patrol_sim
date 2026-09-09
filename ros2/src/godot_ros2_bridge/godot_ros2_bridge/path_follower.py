import signal
from rclpy.executors import ExternalShutdownException
"""Fixed-front geometric path tracking; FL/FR always define the front axle.
Uses simulated truth only when explicitly remapped to it for algorithm smoke tests.
"""
import json
import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PathFollower(Node):
    def __init__(self):
        super().__init__('alpine_path_follower')
        self.declare_parameter('lookahead',2.5);self.declare_parameter('speed',1.0);self.declare_parameter('frame_id','map')
        self.path=None;self.pose=None;self.index=0;self.updated=0.;self.frame=self.get_parameter('frame_id').value
        self.status = {}; self.status_at = 0.; self.started = False; self.accepted = 0.
        self.create_subscription(String, '/sim/status', self.on_status, 10)
        self.pub=self.create_publisher(Twist,'/cmd_vel',1)
        self.create_subscription(Path,'/sim/path',self.on_path,QoSProfile(depth=1))
        self.create_subscription(Odometry,'/sim/localization/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.05,self.update)
    def on_path(self,msg):
        if msg.header.frame_id!=self.frame or any(p.header.frame_id not in ('',self.frame) for p in msg.poses):self.get_logger().warning('path frame mismatch');return
        if not all(math.isfinite(v) for p in msg.poses for v in (p.pose.position.x,p.pose.position.y)):return
        if msg.poses and (self.status.get('estop') or self.status.get('paused')):
            self.get_logger().warning('Clear emergency stop / resume before sending a new path');return
        self.path=msg.poses;self.index=0;self.started=False;self.accepted=time.monotonic()
        if not self.path:self.pub.publish(Twist())
    def on_status(self, msg):
        status = json.loads(msg.data)
        if self.path and (status.get('estop') or status.get('paused') or
                not status.get('connected',True) or
                (self.started and status.get('mode') != 'ros') or
                status.get('epoch') != self.status.get('epoch')):
            self.path = None
            self.pub.publish(Twist())
        self.status = status
        self.status_at = time.monotonic()

    def on_odom(self,msg):
        if msg.header.frame_id!=self.frame:return
        self.pose=msg.pose.pose;self.updated=time.monotonic()
    def update(self):
        if not self.path:
            return
        if time.monotonic()-self.status_at > .5:
            self.path = None
            self.pub.publish(Twist())
            return
        if self.status.get('mode') != 'ros':
            if time.monotonic()-self.accepted > 2.:
                self.path = None
            return
        self.started = True
        if self.pose is None or time.monotonic()-self.updated >= .3:
            self.path = None
            self.pub.publish(Twist())
            return
        cmd=Twist()
        if self.path and self.pose and time.monotonic()-self.updated<.3:
            p=self.pose;goal=self.path[-1].pose.position;remaining=math.hypot(goal.x-p.position.x,goal.y-p.position.y)
            finished = remaining <= .2 and self.index == len(self.path)-1
            if finished:
                self.path = None
            if not finished:
                look=float(self.get_parameter('lookahead').value)
                while self.index<len(self.path)-1 and math.hypot(self.path[self.index].pose.position.x-p.position.x,self.path[self.index].pose.position.y-p.position.y)<look:self.index+=1
                target=self.path[self.index].pose.position;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                dx,dy=target.x-p.position.x,target.y-p.position.y
                angle=math.atan2(math.sin(math.atan2(dy,dx)-yaw),math.cos(math.atan2(dy,dx)-yaw))
                # The physical front axle remains FL/FR. Never reinterpret a
                # side or the rear as forward just because it is nearer a target.
                speed=max(0.,float(self.get_parameter('speed').value))
                if self.index==len(self.path)-1:speed=min(speed,remaining)
                # Ackermann chassis: advance slowly through turns; never request an in-place spin.
                curvature=2*math.sin(angle)/max(.3,math.hypot(dx,dy))
                curvature=max(-.34,min(.34,curvature))
                if abs(angle)>math.pi/3:speed=min(speed,.3)
                if abs(curvature)>1e-9:speed=min(speed,.4/abs(curvature))
                cmd.linear.x=speed
                cmd.angular.z=speed*curvature

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler);node=PathFollower()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node();rclpy.try_shutdown()
