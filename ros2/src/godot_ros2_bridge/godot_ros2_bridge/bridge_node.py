import signal
"""ROS 2 adapter. No Godot, Blender, rosbridge or third-party Python dependency."""
import json
import base64
import binascii
import math
import time
from functools import partial

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock as RclpyClock
from rclpy.clock import ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, CompressedImage, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .transport import SimTransport
from .pointcloud import dense_cloud_bytes

def stamp_from_seconds(seconds):
    whole = math.floor(seconds)
    nano = round((seconds-whole)*1_000_000_000)
    if nano >= 1_000_000_000:
        whole += 1
        nano -= 1_000_000_000
    return int(whole), int(nano)

def set_stamp(stamp, seconds):
    stamp.sec, stamp.nanosec = stamp_from_seconds(seconds)

def xyz(message, values):
    message.x, message.y, message.z = map(float, values)

def quaternion(message, values):
    message.x, message.y, message.z, message.w = map(float, values)

class GodotBridge(Node):
    def __init__(self):
        super().__init__('godot_bridge')
        defaults = {
            'host':'127.0.0.1','port':9090,'token':'',
            'reconnect_seconds':1.0,'ros_command_timeout_s':0.25,
            'cmd_vel_topic':'/cmd_vel','cmd_vel_stamped':False,
            'odom_topic':'/odom','ground_truth_topic':'/ground_truth/pose',
            'imu_topic':'/imu/data','publish_raw_points':False,
            'points_topic':'/points_raw','lio_points_topic':'/points_lio','lidar_frame':'lidar36_link',
            'imu_position_ros':[0.335,0.0,2.55],'publish_odom_tf':True,
            'clock_topic':'/clock','status_topic':'/sim/status',
            'camera_topic':'/camera/image_raw/compressed','camera_info_topic':'/camera/camera_info',
            'camera_frame':'camera_optical_frame',
            'odom_frame':'odom','base_frame':'base_link',
            'imu_frame':'imu_link',
            'publish_clock':True,'publish_tf':True
        }
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.params = {key:self.get_parameter(key).value for key in defaults}
        if not self.params['token']:
            raise RuntimeError('Configure the simulator token in config/bridge.yaml')
        self.transport = SimTransport(self.params['host'],self.params['port'],
                                      self.params['token'],self.params['reconnect_seconds'])
        self.last_command = (0.,0.,-math.inf)
        self.command_seq = 0
        self.last_state_seq = None
        self.last_cloud = None
        self.lidar_mount = tuple(self.params['imu_position_ros'])
        self.last_imu_key = None
        self.has_high_rate_imu = False
        self.was_connected = None
        self.connected_since = 0.
        self.odom_pub = self.create_publisher(Odometry,self.params['odom_topic'],10)
        self.truth_pub = self.create_publisher(PoseStamped,self.params['ground_truth_topic'],10)
        self.imu_pub = self.create_publisher(Imu,self.params['imu_topic'],qos_profile_sensor_data)
        self.points_pub = self.create_publisher(PointCloud2,self.params['points_topic'],qos_profile_sensor_data) if self.params['publish_raw_points'] else None
        self.lio_points_pub = self.create_publisher(PointCloud2,self.params['lio_points_topic'],qos_profile_sensor_data)
        self.clock_pub = self.create_publisher(Clock,self.params['clock_topic'],10)
        self.status_pub = self.create_publisher(String,self.params['status_topic'],10)
        self.camera_pub = self.create_publisher(CompressedImage,self.params['camera_topic'],qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(CameraInfo,self.params['camera_info_topic'],qos_profile_sensor_data)
        connection_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.connection_pub = self.create_publisher(Bool,'/sim/connected',connection_qos)
        command_type = TwistStamped if self.params['cmd_vel_stamped'] else Twist
        self.create_subscription(command_type,self.params['cmd_vel_topic'],self.on_command,10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_service(Trigger,'/sim/reset',self.on_reset)
        self.create_service(SetBool,'/sim/pause',partial(self.on_flag,'pause'))
        self.create_service(SetBool,'/sim/estop',partial(self.on_flag,'estop'))
        self.create_service(SetBool,'/sim/set_remote',self.on_remote)
        self.wall_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02,self.tick,clock=self.wall_clock)
        self.send_static_transforms()
        self.get_logger().info(f"Connecting to independent simulator {self.params['host']}:{self.params['port']}")

    def send_static_transforms(self):
        transforms = []
        # The 36-line lidar and virtual IMU share a calibrated front-module origin.
        for child,position in [(self.params['imu_frame'],self.lidar_mount),(self.params['lidar_frame'],self.lidar_mount)]:
            item = TransformStamped()
            item.header.frame_id = self.params['base_frame']
            item.child_frame_id = child
            xyz(item.transform.translation,position)
            item.transform.rotation.w = 1.
            transforms.append(item)
        optical = TransformStamped()
        optical.header.frame_id = self.params['base_frame']
        optical.child_frame_id = self.params['camera_frame']
        optical.transform.translation.x = .7
        optical.transform.translation.z = 1.5
        quaternion(optical.transform.rotation,[-.5,.5,-.5,.5])
        transforms.append(optical)
        if self.params['publish_tf']:
            self.static_tf.sendTransform(transforms)

    def on_command(self,message):
        twist = message.twist if isinstance(message,TwistStamped) else message
        linear,angular = float(twist.linear.x),float(twist.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().warning('Rejected non-finite velocity command')
            return
        self.last_command = (linear,angular,time.monotonic())

    def on_reset(self,request,response):
        self.last_command = (0.,0.,-math.inf)
        response.success,response.message = self.transport.operation('reset')
        return response

    def on_flag(self,operation,request,response):
        self.last_command = (0.,0.,-math.inf)
        response.success,response.message = self.transport.operation(operation,enabled=bool(request.data))
        return response

    def on_remote(self,request,response):
        self.last_command = (0.,0.,-math.inf)
        response.success,response.message = self.transport.operation('set_mode',mode='ros' if request.data else 'manual')
        return response

    def tick(self):
        connected = self.transport.authenticated.is_set()
        if connected != self.was_connected:
            self.was_connected = connected
            self.last_command = (0.,0.,-math.inf)
            self.last_state_seq = None
            self.last_cloud = None
            self.last_imu_key = None
            self.has_high_rate_imu = False
            self.connection_pub.publish(Bool(data=connected))
            self.get_logger().info('Simulator connected' if connected else 'Simulator disconnected; commands cleared')
        if not connected:
            return
        linear,angular,received = self.last_command
        if time.monotonic()-received > self.params['ros_command_timeout_s']:
            linear=angular=0.
        self.command_seq += 1
        self.transport.command(linear,angular,self.command_seq)
        for sample in self.transport.take_imu_samples():
            self.publish_imu_sample(sample)
        cloud = self.transport.take_cloud()
        if cloud is not None:
            self.publish_cloud(cloud)
        state = self.transport.take_state()
        if state is None or state['seq'] == self.last_state_seq:
            return
        self.last_state_seq = state['seq']
        self.publish_state(state)
        image = self.transport.take_image()
        if image is not None:
            self.publish_image(image)

    def publish_state(self,state):
        t = float(state['sim_time'])
        if self.params['publish_clock']:
            clock = Clock()
            set_stamp(clock.clock,t)
            self.clock_pub.publish(clock)
        odom = Odometry()
        set_stamp(odom.header.stamp,t)
        odom.header.frame_id = self.params['odom_frame']
        odom.child_frame_id = self.params['base_frame']
        xyz(odom.pose.pose.position,state['pose']['position'])
        quaternion(odom.pose.pose.orientation,state['pose']['orientation'])
        xyz(odom.twist.twist.linear,state['twist']['linear'])
        xyz(odom.twist.twist.angular,state['twist']['angular'])
        # Zero covariance is intentional: version 0.1 exposes ideal simulated odometry.
        self.odom_pub.publish(odom)
        truth = PoseStamped()
        truth.header = odom.header
        truth.pose = odom.pose.pose
        self.truth_pub.publish(truth)
        if self.params['publish_tf'] and self.params['publish_odom_tf']:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.params['base_frame']
            xyz(transform.transform.translation,state['pose']['position'])
            quaternion(transform.transform.rotation,state['pose']['orientation'])
            self.tf.sendTransform(transform)
        imu = Imu()
        set_stamp(imu.header.stamp,t)
        imu.header.frame_id = self.params['imu_frame']
        quaternion(imu.orientation,state['imu']['orientation'])
        xyz(imu.angular_velocity,state['imu']['angular_velocity'])
        xyz(imu.linear_acceleration,state['imu']['linear_acceleration'])
        if not self.has_high_rate_imu:
            self.imu_pub.publish(imu)
        self.status_pub.publish(String(data=json.dumps({
            **{key:state[key] for key in ['mode','paused','estop','watchdog_stopped','epoch','sim_time']},
            'visual_wheels':state.get('visual_wheels',{})
        })))

    def publish_cloud(self,packet):
        key=(packet['epoch'],packet['seq'])
        if key==self.last_cloud:
            return
        self.last_cloud=key
        mount=tuple(packet['position_ros'])
        if mount!=self.lidar_mount:
            self.lidar_mount=mount
            self.send_static_transforms()
        message=PointCloud2()
        set_stamp(message.header.stamp,float(packet['stamp']))
        message.header.frame_id=self.params['lidar_frame']
        message.height=packet['height']
        message.width=packet['width']
        message.fields=[PointField(name=name,offset=offset,datatype=kind,count=1) for name,offset,kind in [
            ('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),('z',8,PointField.FLOAT32),
            ('intensity',12,PointField.FLOAT32),('ring',16,PointField.UINT16),('time',20,PointField.FLOAT32)]]
        message.is_bigendian=False
        message.point_step=24
        message.row_step=message.width*24
        message.is_dense=False
        message.data=packet['_decoded_data']
        if self.points_pub is not None:
            self.points_pub.publish(message)
        dense=PointCloud2()
        dense.header=message.header
        dense.fields=message.fields
        dense.height=1
        dense.data=dense_cloud_bytes(message.data)
        dense.width=len(dense.data)//24
        dense.point_step=24
        dense.row_step=dense.width*24
        dense.is_bigendian=False
        dense.is_dense=True
        if dense.width:
            self.lio_points_pub.publish(dense)

    def publish_imu_sample(self,packet):
        key=(packet['epoch'],packet['seq'])
        if key==self.last_imu_key:
            return
        self.last_imu_key=key
        self.has_high_rate_imu=True
        mount=tuple(packet['position_ros'])
        if mount!=self.lidar_mount:
            self.lidar_mount=mount
            self.send_static_transforms()
        message=Imu()
        set_stamp(message.header.stamp,float(packet['stamp']))
        message.header.frame_id=self.params['imu_frame']
        quaternion(message.orientation,packet['orientation'])
        xyz(message.angular_velocity,packet['angular_velocity'])
        xyz(message.linear_acceleration,packet['linear_acceleration'])
        self.imu_pub.publish(message)

    def publish_image(self,packet):
        try:
            width,height = int(packet['width']),int(packet['height'])
            if not (0<width<=1920 and 0<height<=1080):
                return
            data = base64.b64decode(packet['data'],validate=True)
            if not data.startswith(b'\xff\xd8'):
                return
            message = CompressedImage()
            set_stamp(message.header.stamp,float(packet['sim_time']))
            message.header.frame_id = self.params['camera_frame']
            message.format = 'jpeg'
            message.data = data
            info = CameraInfo()
            info.header = message.header
            info.width,info.height = width,height
            fx,fy,cx,cy = [float(packet[k]) for k in ('fx','fy','cx','cy')]
            info.distortion_model = 'plumb_bob'
            info.d = [0.]*5
            info.k = [fx,0.,cx,0.,fy,cy,0.,0.,1.]
            info.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
            info.p = [fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
            self.camera_pub.publish(message)
            self.camera_info_pub.publish(info)
        except (KeyError,TypeError,ValueError,binascii.Error):
            self.get_logger().warning('Invalid camera packet ignored')

    def destroy_node(self):
        # Best-effort stop; Godot also independently enforces its command watchdog.
        if self.transport.authenticated.is_set():
            self.command_seq += 1
            self.transport.command(0.,0.,self.command_seq)
            time.sleep(.03)
        self.transport.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler)
    node = None
    try:
        node = GodotBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
