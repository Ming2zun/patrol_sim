import signal
"""RViz vehicle primitives driven by simulator wheel telemetry; no ground geometry."""
import json,math
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray

class VehicleMarkers(Node):
    def __init__(self):
        super().__init__('alpine_vehicle_markers')
        self.wheels={}
        self.pub=self.create_publisher(MarkerArray,'/sim/vehicle_markers',1)
        self.create_subscription(String,'/sim/status',self.status,10)
        self.create_timer(.1,self.publish)
    def status(self,msg):
        try:self.wheels=json.loads(msg.data).get('visual_wheels',{})
        except (ValueError,TypeError):pass
    def publish(self):
        array=MarkerArray()
        def add(kind,pos,size,color,q=(0.,0.,0.,1.)):
            m=Marker();m.header.frame_id='base_link';m.frame_locked=True
            m.ns='alpine_vehicle';m.id=len(array.markers);m.type=kind;m.action=Marker.ADD
            m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,pos)
            m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
            m.scale.x,m.scale.y,m.scale.z=map(float,size)
            m.color.r,m.color.g,m.color.b,m.color.a=map(float,color)
            array.markers.append(m)
        add(Marker.CUBE,(0,0,1.05),(2.15,1.35,.5),(.8,.86,.91,1))
        add(Marker.CUBE,(0,0,1.4),(1.5,1.25,.3),(.65,.73,.8,1))
        add(Marker.CYLINDER,(.335,0,2.),(.18,.18,1.),(.3,.35,.4,1))
        add(Marker.CYLINDER,(.335,0,2.55),(.23,.23,.16),(.1,.25,.4,1))
        add(Marker.ARROW,(1.1,0,1.4),(.7,.1,.1),(1,.6,.05,1))
        centers=self.wheels.get('centers_godot',[[-.91,.35,-.78],[.91,.35,-.78],[-.91,.35,.78],[.91,.35,.78]])
        radius=float(self.wheels.get('radius_m',.35));steers=self.wheels.get('steering_rad',[0.]*4)
        offsets=self.wheels.get('suspension_m',[0.]*4)
        for i,c in enumerate(centers[:4]):
            angle=float(steers[i]);a=math.sqrt(.5)
            q=(a*math.cos(angle/2),a*math.sin(angle/2),a*math.sin(angle/2),a*math.cos(angle/2))
            add(Marker.CYLINDER,(-c[2],-c[0],c[1]+offsets[i]),(radius*2,radius*2,.28),(.08,.08,.08,1),q)
        self.pub.publish(array)
def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler);node=VehicleMarkers()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
