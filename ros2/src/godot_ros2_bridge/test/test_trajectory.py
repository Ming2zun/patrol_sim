import json
from types import SimpleNamespace
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from godot_ros2_bridge.trajectory import Trajectory

class Fake:
    odom=Trajectory.odom
    status=Trajectory.status
    def __init__(self):
        self.path=Path();self.max_points=3;self.min_distance=.05;self.epoch=None;self.last_stamp=None;self.last_sim_time=None;self.dirty=False
    def clear(self):self.path.poses.clear();self.last_stamp=None;self.dirty=True

def msg(x,z=0.,stamp=1,frame='map'):
    m=Odometry();m.header.frame_id=frame;m.header.stamp.sec=stamp;m.pose.pose.position.x=float(x);m.pose.pose.position.z=float(z);m.pose.pose.orientation.w=1.;return m

def test_stationary_noise_does_not_grow_history_and_height_is_preserved():
    t=Fake();t.odom(msg(0,z=20));t.odom(msg(.01,z=20,stamp=2));assert len(t.path.poses)==1
    t.odom(msg(.2,z=21,stamp=3));assert [p.pose.position.z for p in t.path.poses]==[20.,21.]

def test_history_is_bounded_and_rejects_invalid_frames_and_nan():
    t=Fake()
    for i in range(6):t.odom(msg(i,stamp=i))
    assert [p.pose.position.x for p in t.path.poses]==[3.,4.,5.]
    t.odom(msg(10,stamp=7,frame='odom'));t.odom(msg(float('nan'),stamp=8));assert len(t.path.poses)==3

def test_reset_and_clock_rewind_clear_old_history():
    t=Fake();t.status(String(data=json.dumps({'epoch':0,'sim_time':10})));t.odom(msg(1,stamp=10))
    t.status(String(data=json.dumps({'epoch':1,'sim_time':11})));assert not t.path.poses
    t.odom(msg(2,stamp=11));t.status(String(data=json.dumps({'epoch':1,'sim_time':0})));assert not t.path.poses
    t.odom(msg(5,stamp=5));t.odom(msg(1,stamp=1));assert len(t.path.poses)==1
