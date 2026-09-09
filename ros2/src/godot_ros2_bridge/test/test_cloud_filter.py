import numpy as np
import pytest
from godot_ros2_bridge.cloud_filter import RollingCloud
from godot_ros2_bridge.segmented_cloud import xyz_from_cloud
from sensor_msgs.msg import PointCloud2,PointField


def test_height_boundary_radius_and_non_finite_points():
    grid=RollingCloud()
    grid.add([[0,0,.1],[0,0,.15],[80,0,1],[80.01,0,1],[np.nan,0,1],[1,0,np.inf]],(0,0,0))
    assert len(grid.cells)==2
    assert np.min(grid.points()[:,2])>=.15


def test_accumulates_beyond_single_scan_range_and_prunes_when_vehicle_moves():
    grid=RollingCloud()
    grid.add([[0,0,1]],(0,0,0))
    grid.add([[60,0,1]],(60,0,0))
    grid.prune((60,0,0))
    assert len(grid.cells)==2  # The old point survives despite being >30 m away.
    grid.prune((81,0,0))
    assert grid.points().tolist()==[[60.,0.,1.]]


def test_voxel_deduplication_reset_and_capacity():
    grid=RollingCloud(max_voxels=2)
    grid.add([[0,0,1],[.01,.01,1.01]],(0,0,0))
    assert len(grid.cells)==1
    grid.add([[2,0,1],[3,0,1]],(0,0,0))
    assert len(grid.cells)==2 and grid.evicted==1
    grid.clear();assert grid.points().shape==(0,3)


def test_cloud_parser_supports_padding_and_endianness():
    msg=PointCloud2();msg.height=2;msg.width=1;msg.point_step=16;msg.row_step=24;msg.is_bigendian=True
    msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
    data=bytearray(48)
    data[:12]=np.array([1,2,3],dtype='>f4').tobytes()
    data[24:36]=np.array([4,5,6],dtype='>f4').tobytes()
    msg.data=bytes(data)
    assert xyz_from_cloud(msg).tolist()==[[1,2,3],[4,5,6]]
    msg.row_step=8
    with pytest.raises(ValueError):xyz_from_cloud(msg)
