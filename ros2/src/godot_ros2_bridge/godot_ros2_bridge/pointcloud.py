"""ROS-independent validation for organized 36-ring XYZ/intensity/ring data."""
import base64
import math
import struct

POINT_STEP = 24
POINT_STRUCT = struct.Struct('<ffffHxxf')

def decode_cloud(packet):
    if packet.get('type')!='pointcloud' or packet.get('version')!=1:
        raise ValueError('unsupported pointcloud packet')
    width,height=packet['width'],packet['height']
    if type(width) is not int or type(height) is not int or height!=36 or not 90<=width<=720:
        raise ValueError('invalid organized dimensions')
    if packet.get('point_step')!=24 or packet.get('row_step')!=width*24 or packet.get('is_bigendian') is not False:
        raise ValueError('invalid binary layout')
    if packet.get('encoding')!='xyz_irt_f32_u16_le' or packet.get('snapshot') is not True:
        raise ValueError('unsupported scan semantics')
    stamp=packet['stamp']
    if not isinstance(stamp,(int,float)) or isinstance(stamp,bool) or not math.isfinite(stamp) or stamp<0:
        raise ValueError('invalid scan time')
    mount=packet['position_ros']
    if len(mount)!=3 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and abs(v)<10 for v in mount):
        raise ValueError('invalid sensor mount')
    encoded=packet['data'];expected=width*height*24
    if not isinstance(encoded,str) or len(encoded)!=4*((expected+2)//3):
        raise ValueError('invalid payload size')
    raw=base64.b64decode(encoded,validate=True)
    if len(raw)!=expected:
        raise ValueError('truncated payload')
    for i,(x,y,z,intensity,ring,relative_time) in enumerate(POINT_STRUCT.iter_unpack(raw)):
        if relative_time!=0.0 or ring!=i//width or not math.isfinite(intensity) or not 0<=intensity<=1:
            raise ValueError('invalid ring or intensity')
        if not (all(math.isfinite(v) for v in (x,y,z)) or all(math.isnan(v) for v in (x,y,z))):
            raise ValueError('invalid return coordinates')
    return raw


def dense_cloud_bytes(raw):
    """Drop no-return slots for LIO while retaining ring IDs and snapshot time=0."""
    return b''.join(raw[i*POINT_STEP:(i+1)*POINT_STEP] for i,p in enumerate(POINT_STRUCT.iter_unpack(raw)) if all(math.isfinite(v) for v in p[:3]))


def valid_imu(packet):
    try:
        if packet.get('type')!='imu_sample' or packet.get('version')!=1:
            return False
        groups=[packet['orientation'],packet['angular_velocity'],packet['linear_acceleration'],packet['position_ros']]
        if [len(g) for g in groups]!=[4,3,3,3]:
            return False
        values=[v for group in groups for v in group]+[packet['stamp']]
        return all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values) and packet['stamp']>=0 and abs(sum(v*v for v in packet['orientation'])-1)<.02
    except (KeyError,TypeError,ValueError):
        return False
