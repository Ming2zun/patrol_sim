"""ROS-independent JSONL transport with bounded queues and reconnect isolation."""
import json
import math
import queue
import select
import socket
import threading
import time
import uuid
from .pointcloud import decode_cloud,valid_imu
from collections import deque

MAX_BUFFER = 2097152
MAX_LINE = 1048576

def encode(packet):
    return (json.dumps(packet, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')

class JsonLines:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > MAX_BUFFER:
            raise ValueError('receive buffer limit')
        packets = []
        while b'\n' in self.buffer:
            line, _, remainder = self.buffer.partition(b'\n')
            if len(line) > MAX_LINE:
                raise ValueError('line limit')
            self.buffer = bytearray(remainder)
            packet = json.loads(line)
            if not isinstance(packet, dict):
                raise ValueError('expected JSON object')
            packets.append(packet)
        if len(self.buffer) > MAX_LINE:
            raise ValueError('line limit')
        return packets

def valid_state(packet):
    try:
        if packet.get('type') != 'state' or packet.get('version') != 1:
            return False
        groups = [packet['pose']['position'], packet['pose']['orientation'],
                  packet['twist']['linear'], packet['twist']['angular'],
                  packet['imu']['orientation'], packet['imu']['angular_velocity'],
                  packet['imu']['linear_acceleration']]
        if [len(g) for g in groups] != [3,4,3,3,4,3,3]:
            return False
        values = [v for g in groups for v in g] + [packet['sim_time']]
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values):
            return False
        if packet['sim_time'] < 0 or abs(sum(v*v for v in packet['pose']['orientation'])-1) > .02:
            return False
        if 'laser' not in packet:
            return True
        laser = packet['laser']
        if len(laser['ranges']) > 4096:
            return False
        return all(v is None or (isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and v >= 0) for v in laser['ranges'])
    except (KeyError, TypeError, ValueError):
        return False

class SimTransport:
    def __init__(self, host, port, token, reconnect_seconds=1.0):
        self.host, self.port, self.token = host, int(port), token
        self.reconnect_seconds = reconnect_seconds
        self.authenticated = threading.Event()
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.latest_state = None
        self.latest_image = None
        self.latest_cloud = None
        self.imu_samples = deque(maxlen=512)
        self.pending_command = None
        self.operations = queue.Queue(maxsize=32)
        self.pending_acks = {}
        self.last_error = ''
        self.thread = threading.Thread(target=self._run, daemon=True, name='godot-network')
        self.thread.start()

    def take_state(self):
        with self.lock:
            result, self.latest_state = self.latest_state, None
            return result

    def take_image(self):
        with self.lock:
            result, self.latest_image = self.latest_image, None
            return result

    def take_imu_samples(self):
        with self.lock:
            samples=list(self.imu_samples)
            self.imu_samples.clear()
            return samples

    def take_cloud(self):
        with self.lock:
            result, self.latest_cloud = self.latest_cloud, None
            return result

    def command(self, linear, angular, sequence):
        if not self.authenticated.is_set() or not all(math.isfinite(v) for v in (linear,angular)):
            return
        with self.lock:
            self.pending_command = (time.monotonic(), {'type':'cmd_vel','linear_x':linear,'angular_z':angular,'seq':sequence})

    def operation(self, kind, timeout=1.5, **fields):
        if not self.authenticated.is_set():
            return False, 'Simulator is disconnected'
        request_id = uuid.uuid4().hex
        event = threading.Event()
        item = {'event':event, 'reply':None}
        with self.lock:
            self.pending_command = None
            self.pending_acks[request_id] = item
        try:
            self.operations.put_nowait({'type':kind, 'request_id':request_id, **fields})
        except queue.Full:
            with self.lock:
                self.pending_acks.pop(request_id, None)
            return False, 'Operation queue full'
        finished = event.wait(timeout)
        with self.lock:
            self.pending_acks.pop(request_id,None)
        reply = item['reply'] or {}
        return bool(finished and reply.get('ok')), str(reply.get('message', 'Acknowledged' if finished and reply.get('ok') else 'Operation timed out/disconnected'))

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=3)

    def _clear_connection(self):
        self.authenticated.clear()
        with self.lock:
            self.pending_command = None
            self.latest_state = None
            self.latest_image = None
            self.latest_cloud = None
            self.imu_samples.clear()
            for item in self.pending_acks.values():
                item['reply'] = {'ok':False,'message':'Disconnected'}
                item['event'].set()
        while True:
            try: self.operations.get_nowait()
            except queue.Empty: break

    def _run(self):
        while not self.stopping.is_set():
            self._clear_connection()
            try:
                with socket.create_connection((self.host,self.port), timeout=2) as sock:
                    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    sock.settimeout(2)
                    sock.sendall(encode({'type':'hello','version':1,'token':self.token,'capabilities':['pointcloud','imu_sample']}))
                    sock.setblocking(False)
                    parser = JsonLines()
                    outgoing = bytearray()
                    connected_at = time.monotonic()
                    last_receive = connected_at
                    while not self.stopping.is_set():
                        now = time.monotonic()
                        if now-last_receive > 5:
                            raise TimeoutError('Simulator telemetry timeout')
                        if not self.authenticated.is_set() and now-connected_at > 3:
                            raise TimeoutError('Authentication timeout')
                        if self.authenticated.is_set() and len(outgoing)<32768:
                            try:
                                outgoing.extend(encode(self.operations.get_nowait()))
                            except queue.Empty:
                                pass
                            with self.lock:
                                cmd, self.pending_command = self.pending_command, None
                            if cmd is not None and now-cmd[0]<.25:
                                outgoing.extend(encode(cmd[1]))
                        readable,writable,_ = select.select([sock],[sock] if outgoing else [],[],.02)
                        if readable:
                            data = sock.recv(65536)
                            if not data:
                                raise ConnectionError('Simulator closed the connection')
                            last_receive = now
                            for packet in parser.feed(data):
                                if packet.get('type')=='hello' and packet.get('version')==1:
                                    self.authenticated.set()
                                elif packet.get('type')=='ack':
                                    with self.lock:
                                        item=self.pending_acks.get(packet.get('request_id'))
                                        if item:
                                            item['reply']=packet
                                            item['event'].set()
                                elif self.authenticated.is_set() and packet.get('type')=='image' and packet.get('format')=='jpeg':
                                    if isinstance(packet.get('data'),str) and len(packet['data'])<=60000:
                                        with self.lock:
                                            self.latest_image=packet
                                elif self.authenticated.is_set() and valid_imu(packet):
                                    with self.lock:
                                        self.imu_samples.append(packet)
                                elif self.authenticated.is_set() and packet.get('type')=='pointcloud':
                                    try:
                                        packet['_decoded_data']=decode_cloud(packet)
                                    except (ValueError,KeyError,TypeError):
                                        continue
                                    with self.lock:
                                        self.latest_cloud=packet
                                elif self.authenticated.is_set() and valid_state(packet):
                                    with self.lock:
                                        self.latest_state=packet
                        if writable and outgoing:
                            sent=sock.send(outgoing)
                            del outgoing[:sent]
            except (OSError,ValueError,TimeoutError) as exc:
                self.last_error=str(exc)
            finally:
                self._clear_connection()
            self.stopping.wait(self.reconnect_seconds)
