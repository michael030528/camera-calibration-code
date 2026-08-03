"""Direct Livox MID-360/HAP UDP receiver, ported from JSHZRSYYD/LVX2toPCD."""
import math, socket, struct, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

def crc16(data):
    crc=0xffff
    for b in data:
        crc ^= b<<8
        for _ in range(8): crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc

def crc32(data):
    crc=0xffffffff
    for b in data:
        crc ^= b
        for _ in range(8): crc=(crc>>1)^0xedb88320 if crc&1 else crc>>1
    return crc^0xffffffff

def command(cmd,payload=b'',seq=1):
    out=bytearray(24+len(payload)); out[0]=0xaa
    struct.pack_into('<HIH',out,2,len(out),seq&0xffffffff,cmd)
    struct.pack_into('<H',out,18,crc16(out[:18])); struct.pack_into('<I',out,20,crc32(payload or b'\0'))
    out[24:]=payload; return bytes(out)

def parse_command(data):
    if len(data)<24 or data[0]!=0xaa: return None
    length=struct.unpack_from('<H',data,2)[0]
    if length>len(data) or crc16(data[:18])!=struct.unpack_from('<H',data,18)[0]: return None
    payload=data[24:length]
    if crc32(payload or b'\0')!=struct.unpack_from('<I',data,20)[0]: return None
    return struct.unpack_from('<H',data,8)[0],payload

def kv(key,value): return struct.pack('<HH',key,len(value))+value

class LivoxDirect(Node):
    def __init__(self):
        super().__init__('livox_direct')
        self.declare_parameter('lidar_ip','192.168.1.152'); self.declare_parameter('host_ip','192.168.1.41')
        self.declare_parameter('frame_id','livox_frame'); self.declare_parameter('topic','/livox/lidar')
        self.declare_parameter('point_stride',10)
        self.lidar_ip=self.get_parameter('lidar_ip').value; self.host_ip=self.get_parameter('host_ip').value
        self.frame_id=self.get_parameter('frame_id').value
        self.point_stride=max(1,int(self.get_parameter('point_stride').value))
        self.pub=self.create_publisher(PointCloud2,self.get_parameter('topic').value,qos_profile_sensor_data)
        self.imu_pub=self.create_publisher(Imu,'/livox/imu',qos_profile_sensor_data)
        self.lock=threading.Lock(); self.points=[]; self.stop=False; self.packet_count=0
        self.clock_lock=threading.Lock()
        self.device_clock_offset_ns=None; self.last_time_type=None
        self.first_publish=True
        self.published_frames=0; self.report_at=time.monotonic()+5
        self.device_type,self.command_port=self.discover()
        self.point_port=56301 if self.device_type==9 else 57000
        self.lidar_point_port=56300 if self.device_type==9 else 57000
        self.host_command_port=56101 if self.device_type==9 else 56000
        self.imu_ports=(56401,56400) if self.device_type==9 else (58000,58000)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024); self.sock.bind((self.host_ip,self.point_port)); self.sock.settimeout(.2)
        self.imu_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.imu_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.imu_sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024); self.imu_sock.bind((self.host_ip,self.imu_ports[0])); self.imu_sock.settimeout(.2)
        self.configure()
        threading.Thread(target=self.receive,daemon=True,name='livox-points').start()
        threading.Thread(target=self.receive_imu,daemon=True,name='livox-imu').start(); self.create_timer(.1,self.publish)
        self.get_logger().info(f'Livox direct UDP connected: type={self.model} {self.lidar_ip} -> {self.host_ip}:{self.point_port}')

    @property
    def model(self): return 'MID-360' if self.device_type==9 else 'HAP'

    def discover(self):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        s.bind((self.host_ip,0)); s.settimeout(.25)
        info=struct.pack('<HHHH',2,0,0x8000,0x8001)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            for port in (56100,56000):
                s.sendto(command(0x0101,info,int(time.time()*1000)),(self.lidar_ip,port))
            try:
                data,addr=s.recvfrom(2048); parsed=parse_command(data)
                if parsed and parsed[0]==0x0101 and parsed[1] and parsed[1][0]==0:
                    s.close(); return (9,56100) if addr[1]==56100 else (10,56000)
            except socket.timeout: pass
        s.close(); raise RuntimeError(f'Livox at {self.lidar_ip} did not answer UDP 56000/56100')

    def configure(self):
        ip=socket.inet_aton(self.host_ip)
        values=(kv(0x0000,b'\x01')+kv(0x0006,ip+struct.pack('<HH',self.point_port,self.lidar_point_port))+
                kv(0x0007,ip+struct.pack('<HH',*self.imu_ports))+kv(0x001a,b'\x01')+kv(0x001c,b'\x01'))
        self.send_parameters(struct.pack('<HH',5,0)+values)
        # LVX2toPCD sends this as a separate command for HAP after stream config.
        if self.device_type!=9: self.send_parameters(struct.pack('<HH',1,0)+kv(0x0003,b'\x00'))

    def send_parameters(self,payload):
        cmdsock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); cmdsock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try: cmdsock.bind((self.host_ip,self.host_command_port))
        except OSError: cmdsock.bind((self.host_ip,0))
        cmdsock.settimeout(2.2); cmdsock.sendto(command(0x0100,payload,int(time.time()*1000)),(self.lidar_ip,self.command_port))
        try:
            while True:
                data,_=cmdsock.recvfrom(2048); parsed=parse_command(data)
                if parsed and parsed[0]==0x0100:
                    if not parsed[1] or parsed[1][0]!=0: raise RuntimeError('Livox rejected stream configuration')
                    break
        except socket.timeout: raise RuntimeError('Livox stream configuration timeout; close Livox Viewer/other drivers')
        finally: cmdsock.close()

    def ros_stamp_ns(self, device_ns, time_type, arrival_ns):
        """Map both point and IMU device stamps into the same ROS clock domain."""
        if time_type==1 and device_ns>1_000_000_000_000_000_000:
            return device_ns
        observed=arrival_ns-device_ns
        with self.clock_lock:
            if self.device_clock_offset_ns is None or observed<self.device_clock_offset_ns:
                self.device_clock_offset_ns=observed
            return device_ns+self.device_clock_offset_ns

    def receive(self):
        while not self.stop:
            try: data,addr=self.sock.recvfrom(65535)
            except socket.timeout: continue
            except OSError: break
            if addr[0]!=self.lidar_ip or len(data)<36: continue
            if self.packet_count==0: self.get_logger().info(f'First Livox packet: {len(data)} bytes from {addr}')
            arrival_ns=self.get_clock().now().nanoseconds
            declared,count=struct.unpack_from('<HH',data,1)[0],struct.unpack_from('<H',data,5)[0]
            interval_ns=struct.unpack_from('<H',data,3)[0]*100
            time_type=data[11]; device_ns=struct.unpack_from('<Q',data,28)[0]
            # PTP/GPS timestamps are epoch based. Unsynchronized HAP timestamps
            # are device uptime; map them to ROS time using the minimum observed
            # arrival offset, which rejects variable network/queueing delay.
            packet_start_ns=self.ros_stamp_ns(device_ns,time_type,arrival_ns)
            self.last_time_type=time_type
            dtype=data[10]; size={1:14,2:8,3:10}.get(dtype)
            if not size: continue
            count=min(count,(min(len(data),declared)-36)//size); frame=[]
            for i in range(0,max(0,count),self.point_stride):
                o=36+i*size
                if dtype==1: x,y,z=struct.unpack_from('<iii',data,o); x*=.001; y*=.001; z*=.001; intensity=data[o+12]
                elif dtype==2: x,y,z=struct.unpack_from('<hhh',data,o); x*=.01; y*=.01; z*=.01; intensity=data[o+6]
                else:
                    radius,theta,phi=struct.unpack_from('<IHH',data,o); radius*=.001; theta*=.01*math.pi/180; phi*=.01*math.pi/180
                    x=radius*math.sin(theta)*math.cos(phi); y=radius*math.sin(theta)*math.sin(phi); z=radius*math.cos(theta); intensity=data[o+8]
                point_ns=packet_start_ns+(interval_ns*i//max(1,count-1))
                if x or y or z: frame.append((float(x),float(y),float(z),float(intensity),point_ns))
            if frame:
                with self.lock: self.points.extend(frame)
                self.packet_count+=1

    def receive_imu(self):
        reported=False
        while not self.stop:
            try: data,addr=self.imu_sock.recvfrom(2048)
            except socket.timeout: continue
            except OSError: break
            if addr[0]!=self.lidar_ip or len(data)<60 or data[10]!=0: continue
            arrival_ns=self.get_clock().now().nanoseconds
            device_ns=struct.unpack_from('<Q',data,28)[0]
            stamp_ns=self.ros_stamp_ns(device_ns,data[11],arrival_ns)
            gx,gy,gz,ax,ay,az=struct.unpack_from('<ffffff',data,36)
            msg=Imu(); msg.header.frame_id=self.frame_id
            msg.header.stamp.sec=stamp_ns//1_000_000_000; msg.header.stamp.nanosec=stamp_ns%1_000_000_000
            msg.orientation_covariance[0]=-1.0  # MID360 IMU does not output orientation.
            msg.angular_velocity.x=float(gx); msg.angular_velocity.y=float(gy); msg.angular_velocity.z=float(gz)
            gravity=9.80665
            msg.linear_acceleration.x=float(ax)*gravity; msg.linear_acceleration.y=float(ay)*gravity; msg.linear_acceleration.z=float(az)*gravity
            self.imu_pub.publish(msg)
            if not reported:
                self.get_logger().info('Publishing /livox/imu (gyro rad/s, acceleration m/s^2)')
                reported=True

    def publish(self):
        with self.lock: pts,self.points=self.points,[]
        if not pts: return
        raw=np.asarray(pts,dtype=np.float64)
        start_ns=int(raw[:,4].min()); end_ns=int(raw[:,4].max()); midpoint=(start_ns+end_ns)//2
        header=Header(); header.stamp.sec=midpoint//1_000_000_000; header.stamp.nanosec=midpoint%1_000_000_000; header.frame_id=self.frame_id
        fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
                PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),PointField(name='intensity',offset=12,datatype=PointField.FLOAT32,count=1),
                PointField(name='time_offset_ns',offset=16,datatype=PointField.INT32,count=1)]
        dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('time_offset_ns','<i4')])
        cloud_pts=np.empty(len(raw),dtype=dtype)
        cloud_pts['x']=raw[:,0]; cloud_pts['y']=raw[:,1]; cloud_pts['z']=raw[:,2]
        cloud_pts['intensity']=raw[:,3]; cloud_pts['time_offset_ns']=(raw[:,4]-midpoint).astype(np.int32)
        self.pub.publish(point_cloud2.create_cloud(header,fields,cloud_pts))
        self.published_frames+=1
        if time.monotonic()>=self.report_at:
            self.get_logger().info(f'Livox healthy: {self.published_frames/5:.1f} Hz published, latest={len(raw)} points')
            self.published_frames=0; self.report_at=time.monotonic()+5
        if self.first_publish:
            self.get_logger().info(f'Publishing /livox/lidar: {len(pts)} points, span={(end_ns-start_ns)/1e6:.1f}ms, time_type={self.last_time_type}')
            self.first_publish=False

    def destroy_node(self):
        self.stop=True
        if hasattr(self,'sock'): self.sock.close()
        if hasattr(self,'imu_sock'): self.imu_sock.close()
        super().destroy_node()

def main():
    rclpy.init(); node=None
    try: node=LivoxDirect(); rclpy.spin(node)
    except Exception as e:
        if node: node.get_logger().fatal(str(e))
        else: print(f'Livox direct error: {e}')
        raise
    finally:
        if node: node.destroy_node()
        rclpy.try_shutdown()
