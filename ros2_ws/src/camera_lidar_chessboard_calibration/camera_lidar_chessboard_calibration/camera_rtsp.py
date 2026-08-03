import os, threading, time
from urllib.parse import quote
import cv2, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class CameraRtsp(Node):
    """Continuously drains RTSP and publishes only the newest decoded frame."""
    def __init__(self):
        super().__init__('camera_rtsp')
        defaults={'ip':'192.168.1.102','port':554,'username':'admin','password':'',
                  'path':'/Streaming/Channels/101','frame_id':'camera_optical_frame',
                  'backend':'gstreamer','rtsp_transport':'udp','camera_time_offset_ms':0.0}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        p=lambda k:self.get_parameter(k).value
        password=p('password') or os.environ.get('HIKVISION_PASSWORD','')
        if not password: raise RuntimeError('Camera password missing')
        auth=f"{quote(p('username'),safe='')}:{quote(password,safe='')}@"
        self.url=f"rtsp://{auth}{p('ip')}:{p('port')}{p('path')}"
        self.frame_id=p('frame_id'); self.backend=p('backend'); self.transport=p('rtsp_transport')
        self.offset_ns=int(float(p('camera_time_offset_ms'))*1e6)
        self.bridge=CvBridge(); self.pub=self.create_publisher(Image,'/camera/image_raw',qos_profile_sensor_data)
        self.stop=False; self.first=True
        threading.Thread(target=self.reader,daemon=True,name='rtsp-latest-frame').start()

    def open(self):
        if self.backend=='gstreamer':
            proto='udp' if self.transport=='udp' else 'tcp'
            pipeline=(f'rtspsrc location="{self.url}" protocols={proto} latency=0 drop-on-latency=true ! '
                      'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
                      'video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false')
            cap=cv2.VideoCapture(pipeline,cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.get_logger().info(f'RTSP low-latency GStreamer opened ({proto}, appsink drop=1)'); return cap
            self.get_logger().warning('GStreamer open failed; using FFmpeg low-latency fallback')
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS']=(f'rtsp_transport;{self.transport}|fflags;nobuffer|'
                                                     'flags;low_delay|max_delay;0|reorder_queue_size;0')
        cap=cv2.VideoCapture(self.url,cv2.CAP_FFMPEG); cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        return cap

    def reader(self):
        cap=None
        while not self.stop and rclpy.ok():
            if cap is None or not cap.isOpened():
                if cap: cap.release()
                cap=self.open()
                if not cap.isOpened(): time.sleep(1); continue
            ok,frame=cap.read(); receive_ns=self.get_clock().now().nanoseconds
            if not ok:
                self.get_logger().warning('RTSP read failed; reconnecting',throttle_duration_sec=5.0)
                cap.release(); cap=None; time.sleep(.05); continue
            msg=self.bridge.cv2_to_imgmsg(frame,'bgr8')
            stamp_ns=max(0,receive_ns-self.offset_ns); msg.header.stamp.sec=stamp_ns//1_000_000_000
            msg.header.stamp.nanosec=stamp_ns%1_000_000_000; msg.header.frame_id=self.frame_id
            self.pub.publish(msg)
            if self.first:
                self.get_logger().info(f'RTSP stream OK: {frame.shape[1]}x{frame.shape[0]}, offset={self.offset_ns/1e6:.1f}ms')
                self.first=False
        if cap: cap.release()

    def destroy_node(self): self.stop=True; super().destroy_node()

def main():
    rclpy.init(); node=CameraRtsp()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.try_shutdown()
