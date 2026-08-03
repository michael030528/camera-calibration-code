import copy, json, os, time, threading
from collections import deque
from pathlib import Path
import cv2, numpy as np, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker

LATEST_SENSOR_QOS=QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)

def cloud_array(msg):
    names = [f.name for f in msg.fields]
    intensity = next((n for n in ('intensity','reflectivity') if n in names), None)
    has_time='time_offset_ns' in names
    fields = ['x','y','z'] + ([intensity] if intensity else []) + (['time_offset_ns'] if has_time else [])
    rows = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
    if len(rows)==0: return np.empty((0,4), np.float32)
    # ROS 2 Jazzy returns a structured ndarray instead of a list of tuples.
    if getattr(rows.dtype,'names',None):
        xyz=np.column_stack((rows['x'],rows['y'],rows['z'])).astype(np.float32,copy=False)
        refl=np.asarray(rows[intensity],np.float32) if intensity else np.zeros(len(rows),np.float32)
        base=np.column_stack((xyz,refl))
        return np.column_stack((base,np.asarray(rows['time_offset_ns'],np.int64))) if has_time else base
    a=np.asarray(list(rows),dtype=np.float32)
    base=np.column_stack((a[:,:3],a[:,3] if a.shape[1]>3 else np.zeros(len(a),np.float32)))
    return np.column_stack((base,a[:,4])) if has_time else base

def cloud_source_ids(msg):
    if not any(f.name == 'accumulation_frame' for f in msg.fields):
        return np.zeros(msg.width * msg.height, np.uint32)
    rows = point_cloud2.read_points(
        msg, field_names=['accumulation_frame'], skip_nans=True)
    if getattr(rows.dtype, 'names', None):
        return np.asarray(rows['accumulation_frame'], np.uint32)
    return np.asarray(list(rows), np.uint32).reshape(-1)

def accumulated_cloud(reference, frames):
    """Combine fixed-frame clouds and retain each point's source-frame id."""
    arrays=[]
    for frame_index,msg in enumerate(frames):
        points=cloud_array(msg)
        if len(points): arrays.append((points,frame_index))
    if not arrays: return None
    total=sum(len(points) for points,_ in arrays)
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),
                    ('intensity','<f4'),('accumulation_frame','<u4')])
    packed=np.empty(total,dtype=dtype); offset=0
    for points,frame_index in arrays:
        count=len(points); block=packed[offset:offset+count]
        block['x']=points[:,0]; block['y']=points[:,1]; block['z']=points[:,2]
        block['intensity']=points[:,3] if points.shape[1]>3 else 0.0
        block['accumulation_frame']=frame_index; offset+=count
    fields=[
        PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
        PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
        PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
        PointField(name='intensity',offset=12,datatype=PointField.FLOAT32,count=1),
        PointField(name='accumulation_frame',offset=16,datatype=PointField.UINT32,count=1)]
    # create_cloud converts the NumPy buffer to the array.array representation
    # expected by ROS 2 Jazzy. A raw bytes assignment can be silently ignored
    # by some PointCloud2 consumers, notably RViz.
    out=point_cloud2.create_cloud(copy.deepcopy(reference.header),fields,packed)
    out.is_dense=True
    return out

def fit_plane(points, threshold, rounds=80, min_inliers=50):
    if len(points) < min_inliers: return None
    rng, best = np.random.default_rng(7), np.empty(0, int)
    for _ in range(rounds):
        q = points[rng.choice(len(points), 3, replace=False), :3]
        n = np.cross(q[1]-q[0], q[2]-q[0]); norm = np.linalg.norm(n)
        if norm < 1e-7: continue
        n /= norm; d = -n.dot(q[0]); idx = np.flatnonzero(np.abs(points[:,:3]@n+d) < threshold)
        if len(idx) > len(best): best = idx
    if len(best) < min_inliers: return None
    xyz = points[best,:3]; center = xyz.mean(0)
    _,_,vh = np.linalg.svd(xyz-center, full_matrices=False); n = vh[-1]
    # Keep the normal pointing from the LiDAR origin towards the board.  This
    # removes the otherwise arbitrary per-frame plane-normal sign.
    if n.dot(center) < 0: n = -n
    return n, -n.dot(center), best

class CaptureGui(Node):
    def __init__(self):
        super().__init__('capture_gui')
        defaults={'image_topic':'/camera/image_raw','cloud_topic':'/livox/lidar','board_cols':11,'board_rows':8,
                  'square_size':0.06,'board_outer_width':0.80,'board_outer_height':0.60,
                  'sync_slop':0.05,'selection_cloud_topic':'/calibration/selection_cloud',
                  'selection_freeze_topic':'/calibration/freeze_selection',
                  'selected_points_topic':'/calibration/selected_points',
                  'output_dir':'calibration_samples',
                  'roi_x':[0.5,8.0],'roi_y':[-4.0,4.0],'roi_z':[-3.0,3.0],
                  'reflectivity_min':0.0,'plane_threshold':0.025,
                  'detection_scale':0.65,'detection_hz':12.0,
                  'auto_save':True,'auto_save_stable_frames':3,'auto_save_min_interval':1.5,
                  'auto_save_max_samples':30,'min_lidar_inliers':80,
                  'accumulation_frames':5,'accumulation_min_frames':3,
                  'accumulation_max_span':0.55,'motion_min_points_per_frame':8,
                  'motion_max_normal_deg':4.0,'motion_max_plane_offset':0.04,
                  'motion_max_centroid_shift':0.15}
        defaults.update({'manual_selection_enabled':True,'selection_margin':0.02,
                         'selection_thickness':0.08,'selection_size_tolerance':0.25,
                         'manual_min_points':30})
        for k,v in defaults.items(): self.declare_parameter(k,v)
        p=lambda k:self.get_parameter(k).value
        self.cols,self.rows,self.square=int(p('board_cols')),int(p('board_rows')),float(p('square_size'))
        self.board_width=float(p('board_outer_width')); self.board_height=float(p('board_outer_height'))
        self.roi=[p('roi_x'),p('roi_y'),p('roi_z')]; self.refl=float(p('reflectivity_min'))
        self.threshold=float(p('plane_threshold')); self.out=Path(os.path.expanduser(p('output_dir'))).resolve()
        self.sync_slop=float(p('sync_slop'))
        self.detection_scale=float(p('detection_scale')); self.detection_period=1.0/float(p('detection_hz'))
        self.auto_save=bool(p('auto_save')); self.auto_stable_required=int(p('auto_save_stable_frames'))
        self.auto_min_interval=float(p('auto_save_min_interval')); self.auto_max=int(p('auto_save_max_samples'))
        self.min_lidar_inliers=int(p('min_lidar_inliers')); self.auto_good=0; self.auto_saved=0
        self.accumulation_frames=max(1,int(p('accumulation_frames')))
        self.accumulation_min_frames=max(1,min(self.accumulation_frames,int(p('accumulation_min_frames'))))
        self.accumulation_max_span=float(p('accumulation_max_span'))
        self.motion_min_points=max(3,int(p('motion_min_points_per_frame')))
        self.motion_max_normal=float(p('motion_max_normal_deg'))
        self.motion_max_offset=float(p('motion_max_plane_offset'))
        self.motion_max_centroid=float(p('motion_max_centroid_shift'))
        self.manual_selection=bool(p('manual_selection_enabled')); self.selection_margin=float(p('selection_margin'))
        self.selection_thickness=float(p('selection_thickness')); self.manual_min_points=int(p('manual_min_points'))
        self.selection_size_tolerance=float(p('selection_size_tolerance'))
        self.clicked_corners=[]
        self.image_buffer=deque(maxlen=8); self.cloud_buffer=deque(maxlen=max(8,self.accumulation_frames+2))
        self.display_pair=None; self.frozen_pair=None; self.last_preview_pair_key=None
        self.display_accumulation_meta=None; self.frozen_accumulation_meta=None
        self.frozen_camera_view=None; self.frozen_mode=None
        self.last_auto_time=0.0; self.last_signature=None
        self.out.mkdir(parents=True,exist_ok=True); self.bridge=CvBridge(); self.latest=None; self.seq=0
        self.last_sync_time = 0.0
        cv2.namedWindow('camera chessboard', cv2.WINDOW_NORMAL)
        self.raw_camera=np.zeros((540,960,3),np.uint8); self.detected_camera=None; self.detected_at=0.0
        self.display_camera=self.raw_camera
        self.pending=None; self.pending_lock=threading.Lock(); self.running=True
        self.camera_pending=None; self.camera_lock=threading.Lock(); self.tracked_corners=None
        self.latest_camera_detection=None; self.detect_failures=0
        self.detect_count=0; self.detect_hits=0; self.detect_report_at=time.monotonic()+5
        self.latest_image_msg=None
        self.latest_cloud_msg=None; self.selection_meta=None
        self.accumulation_publish_count=0
        self.preview_sub=self.create_subscription(Image,p('image_topic'),self.preview_cb,LATEST_SENSOR_QOS)
        self.cloud_preview_sub=self.create_subscription(PointCloud2,p('cloud_topic'),self.cloud_preview_cb,LATEST_SENSOR_QOS)
        self.clicked_sub=self.create_subscription(PointStamped,'/clicked_point',self.clicked_cb,10)
        self.freeze_sub=self.create_subscription(
            Empty,p('selection_freeze_topic'),self.freeze_request_cb,10)
        self.selected_points_sub=self.create_subscription(
            PointCloud2,p('selected_points_topic'),self.selected_points_cb,qos_profile_sensor_data)
        self.marker_pub=self.create_publisher(Marker,'/calibration/selection_box',10)
        self.selection_cloud_pub=self.create_publisher(
            PointCloud2,p('selection_cloud_topic'),qos_profile_sensor_data)
        threading.Thread(target=self.worker,daemon=True,name='calibration-detection').start()
        threading.Thread(target=self.camera_detector,daemon=True,name='camera-chessboard-tracker').start()
        self.create_timer(1.0/30.0,self.display_tick)
        self.get_logger().info(
            f'GUI: RViz shows a {self.accumulation_frames}-frame motion-checked cloud; '
            'drag around board points. C cancels, Q quits.')

    def preview_cb(self, msg):
        # Never run detection here: display the newest frame immediately.
        self.latest_image_msg=msg
        self.image_buffer.append(msg)
        image=self.bridge.imgmsg_to_cv2(msg,'bgr8'); self.raw_camera=image
        with self.camera_lock: self.camera_pending=(msg,image)
        self.update_synchronized_preview()

    def detect_board(self,image,allow_exhaustive=False):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY); board=(self.cols,self.rows)
        # Once locked, search a padded local ROI at full resolution. This is
        # both faster and more stable than rescanning the entire 1080p frame.
        if self.tracked_corners is not None:
            xy=self.tracked_corners.reshape(-1,2); x,y,w,h=cv2.boundingRect(xy.astype(np.float32))
            pad=max(50,int(max(w,h)*.35)); x0=max(0,x-pad); y0=max(0,y-pad)
            x1=min(gray.shape[1],x+w+pad); y1=min(gray.shape[0],y+h+pad)
            crop=gray[y0:y1,x0:x1]
            ok,c=cv2.findChessboardCorners(crop,board,cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ok: cv2.cornerSubPix(crop,c,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,20,.03))
            if ok: c[:,:,0]+=x0; c[:,:,1]+=y0; return True,c
        scale=self.detection_scale
        small=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
        classic_flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK
        ok,c=cv2.findChessboardCorners(small,board,classic_flags)
        if ok: cv2.cornerSubPix(small,c,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,20,.03))
        if not ok and allow_exhaustive:
            enhanced=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8)).apply(small)
            ok,c=cv2.findChessboardCornersSB(enhanced,board,cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE)
        if ok: c/=scale
        return ok,c

    def camera_detector(self):
        while self.running:
            started=time.monotonic()
            with self.camera_lock: item,self.camera_pending=self.camera_pending,None
            if item is None: time.sleep(.01); continue
            msg,image=item
            deep_search=self.detect_failures>0 and self.detect_failures%8==0
            found,corners=self.detect_board(image,deep_search)
            if found:
                self.tracked_corners=corners.copy(); self.detect_failures=0
                self.latest_camera_detection=(msg.header.stamp,corners.copy())
            else:
                self.detect_failures+=1
                if self.detect_failures>=4: self.tracked_corners=None
            self.detect_count+=1; self.detect_hits+=int(found)
            if time.monotonic()>=self.detect_report_at:
                self.get_logger().info(f'Chessboard detector: {self.detect_count/5:.1f} Hz, hits={self.detect_hits}/{self.detect_count}')
                self.detect_count=0; self.detect_hits=0; self.detect_report_at=time.monotonic()+5
            time.sleep(max(0.0,self.detection_period-(time.monotonic()-started)))

    def cloud_preview_cb(self,msg):
        # Match every cloud to the nearest buffered camera timestamp. RViz
        # displays this matched cloud topic, not the raw live topic. Once the
        # first corner is clicked, publication stops and the exact pair stays
        # frozen until the RViz selection has been processed.
        self.latest_cloud_msg=msg
        self.cloud_buffer.append(msg)
        self.update_synchronized_preview()

    def update_synchronized_preview(self):
        if self.frozen_pair is not None or not self.image_buffer or not self.cloud_buffer: return
        # Prefer the newest cloud, then its closest image. Calling this from
        # both subscriptions allows a slightly later image to replace a less
        # accurate earlier match for the same cloud.
        cmsg=max(self.cloud_buffer,key=self.msg_time); cts=self.msg_time(cmsg)
        imsg=min(self.image_buffer,key=lambda item:abs(self.msg_time(item)-cts))
        dt=abs(self.msg_time(imsg)-cts)
        if dt>self.sync_slop: return
        key=(cmsg.header.stamp.sec,cmsg.header.stamp.nanosec,
             imsg.header.stamp.sec,imsg.header.stamp.nanosec)
        if key==self.last_preview_pair_key: return
        frames=sorted((item for item in self.cloud_buffer
                       if 0.0<=cts-self.msg_time(item)<=self.accumulation_max_span),
                      key=self.msg_time)[-self.accumulation_frames:]
        combined=accumulated_cloud(cmsg,frames)
        if combined is None: return
        self.last_preview_pair_key=key
        span=self.msg_time(frames[-1])-self.msg_time(frames[0]) if len(frames)>1 else 0.0
        self.display_accumulation_meta={'accumulation_frames':len(frames),
                                        'accumulation_span_ms':span*1000.0,
                                        'accumulated_points':int(combined.width)}
        self.display_pair=(imsg,combined)
        self.selection_cloud_pub.publish(combined)
        self.accumulation_publish_count+=1
        if self.accumulation_publish_count in (1,10):
            self.get_logger().info(
                f'Accumulated cloud published: {len(frames)} frames, '
                f'{combined.width} points, span={span*1000:.1f} ms')
        # Manual RViz selection performs detection and plane fitting only on
        # the frozen pair. Avoid duplicate full-frame work during live preview.
        if self.auto_save: self.cb(imsg,cmsg)

    def clicked_cb(self,click):
        self.get_logger().info(f'RViz click received: ({click.point.x:.2f}, {click.point.y:.2f}, {click.point.z:.2f})')
        if not self.manual_selection: return
        if self.frozen_pair is None and not self.freeze_displayed_pair('corners'): return
        imsg,cmsg,image,camera_corners,dt_ms=self.frozen_pair
        self.clicked_corners.append(np.array([click.point.x,click.point.y,click.point.z],float))
        self.publish_corner_markers(cmsg.header.frame_id)
        count=len(self.clicked_corners)
        self.update_frozen_camera_view(image,camera_corners,dt_ms,count)
        self.get_logger().info(f'Board corner {count}/4 accepted')
        if count<4: return
        corners3d=np.asarray(self.clicked_corners); self.clicked_corners=[]
        try:
            self.process_frozen_selection(corners3d,imsg,cmsg,image,camera_corners,dt_ms)
        finally:
            self.release_frozen_pair()

    @staticmethod
    def msg_time(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def freeze_request_cb(self,_msg):
        if self.frozen_pair is None:
            self.freeze_displayed_pair('select')

    def freeze_displayed_pair(self,mode='select'):
        if self.display_pair is None:
            self.get_logger().warning('No synchronized image/cloud pair is ready yet')
            return False
        imsg,cmsg=self.display_pair
        meta=self.display_accumulation_meta or {}
        if int(meta.get('accumulation_frames',1))<self.accumulation_min_frames:
            self.get_logger().warning(
                f'Cannot freeze: only {meta.get("accumulation_frames",1)} LiDAR frames are buffered; '
                f'need {self.accumulation_min_frames}')
            return False
        dt_ms=(self.msg_time(imsg)-self.msg_time(cmsg))*1000
        if abs(dt_ms)>self.sync_slop*1000:
            self.get_logger().warning(f'Cannot freeze: image/cloud delta {dt_ms:.1f} ms exceeds limit')
            return False
        image=self.bridge.imgmsg_to_cv2(imsg,'bgr8')
        found,corners=self.detect_board(image,True)
        if not found:
            self.get_logger().warning('Cannot freeze: camera chessboard is not detected in the matched frame')
            return False
        self.frozen_pair=(imsg,cmsg,image,corners,dt_ms); self.frozen_mode=mode
        self.frozen_accumulation_meta=dict(meta)
        self.selection_cloud_pub.publish(cmsg)
        self.update_frozen_camera_view(image,corners,dt_ms,0)
        self.get_logger().info(
            f'Frozen synchronized frame: image={self.msg_time(imsg):.9f}, '
            f'cloud={self.msg_time(cmsg):.9f}, dt={dt_ms:+.1f} ms, '
            f'accumulated={meta.get("accumulation_frames",1)} frames / '
            f'{meta.get("accumulated_points",cmsg.width)} points')
        return True

    def update_frozen_camera_view(self,image,corners,dt_ms,count):
        view=image.copy(); cv2.drawChessboardCorners(view,(self.cols,self.rows),corners,True)
        cv2.rectangle(view,(0,0),(view.shape[1],82),(20,20,20),-1)
        count_frames=(self.frozen_accumulation_meta or {}).get('accumulation_frames',1)
        cv2.putText(view,f'FROZEN SYNC FRAME  dt={dt_ms:+.1f} ms  LiDAR x{count_frames}',
                    (15,31),0,.78,(0,255,255),2)
        status=('RViz Select: drag around chessboard points' if self.frozen_mode=='select' else
                f'RViz corners: {count}/4')
        cv2.putText(view,f'{status}   C: cancel',(15,66),0,.68,(255,255,255),2)
        self.frozen_camera_view=view

    def release_frozen_pair(self):
        if self.frozen_pair is not None:
            frame_id=self.frozen_pair[1].header.frame_id or 'livox_frame'
            marker=Marker(); marker.header.frame_id=frame_id; marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='calibration'; marker.id=0; marker.action=Marker.DELETE
            self.marker_pub.publish(marker)
        self.frozen_pair=None; self.frozen_camera_view=None; self.frozen_mode=None; self.clicked_corners=[]
        self.frozen_accumulation_meta=None
        self.display_pair=None; self.last_preview_pair_key=None

    def check_motion_consistency(self,selected,source_ids):
        frame_models=[]
        for frame_id in np.unique(source_ids):
            group=selected[source_ids==frame_id]
            if len(group)<self.motion_min_points: continue
            plane=fit_plane(group,self.threshold*1.25,rounds=100,
                            min_inliers=self.motion_min_points)
            if plane is None: continue
            inlier_points=group[plane[2],:3]
            frame_models.append([int(frame_id),plane[0].copy(),float(plane[1]),
                                 inlier_points.mean(0),len(group),len(inlier_points)])
        if len(frame_models)<self.accumulation_min_frames:
            return False,{'motion_consistent':False,'motion_checked_frames':len(frame_models),
                          'motion_reject_reason':'too_few_per_frame_plane_fits'}
        reference=max(frame_models,key=lambda model:model[5])
        ref_n=reference[1]
        for model in frame_models:
            if np.dot(model[1],ref_n)<0: model[1]*=-1; model[2]*=-1
        normals=np.asarray([model[1] for model in frame_models])
        offsets=np.asarray([model[2] for model in frame_models])
        centers=np.asarray([model[3] for model in frame_models])
        mean_n=normals.mean(0); mean_n/=np.linalg.norm(mean_n)
        normal_angles=np.degrees(np.arccos(np.clip(normals@mean_n,-1,1)))
        offset_spread=float(np.max(offsets)-np.min(offsets))
        median_center=np.median(centers,axis=0)
        center_shift=float(np.max(np.linalg.norm(centers-median_center,axis=1)))
        max_angle=float(np.max(normal_angles))
        consistent=(max_angle<=self.motion_max_normal and
                    offset_spread<=self.motion_max_offset and
                    center_shift<=self.motion_max_centroid)
        meta={'motion_consistent':bool(consistent),'motion_checked_frames':len(frame_models),
              'motion_max_normal_deg':max_angle,'motion_plane_offset_spread_m':offset_spread,
              'motion_max_centroid_shift_m':center_shift,
              'motion_points_per_frame':[model[4] for model in frame_models],
              'motion_inliers_per_frame':[model[5] for model in frame_models]}
        if not consistent: meta['motion_reject_reason']='board_moved_during_lidar_accumulation'
        return consistent,meta

    def selected_points_cb(self,msg):
        if self.frozen_pair is None:
            self.get_logger().warning('RViz selection ignored: no synchronized frame is frozen')
            return
        imsg,cmsg,image,camera_corners,dt_ms=self.frozen_pair
        try:
            same_stamp=(msg.header.stamp.sec==cmsg.header.stamp.sec and
                        msg.header.stamp.nanosec==cmsg.header.stamp.nanosec)
            if not same_stamp:
                self.get_logger().warning('RViz selection ignored: selected points are not from the frozen cloud')
                return
            selected=cloud_array(msg)
            if len(selected)<self.manual_min_points:
                self.get_logger().warning(
                    f'RViz selection has only {len(selected)} points; need {self.manual_min_points}')
                return
            consistent,motion_meta=self.check_motion_consistency(selected,cloud_source_ids(msg))
            if not consistent:
                self.get_logger().warning(
                    'Not saved: accumulated board points moved between frames '
                    f'(normal={motion_meta.get("motion_max_normal_deg",float("nan")):.2f} deg, '
                    f'offset={motion_meta.get("motion_plane_offset_spread_m",float("nan"))*100:.1f} cm, '
                    f'center={motion_meta.get("motion_max_centroid_shift_m",float("nan"))*100:.1f} cm)')
                return
            plane=fit_plane(selected,self.threshold,min_inliers=self.manual_min_points)
            if plane is None or len(plane[2])<self.manual_min_points:
                self.get_logger().warning('No stable chessboard plane in RViz selected points')
                return
            pts=cloud_array(cmsg)
            self.latest=(image,pts,True,camera_corners,plane,imsg,cmsg)
            self.selection_meta={'selection_method':'rviz_rectangle_select_accumulated',
                                 'frozen_synchronized_frame':True,
                                 'selected_points':int(len(selected)),
                                 'plane_inliers':int(len(plane[2]))}
            self.selection_meta.update(self.frozen_accumulation_meta or {})
            self.selection_meta.update(motion_meta)
            self.save(auto=False,capture_mode='rviz_select_frozen_sync_accumulated')
        finally:
            self.release_frozen_pair()

    def cancel_frozen_selection(self):
        if self.frozen_pair is None: return
        self.release_frozen_pair(); self.get_logger().info('Frozen selection cancelled; live synchronized preview resumed')

    def process_frozen_selection(self,corners3d,imsg,cmsg,image,camera_corners,dt_ms):
        try: center,u,v,n,clicked_width,clicked_height=self.oriented_board_frame(corners3d)
        except ValueError as exc:
            self.get_logger().warning(str(exc)); return
        width_error=abs(clicked_width-self.board_width)/self.board_width
        height_error=abs(clicked_height-self.board_height)/self.board_height
        if width_error>self.selection_size_tolerance or height_error>self.selection_size_tolerance:
            self.get_logger().warning(
                f'Corner size {clicked_width:.2f} x {clicked_height:.2f} m does not match '
                f'board {self.board_width:.2f} x {self.board_height:.2f} m; '
                'continuing with the known physical board size')
        # The clicks determine the board center and orientation. Use the known
        # physical outer size for the prism so sparse edge returns cannot make
        # the fitted region randomly grow or shrink.
        width,height=self.board_width,self.board_height
        pts=cloud_array(cmsg)
        rel=pts[:,:3]-center; lu=rel@u; lv=rel@v; ln=rel@n
        mask=((np.abs(lu)<=width/2+self.selection_margin)&
              (np.abs(lv)<=height/2+self.selection_margin)&
              (np.abs(ln)<=self.selection_thickness/2))
        local=pts[mask]
        self.publish_oriented_box(center,u,v,n,width,height,cmsg.header.frame_id)
        if len(local)<self.manual_min_points:
            self.get_logger().warning(f'Oriented board box has only {len(local)} points; need {self.manual_min_points}'); return
        plane=fit_plane(local,self.threshold,min_inliers=self.manual_min_points)
        if plane is None or len(plane[2])<self.manual_min_points:
            self.get_logger().warning('No stable board plane in selected box'); return
        if abs(dt_ms)>self.sync_slop*1000:
            self.get_logger().warning(f'Not saved: image/cloud delta {dt_ms:.1f} ms exceeds limit'); return
        self.latest=(image,pts,True,camera_corners,plane,imsg,cmsg)
        self.selection_meta={'selection_corners_3d':corners3d.tolist(),'selection_center':center.tolist(),
                             'selection_width':float(width),'selection_height':float(height),
                             'clicked_width':float(clicked_width),'clicked_height':float(clicked_height),
                             'selection_thickness':self.selection_thickness,
                             'frozen_synchronized_frame':True,
                             'selected_points':int(len(local)),'plane_inliers':int(len(plane[2]))}
        self.save(auto=False,capture_mode='rviz_four_corner_frozen_sync_frame')

    def oriented_board_frame(self,p):
        # Click order: top-left, top-right, bottom-right, bottom-left.
        uvec=((p[1]-p[0])+(p[2]-p[3]))*.5; vraw=((p[3]-p[0])+(p[2]-p[1]))*.5
        width=np.linalg.norm(uvec)
        if width<.05: raise ValueError('Horizontal corner span is too small; repeat four clicks')
        u=uvec/width; vraw=vraw-u*np.dot(vraw,u); height=np.linalg.norm(vraw)
        if height<.05: raise ValueError('Vertical corner span is too small; repeat four clicks')
        v=vraw/height; n=np.cross(u,v); n/=np.linalg.norm(n); center=p.mean(0)
        return center,u,v,n,width,height

    @staticmethod
    def matrix_to_quaternion(R):
        # R columns are the oriented box x/y/z axes.
        q=np.empty(4); trace=np.trace(R)
        if trace>0:
            s=np.sqrt(trace+1.0)*2; q[3]=.25*s; q[0]=(R[2,1]-R[1,2])/s; q[1]=(R[0,2]-R[2,0])/s; q[2]=(R[1,0]-R[0,1])/s
        else:
            i=int(np.argmax(np.diag(R))); j=(i+1)%3; k=(i+2)%3; s=np.sqrt(1+R[i,i]-R[j,j]-R[k,k])*2
            q[i]=.25*s; q[3]=(R[k,j]-R[j,k])/s; q[j]=(R[j,i]+R[i,j])/s; q[k]=(R[k,i]+R[i,k])/s
        return q/np.linalg.norm(q)

    def publish_corner_markers(self,frame_id):
        m=Marker(); m.header.frame_id=frame_id or 'livox_frame'; m.header.stamp=self.get_clock().now().to_msg()
        m.ns='calibration'; m.id=0; m.type=Marker.SPHERE_LIST; m.action=Marker.ADD
        m.scale.x=m.scale.y=m.scale.z=.055; m.color.r=1.0; m.color.g=.3; m.color.b=.05; m.color.a=1.0
        for p in self.clicked_corners:
            q=Point(); q.x=float(p[0]); q.y=float(p[1]); q.z=float(p[2]); m.points.append(q)
        self.marker_pub.publish(m)

    def publish_oriented_box(self,center,u,v,n,width,height,frame_id):
        corners=Marker(); corners.header.frame_id=frame_id or 'livox_frame'; corners.header.stamp=self.get_clock().now().to_msg()
        corners.ns='calibration'; corners.id=0; corners.action=Marker.DELETE
        self.marker_pub.publish(corners)
        m=Marker(); m.header.frame_id=frame_id or 'livox_frame'; m.header.stamp=self.get_clock().now().to_msg()
        m.ns='calibration'; m.id=1; m.type=Marker.CUBE; m.action=Marker.ADD
        m.pose.position.x=float(center[0]); m.pose.position.y=float(center[1]); m.pose.position.z=float(center[2])
        q=self.matrix_to_quaternion(np.column_stack((u,v,n)))
        m.pose.orientation.x=float(q[0]); m.pose.orientation.y=float(q[1]); m.pose.orientation.z=float(q[2]); m.pose.orientation.w=float(q[3])
        m.scale.x=float(width+2*self.selection_margin); m.scale.y=float(height+2*self.selection_margin); m.scale.z=self.selection_thickness
        m.color.r=0.1; m.color.g=1.0; m.color.b=0.1; m.color.a=0.22; m.lifetime.sec=3
        self.marker_pub.publish(m)

    def display_tick(self):
        # Live view must never wait for chessboard detection. Detection runs in
        # its own latest-frame worker; only a deliberate RViz selection freezes
        # and annotates the exact synchronized camera frame.
        self.display_camera=(self.frozen_camera_view if self.frozen_camera_view is not None
                             else self.raw_camera)
        cv2.imshow('camera chessboard',self.display_camera)
        key=cv2.waitKey(1)&255
        if key in (ord('c'),ord('C')): self.cancel_frozen_selection()
        elif key in (ord('q'),27): rclpy.shutdown()

    def cb(self, imsg, cmsg):
        self.last_sync_time=time.monotonic()
        with self.pending_lock: self.pending=(imsg,cmsg)  # overwrite stale work

    def worker(self):
        while self.running:
            with self.pending_lock: pair,self.pending=self.pending,None
            if pair is None: time.sleep(.01); continue
            self.process_pair(*pair)

    def process_pair(self, imsg, cmsg):
        image=self.bridge.imgmsg_to_cv2(imsg,'bgr8')
        found,corners=self.detect_board(image,True)
        view=image.copy()
        if found: cv2.drawChessboardCorners(view,(self.cols,self.rows),corners,True)
        pts=cloud_array(cmsg); mask=np.ones(len(pts),bool)
        for axis,bounds in enumerate(self.roi): mask &= (pts[:,axis]>=bounds[0]) & (pts[:,axis]<=bounds[1])
        mask &= pts[:,3]>=self.refl; roi=pts[mask]; plane=fit_plane(roi,self.threshold)
        cv2.putText(view,f'chessboard: {found}',(15,30),0,0.8,(0,255,0) if found else (0,0,255),2)
        dt_ms=((imsg.header.stamp.sec+imsg.header.stamp.nanosec*1e-9)-
               (cmsg.header.stamp.sec+cmsg.header.stamp.nanosec*1e-9))*1000
        cv2.putText(view,f'sync dt: {dt_ms:+.1f} ms',(15,62),0,0.7,(255,255,0),2)
        cv2.putText(view,f'AUTO: {self.auto_saved}/{self.auto_max}  stable {self.auto_good}/{self.auto_stable_required}',
                    (15,94),0,0.65,(255,220,100),2)
        self.display_camera=view
        self.latest=(image,pts,found,corners,plane,imsg,cmsg)
        self.maybe_auto_save(image,found,corners,plane,dt_ms)

    def sample_signature(self,image,corners,plane):
        xy=corners.reshape(-1,2); center=xy.mean(0)/np.array([image.shape[1],image.shape[0]])
        x,y,w,h=cv2.boundingRect(xy.astype(np.float32)); area=(w*h)/(image.shape[0]*image.shape[1])
        return np.r_[center,np.log(max(area,1e-8)),plane[0],plane[1]]

    def is_diverse(self,sig):
        if self.last_signature is None: return True
        old=self.last_signature
        center_move=np.linalg.norm(sig[:2]-old[:2])
        scale_change=abs(sig[2]-old[2])
        angle=np.degrees(np.arccos(np.clip(np.dot(sig[3:6],old[3:6]),-1,1)))
        distance=abs(sig[6]-old[6])
        return center_move>0.035 or scale_change>0.08 or angle>3.0 or distance>0.08

    def maybe_auto_save(self,image,found,corners,plane,dt_ms):
        inliers=len(plane[2]) if plane is not None else 0
        good=found and plane is not None and inliers>=self.min_lidar_inliers and abs(dt_ms)<=self.sync_slop*1000
        self.auto_good=self.auto_good+1 if good else 0
        if not self.auto_save or not good or self.auto_good<self.auto_stable_required or self.auto_saved>=self.auto_max: return
        now=time.monotonic()
        if now-self.last_auto_time<self.auto_min_interval: return
        sig=self.sample_signature(image,corners,plane)
        if not self.is_diverse(sig): return
        self.save(auto=True); self.last_signature=sig; self.last_auto_time=now; self.auto_saved+=1; self.auto_good=0

    def save(self,auto=False,capture_mode=None):
        if not self.latest: return
        image,pts,found,corners,plane,imsg,cmsg=self.latest
        if not found or plane is None:
            self.get_logger().warning('Not saved: both camera corners and LiDAR plane are required'); return
        stamp=f'{imsg.header.stamp.sec}_{imsg.header.stamp.nanosec:09d}'; base=self.out/stamp
        cv2.imwrite(str(base)+'.png',image); np.savez_compressed(str(base)+'.npz',points=pts)
        data={'stamp':stamp,'image_stamp':imsg.header.stamp.sec+imsg.header.stamp.nanosec*1e-9,
              'cloud_stamp':cmsg.header.stamp.sec+cmsg.header.stamp.nanosec*1e-9,
              'timestamp_delta_ms':(imsg.header.stamp.sec+imsg.header.stamp.nanosec*1e-9-
                                    cmsg.header.stamp.sec-cmsg.header.stamp.nanosec*1e-9)*1000,
              'lidar_time_offset_min_ns':int(np.min(pts[:,4])) if pts.shape[1]>4 else None,
              'lidar_time_offset_max_ns':int(np.max(pts[:,4])) if pts.shape[1]>4 else None,
              'image_points':corners.reshape(-1,2).tolist(),'lidar_plane_n':plane[0].tolist(),
              'lidar_plane_d':float(plane[1]),'lidar_inliers':int(len(plane[2])),
              'board_cols':self.cols,'board_rows':self.rows,'square_size':self.square}
        data['capture_mode']=capture_mode or ('auto' if auto else 'manual')
        if self.selection_meta is not None: data.update(self.selection_meta); self.selection_meta=None
        (Path(str(base)+'.json')).write_text(json.dumps(data,indent=2),encoding='utf-8')
        mode='AUTO' if auto else 'MANUAL'
        self.get_logger().info(f'{mode} saved synchronized sample {stamp} (dt={abs(data["image_stamp"]-data["cloud_stamp"])*1000:.1f} ms)')

    def destroy_node(self): self.running=False; super().destroy_node()

def main():
    rclpy.init(); node=CaptureGui()
    try: rclpy.spin(node)
    finally: cv2.destroyAllWindows(); node.destroy_node(); rclpy.try_shutdown()
