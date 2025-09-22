#!/usr/bin/env python3
# laser_tracker_meas_node.py
# ROS 2 Humble
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Float64MultiArray, Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.duration import Duration

def clamp(x, lo, hi): return max(lo, min(hi, x))

class LaserTrackerSim(Node):
    """
    레이저 트래커 측정 시뮬레이터(간단 버전)
    - tracker_frame -> target_frame(tf2)를 읽어 range/az/el 계산
    - 가우시안 노이즈 / FOV / max range / 드롭아웃 적용
    """
    def __init__(self):
        super().__init__('laser_tracker_sim')

        # ---- 파라미터 ----
        self.declare_parameter('tracker_frame', 'laser_tracker/base_link')
        self.declare_parameter('target_frame',  'smr')                # 엔드이펙터에 붙인 반사체 프레임
        self.declare_parameter('rate_hz',        100.0)               # 측정 주파수
        # 노이즈 (표준편차)
        self.declare_parameter('sigma_range_m',  0.00005)             # 0.05 mm
        self.declare_parameter('sigma_ang_deg',  0.006)               # 0.006 deg ~= 1e-4 rad
        # FOV, 거리 제한
        self.declare_parameter('fov_az_deg',     80.0)
        self.declare_parameter('fov_el_deg',     60.0)
        self.declare_parameter('max_range_m',    30.0)
        # 드롭아웃 확률 (가끔 측정 실패)
        self.declare_parameter('dropout_prob',   0.0)
        # 측정값을 TF로도 뿌릴지
        self.declare_parameter('publish_tf',     True)

        self.tracker_frame = self.get_parameter('tracker_frame').get_parameter_value().string_value
        self.target_frame  = self.get_parameter('target_frame').get_parameter_value().string_value
        self.rate_hz       = float(self.get_parameter('rate_hz').value)
        self.sigma_r       = float(self.get_parameter('sigma_range_m').value)
        self.sigma_a_rad   = math.radians(float(self.get_parameter('sigma_ang_deg').value))
        self.fov_az        = math.radians(float(self.get_parameter('fov_az_deg').value))
        self.fov_el        = math.radians(float(self.get_parameter('fov_el_deg').value))
        self.max_range     = float(self.get_parameter('max_range_m').value)
        self.dropout_prob  = float(self.get_parameter('dropout_prob').value)
        self.publish_tf    = bool(self.get_parameter('publish_tf').value)

        # ---- TF ----
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- QoS & Publishers ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.pub_point   = self.create_publisher(PointStamped,       'laser_tracker/point', qos)
        self.pub_rae     = self.create_publisher(Float64MultiArray,  'laser_tracker/rae', qos)
        self.pub_visible = self.create_publisher(Bool,               'laser_tracker/visible', qos)

        # ---- Timer ----
        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.01
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"[LaserTrackerSim] tracker_frame='{self.tracker_frame}', target_frame='{self.target_frame}', {self.rate_hz} Hz")

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.tracker_frame, self.target_frame, rclpy.time.Time()
            )
        except Exception as e:
            # 아직 TF가 준비 안됐을 때
            self.pub_visible.publish(Bool(data=False))
            return

        # tracker 좌표계에서의 타겟 위치
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z

        # 구면좌표 (range, az, el)
        r  = math.sqrt(x*x + y*y + z*z)
        az = math.atan2(y, x)                # +y가 좌, +x가 전방 가정
        el = math.atan2(z, math.hypot(x, y)) # 수평면 기준 고각

        # FOV & 거리 제한
        in_range = r <= self.max_range
        in_fov   = (abs(az) <= self.fov_az) and (abs(el) <= self.fov_el)
        visible  = in_range and in_fov and (random.random() > self.dropout_prob)

        if not visible:
            self.pub_visible.publish(Bool(data=False))
            return

        # 노이즈 적용
        r_m  = r  + random.gauss(0.0, self.sigma_r)
        az_m = az + random.gauss(0.0, self.sigma_a_rad)
        el_m = el + random.gauss(0.0, self.sigma_a_rad)

        # 측정 좌표 (트래커 좌표계)
        x_m = r_m * math.cos(el_m) * math.cos(az_m)
        y_m = r_m * math.cos(el_m) * math.sin(az_m)
        z_m = r_m * math.sin(el_m)

        # 발행: Point
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.tracker_frame
        ps.point.x, ps.point.y, ps.point.z = x_m, y_m, z_m
        self.pub_point.publish(ps)

        # 발행: RAE
        rae = Float64MultiArray()
        rae.data = [r_m, az_m, el_m]
        self.pub_rae.publish(rae)

        # 발행: Visible
        self.pub_visible.publish(Bool(data=True))

        # (옵션) TF: smr_measured (자세는 빔 방향을 z축으로 놔도 되지만 여기선 단위쿼터니언)
        if self.publish_tf:
            tfm = TransformStamped()
            tfm.header.stamp = ps.header.stamp
            tfm.header.frame_id = self.tracker_frame
            tfm.child_frame_id  = 'smr_measured'
            tfm.transform.translation.x = x_m
            tfm.transform.translation.y = y_m
            tfm.transform.translation.z = z_m
            tfm.transform.rotation.w = 1.0
            tfm.transform.rotation.x = 0.0
            tfm.transform.rotation.y = 0.0
            tfm.transform.rotation.z = 0.0
            self.tf_broadcaster.sendTransform(tfm)

def main():
    rclpy.init()
    node = LaserTrackerSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
