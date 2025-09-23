#!/usr/bin/env python3
# auto_aim_node.py (ROS 2 Humble) - stable absolute aiming
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class AutoAim(Node):
    def __init__(self):
        super().__init__('laser_tracker_auto_aim')

        # 조준 계산은 pan 조인트 직전 프레임(베이스) 기준이 안정적
        self.declare_parameter('tracker_frame', 'laser_tracker/base_link')
        self.declare_parameter('target_frame',  'yuilrobotics/ee_smr')
        self.declare_parameter('rate_hz', 1000.0)

        # 한계각(절대 명령 클램프)
        self.declare_parameter('lim_pan_deg',  85.0)
        self.declare_parameter('lim_tilt_deg', 60.0)

        # 안정화 파라미터
        self.declare_parameter('alpha', 0.30)          # 지수평활 계수 (0.1~0.4 추천)
        self.declare_parameter('deadband_deg', 0.02)    # 미세오차 무시(지터 억제)
        self.declare_parameter('max_speed_rad_s', 6.0)         # 초당 최대 속도(예: 6 rad/s ≈ 343°/s)
        self.declare_parameter('accel_err_thresh_deg', 5.0)    # 오차 크면 가속 모드

        # 필요시 축/부호 보정
        self.declare_parameter('pan_sign',  1.0)       # -1로 바꾸면 좌/우 반전
        self.declare_parameter('tilt_sign', -1.0)       # -1로 바꾸면 위/아래 반전

        self.tfbuf = Buffer(cache_time=Duration(seconds=10.0))
        self.tfl  = TransformListener(self.tfbuf, self)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/laser_tracker/pan_tilt_controller/commands',
            10
        )

        hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / max(hz, 1.0), self.step)

        # 캐시
        self.trk = self.get_parameter('tracker_frame').value
        self.tgt = self.get_parameter('target_frame').value
        self.lim_pan  = math.radians(float(self.get_parameter('lim_pan_deg').value))
        self.lim_tilt = math.radians(float(self.get_parameter('lim_tilt_deg').value))
        self.alpha    = float(self.get_parameter('alpha').value)
        self.deadband = math.radians(float(self.get_parameter('deadband_deg').value))
        self.max_speed = float(self.get_parameter('max_speed_rad_s').value)
        self.err_thresh = math.radians(float(self.get_parameter('accel_err_thresh_deg').value))
        self.t_prev = self.get_clock().now()
        self.pan_sign = float(self.get_parameter('pan_sign').value)
        self.tilt_sign= float(self.get_parameter('tilt_sign').value)

        # 이전 명령(스무딩/램프용)
        self.pan_prev  = 0.0
        self.tilt_prev = 0.0

    def step(self):
        # 최신 TF 기준으로 타깃 벡터 획득 (time=0)
        try:
            tf: TransformStamped = self.tfbuf.lookup_transform(
                self.trk, self.tgt, rclpy.time.Time()
            )
        except Exception:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z

        # 베이스 프레임 기준 절대 조준각 (yaw=pan, pitch=tilt)
        pan_des  = math.atan2(x, -y) * self.pan_sign
        tilt_des = math.atan2(z, math.hypot(x, y)) * self.tilt_sign

        # dt 계산 (초당 제한에 사용)
        t_now = self.get_clock().now()
        dt = (t_now - self.t_prev).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.2:  # 타이머 점프 보호
            dt = 1.0 / max(float(self.get_parameter('rate_hz').value), 1.0)
        self.t_prev = t_now

        # 오차
        e_pan  = pan_des  - self.pan_prev
        e_tilt = tilt_des - self.tilt_prev
        abs_e_pan, abs_e_tilt = abs(e_pan), abs(e_tilt)

        # 큰 오차 → 가속 모드(스무딩 약화·데드밴드 무시)
        alpha = self.alpha
        deadband = self.deadband
        if abs_e_pan > self.err_thresh or abs_e_tilt > self.err_thresh:
            alpha = max(0.6, self.alpha)   # 더 빠르게 따라가도록
            deadband = 0.0                  # 미세무시 없음

        # 데드밴드
        if abs_e_pan  < deadband:  pan_des  = self.pan_prev
        if abs_e_tilt < deadband:  tilt_des = self.tilt_prev

        # 지수평활(저역통과)
        pan_cmd  = (1.0 - self.alpha) * self.pan_prev  + self.alpha * pan_des
        tilt_cmd = (1.0 - self.alpha) * self.tilt_prev + self.alpha * tilt_des

        # 초당 속도 제한: step_limit = vmax * dt
        step_limit = self.max_speed * dt
        pan_cmd  = self._ramp(self.pan_prev,  pan_cmd,  step_limit)
        tilt_cmd = self._ramp(self.tilt_prev, tilt_cmd, step_limit)

        # 하드 한계 클램프
        pan_cmd  = max(-self.lim_pan,  min(self.lim_pan,  pan_cmd))
        tilt_cmd = max(-self.lim_tilt, min(self.lim_tilt, tilt_cmd))

        # 퍼블리시
        self.pub_cmd.publish(Float64MultiArray(data=[pan_cmd, tilt_cmd]))

        # 상태 업데이트
        self.pan_prev, self.tilt_prev = pan_cmd, tilt_cmd

    @staticmethod
    def _ramp(prev: float, target: float, max_step: float) -> float:
        if target > prev + max_step:
            return prev + max_step
        if target < prev - max_step:
            return prev - max_step
        return target

def main():
    rclpy.init()
    node = AutoAim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
