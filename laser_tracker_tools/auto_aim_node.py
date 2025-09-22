#!/usr/bin/env python3
# auto_aim_node.py (ROS 2 Humble)
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class AutoAim(Node):
    def __init__(self):
        super().__init__('laser_tracker_auto_aim')
        self.declare_parameter('tracker_frame', 'laser_tracker/base_link')
        self.declare_parameter('target_frame',  'yuilrobotics/ee_smr')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('kp_pan',  1.2)
        self.declare_parameter('kp_tilt', 1.2)
        self.declare_parameter('lim_pan',  math.radians(85.0))
        self.declare_parameter('lim_tilt', math.radians(60.0))

        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self)
        self.pub_cmd = self.create_publisher(Float64MultiArray,
                                             '/laser_tracker/pan_tilt_controller/commands', 10)
        self.timer = self.create_timer(1.0/float(self.get_parameter('rate_hz').value), self.step)

        self.trk = self.get_parameter('tracker_frame').value
        self.tgt = self.get_parameter('target_frame').value
        self.kp_pan  = float(self.get_parameter('kp_pan').value)
        self.kp_tilt = float(self.get_parameter('kp_tilt').value)
        self.lim_pan = float(self.get_parameter('lim_pan').value)
        self.lim_tilt= float(self.get_parameter('lim_tilt').value)

    def step(self):
        try:
            tf: TransformStamped = self.tfbuf.lookup_transform(self.trk, self.tgt, rclpy.time.Time())
        except Exception:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        # 원하는 각도(조준 각): pan=yaw, tilt=pitch
        pan_des  = math.atan2(y, x)
        tilt_des = math.atan2(z, math.hypot(x, y))
        # 간단 P제어 (현재 각도 추정이 없으니 목표를 곧바로 명령)
        pan_cmd  = max(-self.lim_pan,  min(self.lim_pan,  pan_des))
        tilt_cmd = max(-self.lim_tilt, min(self.lim_tilt, tilt_des))

        msg = Float64MultiArray(data=[pan_cmd, tilt_cmd])
        self.pub_cmd.publish(msg)

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
