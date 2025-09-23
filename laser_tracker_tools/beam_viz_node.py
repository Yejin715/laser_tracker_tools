# BeamViz 대체: CYLINDER로 그리기
#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from builtin_interfaces.msg import Time

def quat_from_two_vectors(src, dst):
    # src, dst: 정규화된 3D 벡터(tuple/list)
    import numpy as np
    s = np.array(src, dtype=float)
    d = np.array(dst, dtype=float)
    s /= (np.linalg.norm(s) + 1e-12)
    d /= (np.linalg.norm(d) + 1e-12)
    v = np.cross(s, d)
    c = float(np.dot(s, d))
    if c < -0.999999:  # 정반대
        # src에 수직인 임의축
        axis = np.array([1,0,0]) if abs(s[0]) < 0.9 else np.array([0,1,0])
        v = np.cross(s, axis); v /= (np.linalg.norm(v)+1e-12)
        x,y,z = v; w = 0.0  # 180도 회전
    else:
        k = math.sqrt((1.0 + c) * 2.0)
        x,y,z = v / (k + 1e-12)
        w = 0.5 * k
    return (float(x), float(y), float(z), float(w))

class BeamViz(Node):
    def __init__(self):
        super().__init__('beam_viz')
        self.declare_parameter('tracker_frame','laser_tracker/optic_frame')
        self.declare_parameter('target_frame', 'yuilrobotics/ee_smr')
        self.declare_parameter('topic','/laser_tracker/beam_marker')
        self.declare_parameter('fallback_length', 3.0)  # 타깃 없을 때 길이
        self.declare_parameter('diameter', 0.01)        # 원기둥 직경
        self.tfbuf = Buffer(cache_time=Duration(seconds=30.0))
        self.tfl  = TransformListener(self.tfbuf, self)
        self.pub  = self.create_publisher(Marker, self.get_parameter('topic').value, 10)
        self.timer= self.create_timer(1.0/30.0, self.step)

    def step(self):
        trk = self.get_parameter('tracker_frame').value
        tgt = self.get_parameter('target_frame').value
        D   = float(self.get_parameter('diameter').value)
        try:
            tf = self.tfbuf.lookup_transform(trk, tgt, rclpy.time.Time())
            x,y,z = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
            L = math.sqrt(x*x+y*y+z*z) or 1e-6
            ex,ey,ez = x/L, y/L, z/L
        except Exception:
            ex,ey,ez = 1.0,0.0,0.0
            L = float(self.get_parameter('fallback_length').value)

        # CYLINDER는 로컬 Z축이 길이 방향 → +Z를 타깃방향으로 회전
        qx,qy,qz,qw = quat_from_two_vectors((0,0,1), (ex,ey,ez))

        m = Marker()
        m.header.frame_id = trk
        m.header.stamp = Time(sec=0, nanosec=0)  # 최신 TF 기준
        m.ns, m.id, m.type, m.action = 'beam', 0, Marker.CYLINDER, Marker.ADD

        # 원기둥 중심을 빔의 중간 위치로
        m.pose.position.x = 0.5 * L * ex
        m.pose.position.y = 0.5 * L * ey
        m.pose.position.z = 0.5 * L * ez
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        m.scale.x = D  # 지름 X
        m.scale.y = D  # 지름 Y
        m.scale.z = L  # 길이 Z

        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.9
        self.pub.publish(m)

def main():
    rclpy.init(); rclpy.spin(BeamViz()); rclpy.shutdown()
if __name__=='__main__': main()
