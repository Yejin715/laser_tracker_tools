#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped   # ★ 추가: PoseStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.duration import Duration

# ======= 최소 헬퍼 (tf_transformations 대체) =======
def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    # ZYX
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quat_to_mat(q):
    x,y,z,w = q
    xx,yy,zz = x*x, y*y, z*z
    xy,xz,yz = x*y, x*z, y*z
    wx,wy,wz = w*x, w*y, w*z
    return [
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy),   0],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx),   0],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy), 0],
        [0,0,0,1]
    ]

def mat_mul(A,B):
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            C[i][j] = sum(A[i][k]*B[k][j] for k in range(4))
    return C

def mat_inv(T):
    # [ R t; 0 1 ] 의 역: [ R^T  -R^T t; 0 1 ]
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    t = [T[0][3], T[1][3], T[2][3]]
    tinv = [
        -(Rt[0][0]*t[0] + Rt[0][1]*t[1] + Rt[0][2]*t[2]),
        -(Rt[1][0]*t[0] + Rt[1][1]*t[1] + Rt[1][2]*t[2]),
        -(Rt[2][0]*t[0] + Rt[2][1]*t[1] + Rt[2][2]*t[2]),
    ]
    Tinv = [[0]*4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            Tinv[i][j] = Rt[i][j]
    Tinv[0][3], Tinv[1][3], Tinv[2][3] = tinv
    Tinv[3][3] = 1.0
    return Tinv

def tf_to_mat(T: TransformStamped):
    tx,ty,tz = T.transform.translation.x, T.transform.translation.y, T.transform.translation.z
    q = (T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w)
    M = quat_to_mat(q)
    M[0][3], M[1][3], M[2][3] = tx, ty, tz
    return M

def mat_to_quat(M):
    # from rotation matrix (M[:3,:3])
    m00,m01,m02 = M[0][0],M[0][1],M[0][2]
    m10,m11,m12 = M[1][0],M[1][1],M[1][2]
    m20,m21,m22 = M[2][0],M[2][1],M[2][2]
    tr = m00+m11+m22
    if tr > 0:
        S = math.sqrt(tr+1.0)*2
        w = 0.25*S
        x = (m21-m12)/S
        y = (m02-m20)/S
        z = (m10-m01)/S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22)*2
        w = (m21 - m12)/S
        x = 0.25*S
        y = (m01 + m10)/S
        z = (m02 + m20)/S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22)*2
        w = (m02 - m20)/S
        x = (m01 + m10)/S
        y = 0.25*S
        z = (m12 + m21)/S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11)*2
        w = (m10 - m01)/S
        x = (m02 + m20)/S
        y = (m12 + m21)/S
        z = 0.25*S
    return (x,y,z,w)

def pretty_m(v: float, snap_eps: float = 1e-4, round_unit: float = 1e-3) -> float:
    """0.1mm 이하는 0으로, 그 외는 1mm(0.001m) 단위 반올림"""
    if abs(v) < snap_eps:
        return 0.0
    # 0.001 m 단위 반올림
    return round(v / round_unit) * round_unit

def snap_small(v: float, eps: float = 1e-5) -> float:
    return 0.0 if abs(v) < eps else v
# ===============================================

class EEFromTracker(Node):
    def __init__(self):
        super().__init__('ee_from_tracker')
        self.declare_parameter('tracker_frame', 'laser_tracker/optic_frame')
        self.declare_parameter('smr_frame',     'yuilrobotics/ee_smr')
        self.declare_parameter('ee_fk_frame',   'yuilrobotics/Link_6')
        self.declare_parameter('world_frame',   'world')
        self.declare_parameter('smr_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('smr_offset_rpy', [0.0, 0.0, 0.0])

        self.buf = Buffer(cache_time=Duration(seconds=10.0))
        self.tfl = TransformListener(self.buf, self)
        self.tfb = TransformBroadcaster(self)

        # ★ 추가: 포즈 퍼블리셔 2개 (측정/모델)
        self.meas_pub = self.create_publisher(PoseStamped, '/ee_meas_pose', 10)
        self.fk_pub   = self.create_publisher(PoseStamped, '/ee_fk_pose', 10)

        # 기존 에러 퍼블리셔 유지
        self.err_pub = self.create_publisher(Vector3, '/ee_position_error', 10)

        self.timer = self.create_timer(1.0/30.0, self.step)

    def step(self):
        trk = self.get_parameter('tracker_frame').value
        smr = self.get_parameter('smr_frame').value
        ee_fk = self.get_parameter('ee_fk_frame').value
        world = self.get_parameter('world_frame').value
        off_xyz = self.get_parameter('smr_offset_xyz').value
        off_rpy = self.get_parameter('smr_offset_rpy').value

        try:
            T_trk_smr = self.buf.lookup_transform(trk, smr, rclpy.time.Time())
            T_world_trk = self.buf.lookup_transform(world, trk, rclpy.time.Time())
        except Exception:
            return

        # EE<-SMR 고정 변환 (오프셋 회전 ⊕ 평행이동)
        q_off = euler_to_quat(off_rpy[0], off_rpy[1], off_rpy[2])
        M_off_R = quat_to_mat(q_off)
        M_off = [row[:] for row in M_off_R]
        M_off[0][3], M_off[1][3], M_off[2][3] = off_xyz[0], off_xyz[1], off_xyz[2]
        M_smr_ee = mat_inv(M_off)  # smr<-ee

        M_w_trk   = tf_to_mat(T_world_trk)
        M_trk_smr = tf_to_mat(T_trk_smr)
        M_w_ee = mat_mul(mat_mul(M_w_trk, M_trk_smr), M_smr_ee)

        q_w_ee = mat_to_quat(M_w_ee)
        p_w_ee = (M_w_ee[0][3], M_w_ee[1][3], M_w_ee[2][3])

        # TF 브로드캐스트: world -> yuilrobotics/ee_meas
        T = TransformStamped()
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = world
        T.child_frame_id  = 'yuilrobotics/ee_meas'
        T.transform.translation.x = float(p_w_ee[0])
        T.transform.translation.y = float(p_w_ee[1])
        T.transform.translation.z = float(p_w_ee[2])
        T.transform.rotation.x = q_w_ee[0]
        T.transform.rotation.y = q_w_ee[1]
        T.transform.rotation.z = q_w_ee[2]
        T.transform.rotation.w = q_w_ee[3]
        self.tfb.sendTransform(T)

        # ★ 추가: 측정 기반 EE 포즈 퍼블리시 (/ee_meas_pose)
        meas = PoseStamped()
        meas.header.stamp = T.header.stamp
        meas.header.frame_id = world
        meas.pose.position.x = pretty_m(T.transform.translation.x)
        meas.pose.position.y = pretty_m(T.transform.translation.y)
        meas.pose.position.z = pretty_m(T.transform.translation.z)
        meas.pose.orientation.x = snap_small(T.transform.rotation.x)
        meas.pose.orientation.y = snap_small(T.transform.rotation.y)
        meas.pose.orientation.z = snap_small(T.transform.rotation.z)
        meas.pose.orientation.w = snap_small(T.transform.rotation.w)
        self.meas_pub.publish(meas)

        # ★ 추가: FK EE 포즈 퍼블리시 (/ee_fk_pose) + 에러 계산
        try:
            T_w_eefk = self.buf.lookup_transform(world, ee_fk, rclpy.time.Time())

            fk = PoseStamped()
            fk.header.stamp = T_w_eefk.header.stamp
            fk.header.frame_id = world
            fk.pose.position.x = pretty_m(T_w_eefk.transform.translation.x)
            fk.pose.position.y = pretty_m(T_w_eefk.transform.translation.y)
            fk.pose.position.z = pretty_m(T_w_eefk.transform.translation.z)
            fk.pose.orientation.x = snap_small(T_w_eefk.transform.rotation.x)
            fk.pose.orientation.y = snap_small(T_w_eefk.transform.rotation.y)
            fk.pose.orientation.z = snap_small(T_w_eefk.transform.rotation.z)
            fk.pose.orientation.w = snap_small(T_w_eefk.transform.rotation.w)
            self.fk_pub.publish(fk)

            ex = pretty_m(fk.pose.position.x - meas.pose.position.x)
            ey = pretty_m(fk.pose.position.y - meas.pose.position.y)
            ez = pretty_m(fk.pose.position.z - meas.pose.position.z)
            self.err_pub.publish(Vector3(x=ex, y=ey, z=ez))
        except Exception:
            pass

def main():
    rclpy.init(); rclpy.spin(EEFromTracker()); rclpy.shutdown()
if __name__=='__main__': main()
