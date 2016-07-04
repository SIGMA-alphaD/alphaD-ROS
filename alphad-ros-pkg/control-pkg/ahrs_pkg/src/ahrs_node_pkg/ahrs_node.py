import rospy
import numpy as np
from imu_baro_pkg.msg import msgImuBaro
from ahrs_pkg.msg import msgAhrs

_initState = 0
_prevT = 0
_q_acc = [0] * 4
_q_mag = [0] * 4

# sudo _mag
_mag = [1, 0, 0]

_pub = rospy.Publisher('ahrs_msg', msgAhrs, queue_size = 10)

def callback(data):
    global _initState
    global _prevT
    global _q_acc
    global _q_mag
    global _mag

    global _pub

    if _initState == 0:
        _initState = 1
        _prevT = data.msTime / 1000.0

    else:
        dt = data.msTime / 1000.0 - _prevT
        _prevT = data.msTime / 1000.0
        acc = [data.Acc.x, data.Acc.y, data.Acc.z]
        gyro = [data.Gyro.x, data.Gyro.y, data.Gyro.z]

        # update _mag
        m_dot = np.cross(gyro, _mag)
        _mag = np.add(_mag, -np.multiply(m_dot, dt))
        _mag = _mag / np.linalg.norm(_mag)

        # q_acc
        acc = acc / np.linalg.norm(acc)

        if acc[2] >= 0:
            _q_acc[0] = np.sqrt((1 + acc[2]) / 2)
            _q_acc[1] = acc[1] / np.sqrt(2 * (1 + acc[2]))
            _q_acc[2] = -acc[0] / np.sqrt(2 * (1 + acc[2]))
            _q_acc[3] = 0
        else:
            _q_acc[0] = acc[1] / np.sqrt(2 * (1 - acc[2]))
            _q_acc[1] = np.sqrt((1 - acc[2]) / 2)
            _q_acc[2] = 0
            _q_acc[3] = acc[0] / np.sqrt(2 * (1 - acc[2]))

        _q_acc = _q_acc / np.linalg.norm(_q_acc)

        # q_mag
        m = [0, _mag[0], _mag[1], _mag[2]]
        l = q_mult(q_mult(_q_acc, m), q_conjugate(_q_acc))
        psi = np.arctan2(l[2], l[1])

        _q_mag[0] = np.cos(psi / 2)
        _q_mag[1] = 0
        _q_mag[2] = 0
        _q_mag[3] = -np.sin(psi / 2)

        # local to global quaternion
        q_GL = q_mult(_q_mag, _q_acc)
        q_GL = q_GL / np.linalg.norm(q_GL)

        euler = quat2euler(q_GL)

        msg = msgAhrs()
        msg.roll = euler[0]
        msg.pitch = euler[1]
        msg.yaw = euler[2]

        _pub.publish(msg)


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [w, x, y, z]


def q_conjugate(q):
    w, x, y, z = q
    return [w, -x, -y, -z]


def quat2euler(q):
    q1, q2, q3, q4 = q

    phi = np.arctan2(2 * (q1 * q2 + q3 * q4), 1 - 2 * (q2 * q2 + q3 * q3))
    theta = np.arcsin(2 * (q1 * q3 - q4 * q2))
    psi = np.arctan2(2 * (q2 * q3 + q1 * q4), 1 - 2 * (q3 * q3 + q4 * q4))

    return [phi, theta, psi]


def listener():
    rospy.init_node('ahrs_node')
    rospy.Subscriber("imu_baro_msg", msgImuBaro, callback)

    rospy.loginfo(" Test: start spinning!")
    rospy.spin()
