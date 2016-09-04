import rospy
import numpy as np
import madgwick_filter
from sensor_msgs.msg import Imu

_pub = rospy.Publisher('info/imu/data', Imu, queue_size = 10)
_sampleRate = 400 # 400Hz

_Filter = madgwick_filter.MadgwickAHRS(1.0/_sampleRate)

_prevT = -1

def callback(data):
    global _Filter
    global _pub
    global _prevT
    global _sampleRate

    if _prevT is not -1:
        if (data.header.nsec - _prevT) >= 1000000.0/_sampleRate :
            acc = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
            gyro = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]

            _Filter.update_imu(gyro, acc)
            msg = Imu()
            msg.header.frame_id = "imu"
            msg.orientation.w = _Filter.quaternion._q[0]
            msg.orientation.x = _Filter.quaternion._q[1]
            msg.orientation.y = _Filter.quaternion._q[2]
            msg.orientation.z = _Filter.quaternion._q[3]

            _pub.publish(msg)
            _prevT = _prevT + 1000000.0/_sampleRate
    else :
        _prevT = data.header.nsec;

def listener():
    rospy.init_node('imu_filter_run')
    rospy.Subscriber("info/imu/data_raw", Imu, callback)

    rospy.loginfo(" Test: start spinning!")
    rospy.spin()
