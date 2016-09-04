import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from math import pi, atan2, acos, cos
from time import sleep

_pub = rospy.Publisher('control/pwm', Float32MultiArray, queue_size=20)
msg = Float32MultiArray()
flag_shutdown = 0

def callback(data):
	global _pub
	global msg

	p_coeff = 0.5
	pwm_array = [10., 10., 10., 10., 100, 0, 0, 0]
	pwm_example = [12., 12., 12., 12., 100, 0, 0, 0]
	wing_phi_offset = [pi*3/4, -pi*3/4, pi/4, -pi/4]

	quaternion = [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
	for i in range(4):print quaternion[i],', ',
	print ''
		
 
	theta = acos(quaternion[0])/2.0
	plane_x = 1-2*(quaternion[2]**2)-2*(quaternion[3]**2)
	plane_y = 2*quaternion[1]*quaternion[2] + 2*quaternion[0]*quaternion[3]
	
	try:
		phi = atan2(plane_y,plane_x)
	except:
		print Exception
	
	for i in range(4):
		# i+1st wing control
		pwm_array[i] = 17. + p_coeff*cos(phi-wing_phi_offset[i])*theta
		if pwm_array[i]>18.: pwm_array[i]=18.
		if pwm_array[i]<16.: pwm_array[i]=16.
		#print pwm_array[i], ', ',
	#print pwm_array[4]
	
	msg.data =  pwm_array
	if flag_shutdown: msg.data=[10.0, 10.0, 10.0, 10.0, 0, 0, 0, 0]
	_pub.publish(msg)


def shut_Handler():
	print("shutdown node & drone")
	global flag_shutdown
	flag_shutdown = 1

def esc_init():
	rospy.init_node('chung_control_run')	
	rospy.on_shutdown(shut_Handler)
	global _pub
	global msg
	
	msg.data = [20., 20., 20., 20., 0, 0, 0, 0]
	_pub.publish(msg)
	sleep(0.5)	
	
	msg.data = [20., 20., 20., 20., 100, 0, 0, 0]
	_pub.publish(msg)
	sleep(8)	
	
	msg.data = [10., 10., 10., 10., 100, 0, 0, 0]
	_pub.publish(msg)
	sleep(8)	


def listener():
	rospy.Subscriber("info/imu/data", Imu, callback)
	rospy.loginfo("data!")
	rospy.spin()
	

if __name__=='__main__':
	print("fuck")


