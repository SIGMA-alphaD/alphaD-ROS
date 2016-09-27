import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from math import pi, atan2, acos, cos, sqrt
from time import sleep
from random import gauss

_pub = rospy.Publisher('control/pwm', Float32MultiArray, queue_size=20)
msg = Float32MultiArray()
flag_shutdown = 0

#_weight0 = [1., 1., 1., 1., 1.]
#_Reward_best = -10000000.
#_weight_best = _weight0 
_first_confirm = 1
#_X_matrix = []
#_Y_matrix = []
_pwm_array = [10., 10., 10., 10., 0, 0, 0, 0]
#_step = 0

def callback(data):
	global _pub
	global msg
	

	p_coeff = 0.5
	pwm_array = [10., 10., 10., 10., 100, 0, 0, 0]
	pwm_example = [12., 12., 12., 12., 100, 0, 0, 0]
	wing_phi_offset = [pi*3/4, -pi*3/4, pi/4, -pi/4]

	reward = -100

	quaternion = [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
	#for i in range(4):print quaternion[i],', ',
	#print ''
	


	# *******************************************
	# ************** Control Start **************
 	# *******************************************
	


	"""
	## Continous MDP RL control 
	global _weight0
	global _Reward_best
	global _weight_best
	global _first_confirm

	for w->converge:
	

 	first_confirm = 1
		R_total = 0.
		# 'for' skip
		s0 = quaternion		_X_matrix.append([1])
		_X_matrix[len(_X_matrix)].append(s0)
		for i in range(len(_X_matrix)):
	"""		
			

	
	## basic P-control
	omega = acos(quaternion[0])/2.0

	plane_x = 2*quaternion[0]*quaternion[2] + 2*quaternion[1]*quaternion[3] #1-2*(quaternion[2]**2)-2*(quaternion[3]**2)
	plane_y = -2*quaternion[0]*quaternion[1] + 2*quaternion[2]*quaternion[3] #2*quaternion[1]*quaternion[2] + 2*quaternion[0]*quaternion[3]
	plane_z = 1 - 2*quaternion[1]**2 - 2*quaternion[2]**2#-2*quaternion[0]*quaternion[2] + 2*quaternion[1]*quaternion[3]
	#print omega,', ',plane_x,', ',plane_y,', ',plane_z
	if reward < -(plane_x + plane_y)*100:
		reward = -100*(plane_x + plane_y)
		p_coeff_weight_best = p_coeff		
	
	p_coeff = p_coeff_weight_best + gauss(0,0.1)
	#print p_coeff

	try:
		theta = acos(plane_z/sqrt(plane_x**2 + plane_y**2 + plane_z**2))
		phi = atan2(plane_y, plane_x)
	except:
		print Exception
	
	for i in range(4):
		# i+1st wing control
		pwm_array[i] = 17. + p_coeff*cos(phi-wing_phi_offset[i])*theta
		if pwm_array[i]>18.: pwm_array[i]=18.
		if pwm_array[i]<16.: pwm_array[i]=16.
		#print pwm_array[i], ', ',
	
	#print pwm_array[4]
	

	
	# *****************************************
	# ************** Control End ************** 
	# *****************************************


	
	msg.data =  pwm_array
	if flag_shutdown: msg.data=[10.0, 10.0, 10.0, 10.0, 0, 0, 0, 0]
	_pub.publish(msg)
"""
def Reward(state):
	global _X_matrix
	global _step
	buff = 0
	c1 = 
	for i in range(len(_X_matrix)):
		buff += c1*((state[1]-0)**2 + (state[2]-0)**2) 
	err = -buff

	buff = 0		
	c2 =
	err = err - c2*_step

	buff
	c3 = 
	for 
	err = err - c3*
	return err 	

def Policy(weight, theta, phi):

"""

def shut_Handler():
	#print("shutdown node & drone")
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


