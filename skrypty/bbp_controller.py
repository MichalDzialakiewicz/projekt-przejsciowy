#!/usr/bin/env python

import math
import csv
import rospy
import os
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from tf.transformations import euler_from_quaternion
import tf

from enum import Enum
from modules.pid import UavPID

class DronesState(Enum):
	LANDED = 0
	TAKING_OFF = 1
	HOVERING = 2
	FLYING = 3
	LANDING = 4
	EMERGENCY = 5
	USERTAKEOFF = 6
	MOTOR_RAMPING = 7
	EMERGENCY_LANDING = 8


drones_state = DronesState.LANDED


# worlds coordinate system
class ReferenceSystemTransformer:
	def x(self, target_pos, current_pos):
		err_x = target_pos[0] - current_pos[0]
		err_y = target_pos[1] - current_pos[1]
		return err_x * math.cos(current_pos[3]) + err_y * math.sin(current_pos[3])

	def y(self, target_pos, current_pos):
		err_x = target_pos[0] - current_pos[0]
		err_y = target_pos[1] - current_pos[1]
		return err_y * math.cos(current_pos[3]) - err_x * math.sin(current_pos[3])


# global variables
action_takeoff = None
action_land = None
action_move = None
supervisor = None

PID_handler = None
PID_error_locked = True
RST_handler = None

target_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw
target_pos_norm = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw
base_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw
current_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw

__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))


def state_callback(data):
	global drones_state
	drones_state = data.state


def odom_callback(data):
	global PID_handler
	global base_pos
	global target_pos
	global PID_error_locked
	global target_pos_norm
	global current_pos
	global x_container
	global y_container
	global yaw_container

	x_val = data.pose.pose.position.x
	y_val = data.pose.pose.position.y
	x_container = data.pose.pose.position.x
	y_container = data.pose.pose.position.y

	(ar, ap, yaw_val) = tf.transformations.euler_from_quaternion(
		[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
		 data.pose.pose.orientation.w])

	current_pos[0] = (x_val * math.cos(yaw_val)) + (y_val * math.sin(yaw_val))
	current_pos[1] = (-x_val * math.sin(yaw_val)) + (y_val * math.cos(yaw_val))
	current_pos[2] = data.pose.pose.position.z
	current_pos[3] = yaw_val

	yaw_container = yaw_val

	if PID_error_locked == False:
		PID_handler.insert_error_value_pair('X', target_pos[0], current_pos[0])
		PID_handler.insert_error_value_pair('Y', target_pos[1], current_pos[1])
		PID_handler.insert_error_value_pair('Z', target_pos[2], current_pos[2])
		PID_handler.insert_error_value_pair('YAW', target_pos[3], current_pos[3])


def tracking_callback(data):  # X

	trajectory = []
	with open(os.path.join(__location__, "trajectory.csv")) as csv_file:
		csv_reader = csv.reader(csv_file)
		for line in csv_reader:
			tmp = float(line[0])
			trajectory.append(tmp)

	rospy.loginfo('New task has been received!')

	empty_msg = Empty()
	twist_msg = Twist()

	twist_msg.linear.x = 0
	twist_msg.linear.y = 0
	twist_msg.linear.z = 0
	twist_msg.angular.x = 0
	twist_msg.angular.y = 0
	twist_msg.angular.z = 0

	global RST_handler
	RST_handler = ReferenceSystemTransformer()

	rospy.loginfo('Bebop Take Off')
	action_takeoff.publish(empty_msg)
	global drones_state
	while not(drones_state == DronesState.HOVERING.value):
		continue

	global base_pos
	global target_pos
	global current_pos
	global PID_error_locked
	global target_pos_norm

	target_pos[0] = 0
	target_pos[1] = 0
	target_pos[2] = 0
	target_pos[3] = 0

	rate_x_test = rospy.Rate(150)
	result_file_h = open(os.path.join(__location__, "results.csv"), "w")
	iteration = 1

	for kp in range(60, 81, 1):
		for ki in range(1, 21, 1):
			for kd in range(30, 61, 1):
				n_kp = ((kp * 1.0) / 100)
				n_ki = ((ki * 1.0) / 10000)
				n_kd = kd

				PID_error_locked = True
				global PID_handler
				PID_handler = UavPID([n_kp, 0, 0, 0], [n_ki, 0, 0, 0], [n_kd, 0, 0, 0])
				PID_error_locked = False

				rospy.loginfo('Bebop Move [#X] [' + str(iteration) + ']')

				pid_trigger = 0.05
				J_data = []
				J = 0.0
				D_data = []
				D = 0.0

				for point in trajectory:
					target_pos[0] = point
					PID_handler.reset_errors_list()
					PID_result = 1.0
					loop_cond = 0

					while (abs(PID_result) > pid_trigger) or (loop_cond < 180):

						PID_result = PID_handler.calculate('X')
						J_data.append(abs(target_pos[0] - current_pos[0]))
						twist_msg.linear.x = PID_result
						action_move.publish(twist_msg)
						rate_x_test.sleep()
						loop_cond += 1
					if target_pos[0] == 1.5 or target_pos[0] == -1.5 or target_pos[0] == 1 or target_pos[0] == -1:
						D_data.append(abs(target_pos[0] - current_pos[0]))
						rospy.loginfo(str(target_pos[0]) + '\t' + str(current_pos[0]) + '\t' + str(abs(target_pos[0] - current_pos[0])) + '\n')

				for j in J_data:
					J += j

				for d in D_data:
					D += d

				iteration += 1

				rospy.loginfo('Kp = ' + str(n_kp) + '\t' + 'Ki = ' + str(n_ki) + '\t' + 'Kd = ' + str(n_kd) + '\n' + 'J = ' + str(J) + '\t' + 'D = ' + str(D) + '\n')
				result_file_h.write(str(n_kp) + '\t' + str(n_ki) + '\t' + str(n_kd) + '\t' + str(J) + '\t' + str(D) + '\n')
				result_file_h.flush()

	result_file_h.close()
	rospy.sleep(5)

	rospy.loginfo('Bebop Land')
	action_land.publish(empty_msg)

	while not(drones_state == DronesState.LANDED.value):
		continue

	rospy.loginfo('Done!')


def dbg_callback(data):  # YAW
	rospy.loginfo('New task has been received!')

	empty_msg = Empty()
	twist_msg = Twist()

	twist_msg.linear.x = 0
	twist_msg.linear.y = 0
	twist_msg.linear.z = 0
	twist_msg.angular.x = 0
	twist_msg.angular.y = 0
	twist_msg.angular.z = 0

	global RST_handler
	RST_handler = ReferenceSystemTransformer()

	rospy.loginfo('Bebop Take Off')
	action_takeoff.publish(empty_msg)
	global drones_state
	while not(drones_state == DronesState.HOVERING.value):
		continue

	global base_pos
	global target_pos
	global current_pos
	global PID_error_locked
	global target_pos_norm

	target_pos[0] = 0
	target_pos[1] = 0
	target_pos[2] = 0
	target_pos[3] = 0

	rate_yaw_test = rospy.Rate(90)
	result_file_h = open(os.path.join(__location__, "results.csv"), "w")
	iteration = 1

	for kp in range(60, 80, 1):
		for ki in range(10, 20, 1):
			for kd in range(40, 80, 1):
				n_kp = 1.0     # ((kp * 1.0) / 100)
				n_ki = 0.0002  # ((ki * 1.0) / 10000)
				n_kd = 10.0    # kd

				PID_error_locked = True
				global PID_handler
				PID_handler = UavPID([0, 0, 0, n_kp], [0, 0, 0, n_ki], [0, 0, 0, n_kd])
				PID_error_locked = False

				pid_trigger = 0.008
				J_data = []
				J = 0.0
				D_data = []
				D = 0.0

				target_pos[3] = 2.0  # yaw
				PID_handler.reset_errors_list()
				rospy.loginfo('Bebop Move [#YAW]')
				PID_result = 1.0
				loop_cond = 0

				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):

					PID_result = PID_handler.calculate('YAW')
					J_data.append(abs(target_pos[3] - current_pos[3]))
					twist_msg.angular.z = PID_result
					action_move.publish(twist_msg)
					rate_yaw_test.sleep()
					loop_cond += 1
				D_data.append(abs(target_pos[3] - current_pos[3]))

				target_pos[3] = -2.5
				PID_handler.reset_errors_list()
				rospy.loginfo('Bebop Move [#YAW]')
				PID_result = 1.0
				loop_cond = 0

				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):

					PID_result = PID_handler.calculate('YAW')
					J_data.append(abs(target_pos[3] - current_pos[3]))
					twist_msg.angular.z = PID_result
					action_move.publish(twist_msg)
					rate_yaw_test.sleep()
					loop_cond += 1
				D_data.append(abs(target_pos[3] - current_pos[3]))

				target_pos[3] = 3.0
				PID_handler.reset_errors_list()
				rospy.loginfo('Bebop Move [#YAW]')
				PID_result = 1.0
				loop_cond = 0

				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):

					PID_result = PID_handler.calculate('YAW')
					J_data.append(abs(target_pos[3] - current_pos[3]))
					twist_msg.angular.z = PID_result
					action_move.publish(twist_msg)
					rate_yaw_test.sleep()
					loop_cond += 1
				D_data.append(abs(target_pos[3] - current_pos[3]))

				target_pos[3] = 0.0
				PID_handler.reset_errors_list()
				rospy.loginfo('Bebop Move [#YAW] (backward)')
				PID_result = 1.0
				loop_cond = 0

				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):

					PID_result = PID_handler.calculate('YAW')
					J_data.append(abs(target_pos[3] - current_pos[3]))
					twist_msg.angular.z = PID_result
					action_move.publish(twist_msg)
					rate_yaw_test.sleep()
					loop_cond += 1
				D_data.append(abs(target_pos[3] - current_pos[3]))

				for j in J_data:
					J += j

				for d in D_data:
					D += d

				iteration += 1

				rospy.loginfo('Kp = ' + str(n_kp) + '\t' + 'Ki = ' + str(n_ki) + '\t' + 'Kd = ' + str(n_kd) + '\n' + 'J = ' + str(J) + '\t' + 'D = ' + str(D) + '\n')
				result_file_h.write(str(n_kp) + '\t' + str(n_ki) + '\t' + str(n_kd) + '\t' + str(J) + '\t' + str(D) + '\n')
				result_file_h.flush()

	result_file_h.close()
	rospy.sleep(5)

	rospy.loginfo('Bebop Land')
	action_land.publish(empty_msg)

	while not(drones_state == DronesState.LANDED.value):
		continue

	rospy.loginfo('Done!')


def performance_callback(data):  # Z
	rospy.loginfo('starting performance tests!')

	rate_z_test = rospy.Rate(90)
	empty_msg = Empty()
	twist_msg = Twist()

	twist_msg.linear.x = 0
	twist_msg.linear.y = 0
	twist_msg.linear.z = 0
	twist_msg.angular.x = 0
	twist_msg.angular.y = 0
	twist_msg.angular.z = 0

	global RST_handler
	RST_handler = ReferenceSystemTransformer()

	rospy.loginfo('Bebop Take Off #1')
	action_takeoff.publish(empty_msg)
	global drones_state
	while not(drones_state == DronesState.HOVERING.value):
		continue

	global base_pos
	global target_pos
	global current_pos
	global PID_error_locked
	global pos_flag_1
	global pos_flag_2
	global pos_flag_3
	global pos_flag_4
	global target_pos_norm

	target_pos[0] = 0
	target_pos[1] = 0
	target_pos[2] = 0
	target_pos[3] = 0
	pid_trigger = 0.05
	result_file_h = open(os.path.join(__location__, "results_test.csv"), "w")
	wynik_file = open(os.path.join(__location__, "wynik.csv"), "w")
	for kp in range(83, 85, 1):
		for ki in range(11, 21, 1):
			for kd in range(30, 70, 1):
				n_kp = ((kp * 1.0) / 10)
				n_ki = ((ki * 1.0) / 10000)
				n_kd = kd

				pos_flag_1 = 0 #3.0
				pos_flag_2 = 0 #1.2
				pos_flag_3 = 0 #3.0
				pos_flag_4 = 0 #1.5

				PID_error_locked = True

				global PID_handler
				PID_handler = UavPID([0, 0, 6.4, 0], [0, 0, 0.0001, 0], [0, 0, 34, 0])

				PID_error_locked = False

				target_pos[2] = 2.0  # Z
				PID_handler.reset_errors_list()

				rospy.loginfo('Bebop Move [#Z] (UP)')

				PID_result = 1.0
				loop_cond = 0
				diff_data = []
				J = 0.0
				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):

					PID_result = PID_handler.calculate('Z')

					diff_data.append(abs(target_pos[2] - current_pos[2]))
					wynik_file.write(str(current_pos[2]) + '\n')
					twist_msg.linear.z = PID_result

					action_move.publish(twist_msg)
					#supervisor.publish(PID_result)
					rate_z_test.sleep()

					loop_cond += 1

				pos_flag_1 = current_pos[2] #3.0

				target_pos[2] = 1.5  # Z
				PID_handler.reset_errors_list()

				rospy.loginfo('Bebop Move [#Z] (DOWN)')

				PID_result = 1.0
				loop_cond = 0
				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):
					PID_result = PID_handler.calculate('Z')
					diff_data.append(abs(target_pos[2] - current_pos[2]))
					wynik_file.write(str(current_pos[2]) + '\n')
					twist_msg.linear.z = PID_result

					action_move.publish(twist_msg)
					rate_z_test.sleep()

					loop_cond += 1

				pos_flag_2 = current_pos[2] #1.2
				target_pos[2] = 2.5  # Z
				PID_handler.reset_errors_list()

				rospy.loginfo('Bebop Move [#Z] (UP)')

				PID_result = 1.0
				loop_cond = 0
				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):
					PID_result = PID_handler.calculate('Z')
					diff_data.append(abs(target_pos[2] - current_pos[2]))
					wynik_file.write(str(current_pos[2]) + '\n')
					twist_msg.linear.z = PID_result

					action_move.publish(twist_msg)
					rate_z_test.sleep()

					loop_cond += 1

				pos_flag_3 = current_pos[2] #3.0

				target_pos[2] = 1.0  # Z
				PID_handler.reset_errors_list()

				rospy.loginfo('Bebop Move [#Z] (DOWN)')

				PID_result = 1.0
				loop_cond = 0
				while (abs(PID_result) > pid_trigger) or (loop_cond < 180):
					PID_result = PID_handler.calculate('Z')
					diff_data.append(abs(target_pos[2] - current_pos[2]))
					wynik_file.write(str(current_pos[2]) + '\n')
					twist_msg.linear.z = PID_result

					action_move.publish(twist_msg)
					rate_z_test.sleep()

					loop_cond += 1

				pos_flag_4 = current_pos[2] #1.5

				for d in diff_data:
					J += d

				rospy.loginfo('Kp = 6.4' + '\t' + 'Ki = 0.0001' + '\t' + 'Kd = 34' + '\t' + 'J = ' + str(J) + '\n')
				rospy.loginfo(str(pos_flag_1) + '\t' + str(pos_flag_2) + '\t' + str(pos_flag_3) + '\t' + str(pos_flag_4) + '\n')
				result_file_h.write(str(n_kp) + '\t' + str(n_ki) + '\t' + str(n_kd) + '\t' + str(J) + '\t' + str(pos_flag_1) + '\t' + str(pos_flag_2) + '\t' + str(pos_flag_3) + '\t' + str(pos_flag_4) +'\n')
				result_file_h.flush()
	wynik_file.close()
	result_file_h.close()

	rospy.sleep(5)

	rospy.loginfo('Bebop Land #1')
	action_land.publish(empty_msg)

	while not(drones_state == DronesState.LANDED.value):
		continue

	rospy.loginfo('Done!')


def bbp_controller():
	rospy.init_node('bbp_controller', anonymous=True)

	global action_takeoff
	global action_land
	global action_move
	global supervisor

	state_changed = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/FlyingStateChanged', Ardrone3PilotingStateFlyingStateChanged, state_callback)
	rospy.Subscriber('performance_testing', Empty, performance_callback)
	rospy.Subscriber('auto_tune_task', Empty, dbg_callback)
	rospy.Subscriber('tracking', Empty, tracking_callback)
	supervisor = rospy.Publisher('/supervise', Float64, queue_size=10)
	action_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
	action_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
	action_move = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1000)
	odom_topic_handler = rospy.Subscriber('/repeater/bebop2/pose/info', Odometry, odom_callback)

	rospy.loginfo('Node activated!')

	rospy.spin()

	rospy.loginfo('Node closed!')


if __name__ == '__main__':
	bbp_controller()

