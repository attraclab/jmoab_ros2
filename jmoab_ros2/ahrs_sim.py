
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray, Int16MultiArray, UInt8
from sensor_msgs.msg import Imu, NavSatFix
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import time
import numpy as np
import os
import struct

class AHRS_SIM(Node):

	def __init__(self):

		super().__init__('jmoab_ros2_ahrs_sim')
		self.get_logger().info('Start JMOAB AHRS_Sim node')

		self.declare_parameter('show_log', False)
		self.add_on_set_parameters_callback(self.parameter_callback)
		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("show_log: {}".format(self.show_log))

		### Variables and Parameters ###
		self.sbus_cmd_throttle = 1024
		self.sbus_cmd_steering = 1024
		self.fix_stat = 0
		self.sbus_steering_stick = 1024
		self.sbus_throttle_stick = 1024
		self.lat = 0.0
		self.lon = 0.0
		self.cart_mode = 0
		self.prev_lat = self.lat
		self.prev_lon = self.lon
		self.pure_hdg = 0.0
		self.sign = 1.0
		self.brg = 0.0
		self.last_time_manual_cal = time.time()
		self.last_time_auto_cal = time.time()
		self.cal_offset = False
		self.do_estimation = False

		self.roll = 0.0
		self.pitch = 0.0

		self.last_log_stamp = time.time()

		### Kalman filter of hdg_offset estimate ###
		self.hdg_off_est = 0.0 # first guess
		self.state_predict = self.hdg_off_est
		self.error_est = 10.0 
		self.error_mea = 8.0 # a variance of

		### Jmoab Pub/Sub ###
		self.ahrs_pub = self.create_publisher(Float32MultiArray, 'jmoab/ahrs', 10)

		self.ahrs_msg = Float32MultiArray()

		self.sbus_rc_ch_sub = self.create_subscription(Int16MultiArray, 'jmoab/sbus_rc_ch', self.sbus_rc_callback, 10)
		self.cart_mode_sub = self.create_subscription(UInt8, 'jmoab/cart_mode', self.cart_mode_callback, 10)

		### Gazebo Pub/Sub ###
		self.gps_sim_sub = self.create_subscription(NavSatFix, '/ublox/fix', self.gps_callback, 10)
		self.ahrs_sim_sub = self.create_subscription(Float32MultiArray, '/atcart8/ahrs', self.ahrs_sim_callback, 10)
		
		self.get_logger().info('Publishing to /jmoab/ahrs [std_msgs/msg/Float32MultiArray]')
		self.get_logger().info('Publishing to /imu/data [sensor_msgs/msg/Imu]')

		### Loop spin ###
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)

	#######################################	
	############ ROS callbacks ############
	#######################################
	def parameter_callback(self, params):
		for param in params:
			# print(param.name, param.type_)
			if (param.name == 'show_log') and (param.type_ == Parameter.Type.BOOL):
				self.show_log = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)


	def ahrs_sim_callback(self, msg):

		self.roll = msg.data[0]
		self.pitch = msg.data[1]
		self.pure_hdg = msg.data[2]

	def sbus_rc_callback(self, msg):
		self.sbus_steering_stick = msg.data[0]
		self.sbus_throttle_stick = msg.data[1]

	def cart_mode_callback(self, msg):
		self.cart_mode = msg.data

	def gps_callback(self, msg):

		if msg.latitude != self.lat:
			self.prev_lat = self.lat

		if msg.longitude != self.lon:
			self.prev_lon = self.lon

		self.lat = msg.latitude
		self.lon = msg.longitude
		self.fix_stat = msg.status.status
		###########################################################################
		## we put the calculation/estimation inside the gps_callback             ##
		## because the sampling time of gps is slowest,                          ##
		## so we could get the real difference between current and previous data ##
		###########################################################################
		if self.do_estimation:
			self.brg = self.get_bearing(self.prev_lat, self.prev_lon, self.lat, self.lon)
			self.brg = self.ConvertTo360Range(self.brg)

			if self.cart_mode == 1:
				self.period_manual_cal = time.time() - self.last_time_manual_cal
				if self.period_manual_cal > 1.0:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.brg, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)
			
			elif self.cart_mode == 2:
				self.period_auto_cal = time.time() - self.last_time_auto_cal
				if self.period_auto_cal > 0.3:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.brg, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)


	################################	
	############ Maths #############
	################################

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	def ConvertTo180Range(self, deg):

		deg = self.ConvertTo360Range(deg)
		if deg > 180.0:
			deg = -(180.0 - (deg%180.0))

		return deg

	def get_bearing(self, lat1, lon1, lat2, lon2):

		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		y = np.sin(dLon)*np.cos(lat_end)
		x = np.cos(lat_start)*np.sin(lat_end) - np.sin(lat_start)*np.cos(lat_end)*np.cos(dLon)
		bearing = np.degrees(np.arctan2(y,x))

		return bearing


	def find_smallest_diff_ang(self, goal, cur):

		## goal is in 180ranges, we need to convert to 360ranges first

		diff_ang1 = abs(self.ConvertTo360Range(goal) - cur)

		if diff_ang1 > 180.0:

			diff_ang = 180.0 - (diff_ang1%180.0)
		else:
			diff_ang = diff_ang1

		## check closet direction
		compare1 = self.ConvertTo360Range(self.ConvertTo360Range(goal) - self.ConvertTo360Range(cur + diff_ang))
		compare2 = self.ConvertTo180Range(goal - self.ConvertTo180Range(cur + diff_ang))
		# print(compare1, compare2)
		if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
			sign = 1.0 # clockwise count from current hdg to target
		else:
			sign = -1.0 # counter-clockwise count from current hdg to target


		return diff_ang, sign

	def kalman_filter(self, measure, prev_state_est, prev_error_est):

		## reference
		## https://www.kalmanfilter.net/kalman1d.html

		######################
		## state estimation ##
		######################
		KG = prev_error_est/(prev_error_est + self.error_mea)
		cur_state_est = prev_state_est + KG*(measure - prev_state_est)
		cur_error_est = (1 - KG)*(prev_error_est)

		################
		## prediction ##
		################
		self.error_est = cur_error_est + 0.01 # 0.01 is process noise, could help when hdg_offset is fluctuating during time
		
		# hdg_offset is not dynamic behaviour, so predicted state is constant as estimated state
		self.state_predict = cur_state_est  

		return cur_state_est

	#######################################	
	######## ROS2 timer callback ##########
	#######################################

	def timer_callback(self):


		##########################################
		### Kalman filtering on yaw or heading ###
		##########################################
		hdg = self.ConvertTo360Range(self.pure_hdg + (self.sign)*self.hdg_off_est)

		## in manual case
		if self.cart_mode == 1:
			###############################################################
			## we do estimation only when throttle is up and no steering ##
			###############################################################
			if (self.sbus_throttle_stick > 1048) and (924 < self.sbus_steering_stick < 1124):
				self.do_estimation = True
			
			else:
				self.last_time_manual_cal = time.time()
				self.last_time_auto_cal = time.time()
				self.cal_offset = False
				self.do_estimation = False

		## in auto case
		elif self.cart_mode == 2:
			#######################################################
			## we do estimation only when sbus throttle is high, ##
			## and sbus steering is no curvy                     ##
			#######################################################
			# if (self.sbus_cmd_throttle > 1090) and (970 < self.sbus_cmd_steering < 1074):
			# 	self.do_estimation = True
			# else:
			# 	self.last_time_manual_cal = time.time()
			# 	self.last_time_auto_cal = time.time()
			# 	self.cal_offset = False
			# 	self.do_estimation = False
			pass
		else:
			self.last_time_manual_cal = time.time()
			self.last_time_auto_cal = time.time()


		if ((time.time() - self.last_log_stamp) > 1.0) and self.show_log:
			self.get_logger().info("p_hdg: {:.2f} hdg_off_est: {:.2f} brg: {:.2f} hdg: {:.2f} KF_cal: {}".format(\
				self.pure_hdg, self.hdg_off_est, self.brg, hdg, self.cal_offset))
			self.last_log_stamp = time.time()

		self.ahrs_msg.data = [self.roll, self.pitch, hdg]

		self.ahrs_pub.publish(self.ahrs_msg)



def main(args=None):

	rclpy.init(args=args)
	node = AHRS_SIM()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":

	main()
