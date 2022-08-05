
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray, Int16MultiArray, UInt8
from sensor_msgs.msg import Imu, NavSatFix
import time
import numpy as np
from smbus2 import SMBus
import os
import struct
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

## BNO055 address
IMU_ADDR = 0x28

## Registers and Values
### config
OPR_REG = 0x3d
SYS_TRIGGER = 0x3F
AXIS_MAP_CONFIG_REG = 0x41
AXIS_MAP_SIGN_REG = 0x42
### OPR_REG mode
CONFIG_MODE = 0x00
IMU_MODE = 0x08
NDOF_FMC_OFF = 0x0b
NDOF_MODE = 0x0C
### Reset ###
RST_SYS = 0x20 # 5th bit
### AXIS REMAP
REMAP_DEFAULT = 0x24
REMAP_X_Y = 0x21
REMAP_Y_Z = 0x18
REMAP_Z_X = 0x06
REMAP_X_Y_Z_TYPE0 = 0x12
REMAP_X_Y_Z_TYPE1 = 0x09
### AXIS SIG
SIGN_DEFAULT = 0x00	
Z_REV = 0x01			
YZ_REV = 0x03	
### data
GYRO_X_LSB = 0x14
EUL_Heading_LSB = 0x1a
QUA_W_LSB = 0x20
ACC_X_LSB = 0x28
### offset 
ACC_OFF_X_LSB = 0x55
ACC_OFF_X_MSB = 0x56
ACC_OFF_Y_LSB = 0x57
ACC_OFF_Y_MSB = 0x58
ACC_OFF_Z_LSB = 0x59
ACC_OFF_Z_MSB = 0x5a
MAG_OFF_X_LSB = 0x5b
MAG_OFF_X_MSB = 0x5c
MAG_OFF_Y_LSB = 0x5d
MAG_OFF_Y_MSB = 0x5e
MAG_OFF_Z_LSB = 0x5f
MAG_OFF_Z_MSB = 0x60
GYR_OFF_X_LSB = 0x61
GYR_OFF_X_MSB = 0x62
GYR_OFF_Y_LSB = 0x63
GYR_OFF_Y_MSB = 0x64
GYR_OFF_Z_LSB = 0x65
GYR_OFF_Z_MSB = 0x66
ACC_RAD_LSB = 0x67
ACC_RAD_MSB = 0x68
MAG_RAD_LSB = 0x69
MAG_RAD_MSB = 0x6a

class JMOAB_BNO055(Node):

	def __init__(self):

		super().__init__('jmoab_ros2_bno055')
		self.get_logger().info('Start JMOAB BNO055 node')

		self.declare_parameter('show_log', False)
		self.add_on_set_parameters_callback(self.parameter_callback)
		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("show_log: {}".format(self.show_log))


		self.bus = SMBus(1)

		### Get ros2 pacakage path ###
		pkg_path = get_package_share_directory('jmoab_ros2')
		pkg_path_list = pkg_path.split("/")
		scripts_path = os.path.join(("/"+pkg_path_list[1]),pkg_path_list[2],pkg_path_list[3],\
									"src", pkg_path_list[5], "scripts")
		calib_file_dir = os.path.join(scripts_path, "calibration_offset.txt")
		#### get calibration offset
		self.pre_calib = []
		self.get_logger().info("Read jmoab_ros2/scripts/calibration_offset.txt")
		if os.path.exists(calib_file_dir):
			with open(calib_file_dir, "r") as f:
				for line in f:
					self.pre_calib.append(int(line.strip()))
		else:
			self.get_logger().info("There is no calibration_offset.txt file!")
			self.get_logger().info("Do the calibration step first")
			quit()

		### Imu setup ###
		self.imu_int()

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
		self.last_log_stamp = time.time()


		### Kalman filter of hdg_offset estimate ###
		self.hdg_off_est = 0.0 # first guess
		self.state_predict = self.hdg_off_est
		self.error_est = 10.0 
		self.error_mea = 8.0 # a variance of

		### Pub/Sub ###
		self.ahrs_pub = self.create_publisher(Float32MultiArray, 'jmoab/ahrs', 10)
		self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

		self.ahrs_msg = Float32MultiArray()
		self.imu_msg = Imu()

		self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
		self.sbus_rc_ch_sub = self.create_subscription(Int16MultiArray, 'jmoab/sbus_rc_ch', self.sbus_rc_callback, 10)
		self.cart_mode_sub = self.create_subscription(UInt8, 'jmoab/cart_mode', self.cart_mode_callback, 10)
		# self.cart_cmd_sub = self.create_subscription(UInt16MultiArray, 'jmoab/cart_cmd', self.cart_cmd_callback, 10)
		
		self.get_logger().info('Publishing to /jmoab/ahrs [std_msgs/msg/Float32MultiArray]')
		self.get_logger().info('Publishing to /imu/data [sensor_msgs/msg/Imu]')

		### Loop spin ###
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)

	#######################################	
	############ ROS callbacks ############
	#######################################	
	# def cart_cmd_callback(self, msg):
	# 	self.sbus_cmd_steering = msg.data[0]
	# 	self.sbus_cmd_throttle = msg.data[1]

	def parameter_callback(self, params):
		for param in params:
			# print(param.name, param.type_)
			if (param.name == 'show_log') and (param.type_ == Parameter.Type.BOOL):
				self.show_log = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)	

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



	############################################	
	######## BNO055 SMBus2 read/write ##########
	############################################

	def imu_int(self):

		## put in reset, to reset all data in registers
		self.bus.write_byte_data(IMU_ADDR, SYS_TRIGGER, RST_SYS)
		time.sleep(1.0)	 ## give it sometime to reset/restart again

		## change to operating mode
		self.bus.write_byte_data(IMU_ADDR, OPR_REG, CONFIG_MODE)
		time.sleep(0.05)	# 7ms for operating mode

		## comment this if no need config
		# self.config_axis_remap()
		self.config_axis_sign(SIGN_DEFAULT)

		self.write_pre_calib()

		## change to operating mode
		# self.bus.write_byte_data(IMU_ADDR, OPR_REG, NDOF_MODE)
		self.bus.write_byte_data(IMU_ADDR, OPR_REG, IMU_MODE)
		time.sleep(0.05)	# 7ms for operating mode

	def write_pre_calib(self):
		timeSleep = 0.05
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_X_LSB, self.pre_calib[0])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_X_MSB, self.pre_calib[1])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Y_LSB, self.pre_calib[2])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Y_MSB, self.pre_calib[3])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Z_LSB, self.pre_calib[4])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Z_MSB, self.pre_calib[5])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_X_LSB, self.pre_calib[6])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_X_MSB, self.pre_calib[7])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Y_LSB, self.pre_calib[8])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Y_MSB, self.pre_calib[9])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Z_LSB, self.pre_calib[10])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Z_MSB, self.pre_calib[11])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_X_LSB, self.pre_calib[12])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_X_MSB, self.pre_calib[13])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Y_LSB, self.pre_calib[14])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Y_MSB, self.pre_calib[15])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Z_LSB, self.pre_calib[16])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Z_MSB, self.pre_calib[17])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_RAD_LSB, self.pre_calib[18])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_RAD_MSB, self.pre_calib[19])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_RAD_LSB, self.pre_calib[20])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_RAD_MSB, self.pre_calib[21])
		time.sleep(timeSleep)

	def config_axis_sign(self, SIGN):
		self.bus.write_byte_data(IMU_ADDR, AXIS_MAP_SIGN_REG, SIGN)
		time.sleep(0.1)	# 19ms from any mode to config mode

	def config_remap(self, REMAP):
		self.bus.write_byte_data(IMU_ADDR, AXIS_MAP_CONFIG_REG, REMAP)
		time.sleep(0.1)	# 19ms from any mode to config mode

	def read_euler_angles(self):
		raw = self.bus.read_i2c_block_data(IMU_ADDR, EUL_Heading_LSB, 6)
		yaw,roll,pitch = struct.unpack('<hhh', bytearray(raw))
		yaw = yaw/16.0
		roll = roll/16.0
		pitch = pitch/16.0

		return [roll, pitch, yaw]

	def read_quaternions(self):
		raw_quat = self.bus.read_i2c_block_data(IMU_ADDR, QUA_W_LSB, 8)
		qw,qx,qy,qz = struct.unpack('<hhhh', bytearray(raw_quat))
		qw = qw/16384.0
		qx = qx/16384.0
		qy = qy/16384.0
		qz = qz/16384.0

		return qx,qy,qz,qw

	def raed_gyro(self):
		raw_gyro = self.bus.read_i2c_block_data(IMU_ADDR, GYRO_X_LSB, 6)
		gx,gy,gz = struct.unpack('<hhh', bytearray(raw_gyro))
		gx = np.radians(gx/16.0)
		gy = np.radians(gy/16.0)
		gz = np.radians(gz/16.0)

		return gx,gy,gz

	def read_accel(self):
		raw_acc = self.bus.read_i2c_block_data(IMU_ADDR, ACC_X_LSB, 6)
		ax,ay,az = struct.unpack('<hhh', bytearray(raw_acc))
		ax = ax/100.0
		ay = ay/100.0
		az = az/100.0

		return ax,ay,az


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

		startTime = time.time()
		#############################
		### Read data from BNO055 ###
		#############################
		ahrs = self.read_euler_angles()
		quat = self.read_quaternions()
		gyro = self.raed_gyro()
		accl = self.read_accel()

		##########################################
		### Kalman filtering on yaw or heading ###
		##########################################
		self.pure_hdg = self.ConvertTo360Range(ahrs[2])
		hdg = self.ConvertTo360Range(self.pure_hdg + (self.sign)*self.hdg_off_est)
		#### must be rtk-fixed, to get precise position
		if self.fix_stat != 0:

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

		##################################
		### Contruct imu_msg/ ahrs_msg ###
		##################################
		self.imu_msg.header.stamp = self.get_clock().now().to_msg()
		self.imu_msg.header.frame_id = "imu_link"
		self.imu_msg.orientation.x = quat[0] #roll
		self.imu_msg.orientation.y = quat[1] #pitch
		self.imu_msg.orientation.z = quat[2] #yaw
		self.imu_msg.orientation.w = quat[3] 
		self.imu_msg.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

		self.imu_msg.angular_velocity.x = gyro[0]
		self.imu_msg.angular_velocity.y = gyro[1]
		self.imu_msg.angular_velocity.z = gyro[2]
		self.imu_msg.angular_velocity_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

		self.imu_msg.linear_acceleration.x = accl[0]
		self.imu_msg.linear_acceleration.y = accl[1]
		self.imu_msg.linear_acceleration.z = accl[2]
		self.imu_msg.linear_acceleration_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]


		self.ahrs_msg.data = [ahrs[0], ahrs[1], hdg]

		self.imu_pub.publish(self.imu_msg)
		self.ahrs_pub.publish(self.ahrs_msg)

		if ((time.time() - self.last_log_stamp) > 1.0) and self.show_log:
			self.get_logger().info("p_hdg: {:.2f} hdg_off_est: {:.2f} brg: {:.2f} hdg: {:.2f} KF_cal: {}".format(\
				self.pure_hdg, self.hdg_off_est, self.brg, hdg, self.cal_offset))
			self.last_log_stamp = time.time()

		period = time.time() - startTime
		if period < 0.007:
			sleepMore = 0.007 - period
			time.sleep(sleepMore)


def main(args=None):

	rclpy.init(args=args)
	node = JMOAB_BNO055()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":

	main()
