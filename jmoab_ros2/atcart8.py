
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, UInt8, Float32MultiArray, Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import time
import numpy as np
from smbus2 import SMBus
from zlac8015d import ZLAC8015D

## i2c's Address
JMOAB_RELAY_ADDR = 0x70
JMOAB_MAIN_ADDR = 0x71

## i2c's register
RY1 = 0x00
ADC_AC0 = 0x00
PWM1_H = 0x06
IN_SBUS_CH1_H = 0x0C
CARTIF_SBUS_CH1_H = 0x30
CARTIF_INPUT_SELECT = 0x52
AUX_LED1 = 0x54
DISABLE_SBUS_FAILSAFE = 0x57

ZLAC_VEL_MODE = 3
ZLAC_POS_MODE = 1


class JMOAB_ATCART8(Node):

	def __init__(self):

		super().__init__('jmoab_ros2_atcart8')
		self.get_logger().info('Start JMOAB ROS2 node')

		#####################
		### SMBus2 JMOAB ####
		#####################
		self.bus = SMBus(1)
		#### cart_cmd
		##### mixing for steering/throttle control
		self.cmd_sbus_steering = 1024
		self.cmd_sbus_throttle = 1024
		self.cart_sbus_cmd_cb_timeout = 1.0 # second
		self.cart_sbus_cmd_cb_timestamp = time.time()
		self.cart_sbus_cmd_flag = False
		self.prev_y = 0.0
		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0
		self.sbus_max_DB = self.sbus_mid + 1.0
		self.sbus_min_DB = self.sbus_mid - 1.0
		self.rev_str = False #False
		self.rev_thr = False

		##### left/right RPM velocity control
		self.cmd_rpm_left = 0.0
		self.cmd_rpm_right = 0.0
		self.cart_rpm_cmd_cb_timeout = 1.0
		self.cart_rpm_cmd_cb_timestamp = time.time()
		self.cart_rpm_cmd_flag = False

		##### left/right angle position control
		self.cmd_deg_left = 0.0
		self.cmd_deg_right = 0.0
		self.cart_deg_cmd_cb_timeout = 1.0
		self.cart_deg_cmd_cb_timestamp = time.time()
		self.cart_deg_cmd_flag = False

		##### left/right distance position control
		self.cmd_dist_left = 0.0
		self.cmd_dist_right = 0.0
		self.cart_dist_cmd_cb_timeout = 1.0
		self.cart_dist_cmd_cb_timestamp = time.time()
		self.cart_dist_cmd_flag = False

		##### cmd_vel
		self.vx_cmd = 0.0
		self.wz_cmd = 0.0
		self.cmd_vel_cb_timeout = 1.0
		self.cmd_vel_timestamp = time.time()
		self.cmd_vel_flag = False

		#### cart mode cmd
		self.cart_mode_cmd_flag = False
		self.cart_mode_cmd = 1

		#### relay cmd
		self.relay_cb_flag = False
		self.relay_list = 0

		#### servo cmd
		self.servo_cb_flag = False
		self.pwm_min = 1120
		self.pwm_max = 1920
		self.pwm_mid = 1520
		self.servo_list = []

		#### led cmd
		self.led_cb_flag = False
		self.led_list = []

		#################
		### ZLAC8015D ###
		#################
		self.zlc = ZLAC8015D.Controller(port="/dev/usb_rs485") ## must make udev rules first in order to provide dev like this
		self.zlac8015d_speed_mode_init()
		self.max_rpm = 50 #150
		self.ultimate_rpm = 200 
		self.deadband_rpm = 5
		self.zlac8015d_mode = 3
		self.l_meter = 0.0
		self.r_meter = 0.0
		self.travel_in_one_rev = self.zlc.travel_in_one_rev
		self.l_meter_init, self.r_meter_init = self.zlc.get_wheels_travelled()
		self.prev_l_meter = 0.0
		self.prev_r_meter = 0.0
		self.L = 0.485
		self.R_wheel = self.zlc.R_Wheel

		###############
		### Odom TF ###
		###############
		self.br = TransformBroadcaster(self)
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

		###############
		### Pub/Sub ###
		###############
		self.sbus_rc_pub = self.create_publisher(UInt16MultiArray, 'jmoab/sbus_rc_ch', 10)
		self.atcart_mode_pub = self.create_publisher(UInt8, 'jmoab/atcart_mode', 10)
		self.adc_pub = self.create_publisher(Float32MultiArray, 'jmoab/adc', 10)
		self.wheels_rpm_pub = self.create_publisher(Float32MultiArray, 'jmoab/wheels_rpm', 10)
		self.odom_pub = self.create_publisher(Odometry, 'atcart8/odom', 10)
		# self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

		self.sbus_rc_msg = UInt16MultiArray()
		self.atcart_mode_msg = UInt8()
		self.adc_msg = Float32MultiArray()
		self.wheels_rpm_msg = Float32MultiArray()
		self.odom_msg = Odometry()

		self.cart_sbus_cmd_sub = self.create_subscription(UInt16MultiArray, 'jmoab/cart_cmd', self.cart_sbus_cmd_callback, 10)
		self.cart_rpm_cmd_sub = self.create_subscription(Int16MultiArray, 'jmoab/cart_rpm_cmd', self.cart_rpm_cmd_callback, 10)
		self.cart_deg_cmd_sub = self.create_subscription(Int16MultiArray, 'jmoab/cart_deg_cmd', self.cart_deg_cmd_callback, 10)
		self.cart_dist_cmd_sub = self.create_subscription(Float32MultiArray, 'jmoab/cart_dist_cmd', self.cart_dist_cmd_callback, 10)
		self.atcart_mode_cmd_sub = self.create_subscription(UInt8, 'jmoab/atcart_mode_cmd', self.atcart_mode_cmd_callback, 10)
		self.relay_sub = self.create_subscription(UInt8MultiArray, 'jmoab/relay', self.relay_cmd_callback, 10)
		self.servo_sub = self.create_subscription(UInt16MultiArray, 'jmoab/servo', self.servo_cmd_callback, 10)
		self.led_sub = self.create_subscription(UInt8MultiArray, 'jmoab/led', self.led_cmd_callback, 10)
		self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

		self.get_logger().info('Publishing to jmoab/sbus_rc_ch [std_msgs/msg/UInt16MultiArray]')
		self.get_logger().info('Publishing to jmoab/atcart_mode [std_msgs/msg/UInt8]')
		self.get_logger().info('Publishing to jmoab/adc [std_msgs/msg/Float32MultiArray]')
		self.get_logger().info('Subscribing on jmoab/atcart_mode_cmd [std_msgs/msg/UInt8]')
		self.get_logger().info('Subscribing on jmoab/relay [std_msgs/msg/UInt8MultiArray]')
		self.get_logger().info('Subscribing on jmoab/servo [std_msgs/msg/UInt16MultiArry]')
		self.get_logger().info('Subscribing on jmoab/led [std_msgs/msg/UInt8MultiArray]')
		self.get_logger().info('There are five cart command types in auto mode')
		self.get_logger().info('1. jmoab/cart_cmd [std_msgs/msg/UInt16MultiArray] | continuously')
		self.get_logger().info('   ex. [sbus_steering, sbus_throttle] 368-1024-1680 as sbus range')
		self.get_logger().info('2. jmoab/cart_rpm_cmd [std_msgs/msg/Int16MultiArray] | continuously')
		self.get_logger().info('   ex. [rpm_left, rpm_right], -150.0 (rev) to 150.0 (fwd)')
		self.get_logger().info('3. jmoab/cart_deg_cmd [std_msgs/msg/Int16MultiArray] | one at a time')
		self.get_logger().info('   ex. [deg_left, deg_right], -94M deg to 94M deg')
		self.get_logger().info('4. jmoab/cart_dist_cmd [std_msgs/msg/Float32MultiArray] | one at a time')
		self.get_logger().info('   ex. [meters_left, meters_right], -171km to 171km')
		self.get_logger().info('5. cmd_vel [geometry_msgs/msg/Twist] | continuously')
		self.get_logger().info('   ex. this is general cmd_vel to use with ROS navigation stack')
		self.get_logger().info('Command types can be switched anytime depends on user publisher')
		
		#################
		### Loop spin ###
		#################
		timer_period = 0.01
		self.period = timer_period
		self.timer = self.create_timer(timer_period, self.timer_callback)

		#################
		### ROS Param ###
		#################
		# self.declare_parameters(
		# 	namespace='',
		# 	parameters=[
		# 		('base_width', self.L),
		# 		('max_rpm', self.max_rpm)
		# 	]
		# )
		self.declare_parameter('base_width')
		self.declare_parameter('max_rpm')
		param_base_width = Parameter('base_width', Parameter.Type.DOUBLE, self.L)
		param_max_rpm = Parameter('max_rpm', Parameter.Type.INTEGER, self.max_rpm)
		self.set_parameters([param_base_width, param_max_rpm])
		self.get_logger().info('ATCART8 Parameters')
		self.get_logger().info(f'base_width: {self.L}')
		self.get_logger().info(f'max_rpm: {self.max_rpm}')
		self.add_on_set_parameters_callback(self.parameter_callback)


	#######################################	
	############ ROS callbacks ############
	#######################################	
	def cart_sbus_cmd_callback(self, msg):
		if len(msg.data) > 0:
			if (not self.cart_rpm_cmd_flag) and (not self.cart_deg_cmd_flag) and (not self.cart_dist_cmd_flag) and (not self.cmd_vel_flag):
				self.cmd_sbus_steering = msg.data[0]
				self.cmd_sbus_throttle = msg.data[1]
				self.cart_sbus_cmd_cb_timestamp = time.time()
				self.cart_sbus_cmd_flag = True
			else:
				self.get_logger().warning("sbus_cmd: Try not to publish others cart command type in the same time")

	def cart_rpm_cmd_callback(self, msg):
		if len(msg.data) > 0:
			if (not self.cart_sbus_cmd_flag) and (not self.cart_deg_cmd_flag) and (not self.cart_dist_cmd_flag) and (not self.cmd_vel_flag):
				self.cmd_rpm_left = msg.data[0]
				self.cmd_rpm_right = -msg.data[1]
				self.cart_rpm_cmd_cb_timestamp = time.time()
				self.cart_rpm_cmd_flag = True
			else:
				self.get_logger().warning("rpm_cmd: Try not to publish others cart command type in the same time")

	def cart_deg_cmd_callback(self, msg):
		if len(msg.data) > 0:
			if (not self.cart_sbus_cmd_flag) and (not self.cart_rpm_cmd_flag) and (not self.cart_dist_cmd_flag) and (not self.cmd_vel_flag):
				self.cmd_deg_left = msg.data[0]
				self.cmd_deg_right = -msg.data[1]
				self.cart_deg_cmd_cb_timestamp = time.time()
				self.cart_deg_cmd_flag = True
			else:
				self.get_logger().warning("deg_cmd: Try not to publish others cart command type in the same time")

	def cart_dist_cmd_callback(self, msg):
		if len(msg.data) > 0:
			if (not self.cart_sbus_cmd_flag) and (not self.cart_rpm_cmd_flag) and (not self.cart_deg_cmd_flag) and (not self.cmd_vel_flag):
				self.cmd_dist_left = msg.data[0]
				self.cmd_dist_right = -msg.data[1]
				self.cart_dist_cmd_cb_timestamp = time.time()
				self.cart_dist_cmd_flag = True
			else:
				self.get_logger().warning("dist_cmd: Try not to publish others cart command type in the same time")

	def cmd_vel_callback(self, msg):
		if (not self.cart_sbus_cmd_flag) and (not self.cart_rpm_cmd_flag) and (not self.cart_deg_cmd_flag) and (not self.cart_dist_cmd_flag):
			self.vx_cmd = msg.linear.x
			self.wz_cmd = msg.angular.z
			self.cmd_vel_timestamp = time.time()
			self.cmd_vel_flag = True
		else:
			self.get_logger().warning("cmd_vel: Try not to publish others cart command type in the same time")

	def atcart_mode_cmd_callback(self, msg):
		self.cart_mode_cmd_flag = True
		self.cart_mode_cmd = msg.data

	def relay_cmd_callback(self, msg):
		self.relay_cb_flag = True
		self.relay_list = msg.data

	def servo_cmd_callback(self, msg):
		self.servo_cb_flag = True
		self.servo_list = msg.data

	def led_cmd_callback(self, msg):
		self.led_cb_flag = True
		self.led_list = msg.data

	def parameter_callback(self, params):
		for param in params:
			if param.name == 'base_width' and param.type_ == Parameter.Type.DOUBLE:
				self.L = param.value
				self.get_logger().info(f'base_width (L) has updated to {self.L}')
			elif param.name == 'max_rpm' and param.type_ == Parameter.Type.INTEGER:
				self.max_rpm = param.value
				self.get_logger().info(f'max_rpm has updated to {self.max_rpm}')
		return SetParametersResult(successful=True)

	######################################	
	##### ZLAC8015D Helper Functions #####
	######################################	

	def zlac8015d_speed_mode_init(self):
		self.zlc.disable_motor()
		self.zlc.set_mode(3)
		## we want responsive reaction, so accel/decel should be 0
		self.zlc.set_accel_time(0,0)
		self.zlc.set_decel_time(0,0)
		self.zlc.enable_motor()
		time.sleep(0.1)
		self.get_logger().info('Set ZLAC8015D to Velocity Control')

	def zlac8015d_position_mode_init(self):
		self.zlc.disable_motor()
		self.zlc.set_mode(1)
		## I don't recommend to change this accel/decel other than 0
		## because there is some issue where switch from pos to vel control
		## and the accel/decel still not changed in manual (vel) control
		## so it causes some laggy response on the RC propo. 
		self.zlc.set_accel_time(0,0)
		self.zlc.set_decel_time(0,0)
		self.zlc.set_position_async_control()
		self.zlc.set_maxRPM_pos(30,30)
		self.zlc.enable_motor()
		time.sleep(0.1)
		self.get_logger().info('Set ZLAC8015D to Position Control')

	def zlac8015d_set_rpm_with_limit(self, left_rpm, right_rpm):
		if (-self.ultimate_rpm < left_rpm < self.ultimate_rpm) and (-self.ultimate_rpm < right_rpm < self.ultimate_rpm): 
			# print(left_rpm, right_rpm)
			if (-self.deadband_rpm < left_rpm < self.deadband_rpm): 
				left_rpm = 0

			if (-self.deadband_rpm < right_rpm < self.deadband_rpm): 
				right_rpm = 0

			self.zlc.set_rpm(int(left_rpm), int(right_rpm))

	def check_zlac8015d_mode(self, desired_mode):

		if self.zlac8015d_mode != desired_mode:
			print("GET MODE--------------")
			_mode = self.zlc.get_mode()
			if _mode == desired_mode:
				return
			else:
				if desired_mode == 1:
					self.zlac8015d_position_mode_init()
					self.zlac8015d_mode = 1
					print("changed to position mode")
					return
				elif desired_mode == 3:
					self.zlac8015d_speed_mode_init()
					self.zlac8015d_mode = 3
					print("changed to velocity mode")
					return
		else:
			return


	#######################################	
	################ Maths ################
	#######################################	

	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def convert2voltage(self, raw_list):
	
		raw_list = np.asarray(raw_list)
		voltage_array =  self.map(raw_list, 0.0, 255.0, 0.0, 40.96)
		return voltage_array.tolist()

	def linear_to_rpm(self, v):
		rpm = (60.0/(2.0*np.pi))*(v/self.R_wheel)
		return rpm

	#######################################	
	######### SMBus2 read/write ###########
	#######################################

	def bypass_sbus_failsafe(self):
		## Disable SBUS Failsafe
		self.bus.write_byte_data(JMOAB_MAIN_ADDR, DISABLE_SBUS_FAILSAFE, 0x01)
		time.sleep(0.1)

		## Set back to hold mode
		self.write_atcart_mode(0x00)
		time.sleep(0.1)

		## Set back to manual mode
		self.write_atcart_mode(0x01)
		time.sleep(0.1)
		self.write_atcart_mode(0x01)
		time.sleep(0.1)
		self.write_atcart_mode(0x01)
		time.sleep(0.1)

	def read_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(JMOAB_MAIN_ADDR, IN_SBUS_CH1_H, 20)

		SBUS_ch = [None]*10
		for i in range(10):
			hibyte = input_SBUS[i*2] 
			lobyte = input_SBUS[(2*i)+1]
			# needs only 3 bits from hibyte
			SBUS_ch[i] = ((lobyte & 0xFF) | ((hibyte&0x07) << 8))

		return SBUS_ch

	def read_atcart_mode(self):
		return self.bus.read_byte_data(JMOAB_MAIN_ADDR, CARTIF_INPUT_SELECT)

	def read_adc(self):
		raw = self.bus.read_i2c_block_data(JMOAB_MAIN_ADDR, 0x00, 6)
		voltage_list = self.convert2voltage(raw)
		return voltage_list

	def write_atcart_mode(self, mode_num):
		## need to write multiple times to take effect
		## publisher side only sends one time is ok
		for i in range(10):
			self.bus.write_byte_data(JMOAB_MAIN_ADDR, CARTIF_INPUT_SELECT, mode_num)
		# self.bus.write_byte_data(JMOAB_MAIN_ADDR, CARTIF_INPUT_SELECT, mode_num)

	def write_relay(self, relay_list):
		all_bytes = [relay_list[0], relay_list[1]]
		self.bus.write_i2c_block_data(JMOAB_RELAY_ADDR, RY1, all_bytes)

	def write_servo(self, servo_list):
		pwm1_bytes = self.pwm2word(servo_list[0])
		pwm2_bytes = self.pwm2word(servo_list[1])
		pwm3_bytes = self.pwm2word(servo_list[2])
		all_bytes = pwm1_bytes + pwm2_bytes + pwm3_bytes
		self.bus.write_i2c_block_data(JMOAB_MAIN_ADDR, PWM1_H, all_bytes)

	def write_led(self, led_list):
		all_bytes = [led_list[0], led_list[1], led_list[2]]
		self.bus.write_i2c_block_data(JMOAB_MAIN_ADDR, AUX_LED1, all_bytes)

	#######################################	
	############ Cart control #############
	#######################################

	def xy_mixing(self, x, y):
		## x, y must be in the range of -100 to 100

		left = y+x
		right = y-x
		diff = abs(x) - abs(y)

		if (left < 0.0):
			left = left - abs(diff)
		else:
			left = left + abs(diff)

		if (right < 0.0):
			right = right - abs(diff)
		else:
			right = right + abs(diff)

		if (self.prev_y < 0.0):
			swap = left
			left = right
			right = swap
			# print("swap")
		
		self.prev_y = y

		## left and right are in -200 to 200 ranges

		return left, right

	def sbus2percent(self, sbus):
		if sbus >= self.sbus_max_DB:
			percent = self.map(sbus, self.sbus_max_DB, self.sbus_max, 0.0, 100.0)
			if percent > 100.0:
				percent = 100.0
		elif sbus <= self.sbus_min_DB:
			percent = self.map(sbus, self.sbus_min, self.sbus_min_DB, -100.0, 0.0)
			if percent < -100.0:
				percent = -100.0
		else:
			percent = 0

		return percent

	def sbusCmds_to_RPMs(self, sbus_steering, sbus_throttle):
		# x_percent = self.sbus2percent(sbus_steering)
		# y_percent = self.sbus2percent(sbus_throttle)
		if self.rev_thr:
			y = self.map(sbus_throttle, self.sbus_min, self.sbus_max, 100.0, -100.0)
		else:
			y = self.map(sbus_throttle, self.sbus_min, self.sbus_max, -100.0, 100.0)

		if self.rev_str:
			x = self.map(sbus_steering, self.sbus_min, self.sbus_max, 100.0, -100.0)
		else:
			x = self.map(sbus_steering, self.sbus_min, self.sbus_max, -100.0, 100.0)

		left_percent, right_percent = self.xy_mixing(x, y)
		left_rpm = self.map(left_percent, -200.0, 200.0, -self.max_rpm, self.max_rpm)
		right_rpm = self.map(right_percent, -200.0, 200.0, self.max_rpm, -self.max_rpm)

		return left_rpm, right_rpm

	def sbus_db_check(self, sbus):

		if 1014 < sbus < 1034:
			return 1024
		else:
			return sbus

	#######################################	
	######## ROS2 timer callback ##########
	#######################################

	def timer_callback(self):

		start_time = time.time()

		#######################
		### data from JMOAB ###
		#######################
		self.adc_msg.data = self.read_adc()
		sbus_ch_array = self.read_sbus_channel()
		self.sbus_rc_msg.data = sbus_ch_array
		atcart_mode = self.read_atcart_mode()
		self.atcart_mode_msg.data = atcart_mode
		
		self.adc_pub.publish(self.adc_msg)
		self.sbus_rc_pub.publish(self.sbus_rc_msg)
		self.atcart_mode_pub.publish(self.atcart_mode_msg)

		############################
		### cart control command ###
		############################
		#### Auto mode ####
		if atcart_mode == 2:

			###################################
			### SBUS command mixing control ###
			###################################
			if self.cart_sbus_cmd_flag:
				## confirm the mode to be in velocity control
				self.check_zlac8015d_mode(ZLAC_VEL_MODE)

				if (time.time() - self.cart_sbus_cmd_cb_timestamp) < self.cart_sbus_cmd_cb_timeout:
					left_rpm, right_rpm = self.sbusCmds_to_RPMs(self.cmd_sbus_steering, self.cmd_sbus_throttle)
				else:
					left_rpm = 0.0
					right_rpm = 0.0
					self.cart_sbus_cmd_flag = False

				self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

				## set other flags to False if there is other command trying to come in the same time
				self.cart_rpm_cmd_flag = False
				self.cart_deg_cmd_flag = False
				self.cart_dist_cmd_flag = False
				self.cmd_vel_flag = False

				## get feedback rpm from ZLAC8015D
				fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
				self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)

			####################################
			### RPM command velocity control ###
			####################################
			elif self.cart_rpm_cmd_flag:
				## confirm the mode to be in velocity control
				self.check_zlac8015d_mode(ZLAC_VEL_MODE)

				if (time.time() - self.cart_rpm_cmd_cb_timestamp) < self.cart_rpm_cmd_cb_timeout:
					left_rpm = self.cmd_rpm_left
					right_rpm = self.cmd_rpm_right
				else:
					left_rpm = 0.0
					right_rpm = 0.0
					self.cart_rpm_cmd_flag = False

				self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

				## set other flags to False if there is other command trying to come in the same time
				self.cart_sbus_cmd_flag = False
				self.cart_deg_cmd_flag = False
				self.cart_dist_cmd_flag = False
				self.cmd_vel_flag = False

				## get feedback rpm from ZLAC8015D
				fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
				self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)

			####################################
			## cmd_vel for navaigation stack ###
			####################################
			elif self.cmd_vel_flag:
				
				## confirm the mode to be in velocity control
				self.check_zlac8015d_mode(ZLAC_VEL_MODE)

				if (time.time() - self.cmd_vel_timestamp) < self.cmd_vel_cb_timeout:
					## straight line
					if (self.vx_cmd != 0.0) and (self.wz_cmd == 0.0):
						vl_cmd = self.vx_cmd/2.0
						# vr_cmd = self.vx_cmd/2.0
						left_rpm = self.linear_to_rpm(vl_cmd)
						right_rpm = -left_rpm
					## skidding in-place
					elif (self.vx_cmd == 0.0) and (self.wz_cmd != 0.0):
						vl_cmd = -self.wz_cmd*(self.L/2.0)
						vr_cmd = -self.wz_cmd*(self.L/2.0)
						left_rpm = self.linear_to_rpm(vl_cmd)
						right_rpm = self.linear_to_rpm(vr_cmd)
					## curve left
					elif (self.vx_cmd != 0.0) and (self.wz_cmd != 0.0):
						R_icc = abs(self.vx_cmd)/abs(self.wz_cmd)
						sign_vx = self.vx_cmd/abs(self.vx_cmd)
						if self.wz_cmd > 0.0:
							print("curve left")
							vl_cmd = (sign_vx)*(self.wz_cmd*(R_icc - self.L/2.0))/2.0
							vr_cmd = (sign_vx)*(self.wz_cmd*(R_icc + self.L/2.0))/2.0
						elif self.wz_cmd < 0.0:
							print("curve right")
							vl_cmd = (sign_vx)*(abs(self.wz_cmd)*(R_icc + self.L/2.0))/2.0
							vr_cmd = (sign_vx)*(abs(self.wz_cmd)*(R_icc - self.L/2.0))/2.0
						left_rpm = self.linear_to_rpm(vl_cmd)
						right_rpm = self.linear_to_rpm(-vr_cmd)
					else:
						left_rpm = 0.0
						right_rpm = 0.0
				else:
					left_rpm = 0.0
					right_rpm = 0.0
					self.cmd_vel_flag = False

				self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

				## set other flags to False if there is other command trying to come in the same time
				self.cart_sbus_cmd_flag = False
				self.cart_rpm_cmd_flag = False
				self.cart_deg_cmd_flag = False
				self.cart_dist_cmd_flag = False

				## get feedback rpm from ZLAC8015D
				fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
				self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)

			####################################
			### Deg command position control ###
			####################################
			elif self.cart_deg_cmd_flag:
				## confirm the mode to be in position control
				self.check_zlac8015d_mode(ZLAC_POS_MODE)
				self.zlc.set_relative_angle(self.cmd_deg_left, self.cmd_deg_right)
				self.zlc.move_left_wheel()
				self.zlc.move_right_wheel()
				self.cart_deg_cmd_flag = False
				left_rpm = 0.0
				right_rpm = 0.0
				## set other flags to False if there is other command trying to come in the same time
				self.cart_sbus_cmd_flag = False
				self.cart_rpm_cmd_flag = False
				self.cart_dist_cmd_flag = False
				self.cmd_vel_flag = False


			#####################################
			### Dist command position control ###
			#####################################
			elif self.cart_dist_cmd_flag:
				## confirm the mode to be in position control
				self.check_zlac8015d_mode(ZLAC_POS_MODE)
				left_cmd_deg = (self.cmd_dist_left*360.0)/self.travel_in_one_rev
				right_cmd_deg = (self.cmd_dist_right*360.0)/self.travel_in_one_rev
				self.zlc.set_relative_angle(left_cmd_deg, right_cmd_deg)
				self.zlc.move_left_wheel()
				self.zlc.move_right_wheel()
				self.cart_dist_cmd_flag = False
				left_rpm = 0.0
				right_rpm = 0.0
				## set other flags to False if there is other command trying to come in the same time
				self.cart_sbus_cmd_flag = False
				self.cart_rpm_cmd_flag = False
				self.cart_deg_cmd_flag = False
				self.cmd_vel_flag = False

			else:
				left_rpm = 0.0
				right_rpm = 0.0


		#### Manual mode ####
		elif atcart_mode == 1:
			## allow only velocity control in manual mode
			## confirm the mode to be in velocity control
			self.check_zlac8015d_mode(ZLAC_VEL_MODE)
			left_rpm, right_rpm = self.sbusCmds_to_RPMs(self.sbus_db_check(sbus_ch_array[0]), self.sbus_db_check(sbus_ch_array[1]))
			self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

			## get feedback rpm from ZLAC8015D
			fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
			self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
			self.wheels_rpm_pub.publish(self.wheels_rpm_msg)

		#### Hold mode ####
		else:
			self.check_zlac8015d_mode(ZLAC_VEL_MODE)
			left_rpm = 0.0
			right_rpm = 0.0
			self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

		###########################
		## Odometry computation ###
		###########################
		self.l_meter, self.r_meter = self.zlc.get_wheels_travelled()
		self.l_meter = self.l_meter - self.l_meter_init 
		self.r_meter = (-1*self.r_meter) - (-1*self.r_meter_init)

		if self.zlac8015d_mode == 1:
			vl = (self.l_meter-self.prev_l_meter)/self.period
			vr = (self.r_meter-self.prev_r_meter)/self.period
		elif self.zlac8015d_mode == 3:
			vl,vr = self.zlc.get_linear_velocities()

		vl = round(vl, 3)
		vr = round(vr, 3)
		if (vl > 0.0) and (vr < 0.0) and (abs(vl) == abs(vr)):
			## rotatiing CW
			V = -vl
			Wz = 2.0*V/self.L
			self.theta = self.theta + Wz*self.period

			path = "skid_right"

		elif (vr > 0.0) and (vl < 0.0) and (abs(vl) == abs(vr)):
			## rotatiing CCW
			V = vr
			Wz = 2.0*V/self.L
			self.theta = self.theta + Wz*self.period

			path = "skid_left"

		elif abs(vl) > abs(vr):
			## curving CW
			V = (vl + vr)/2.0
			Wz = (vl-vr)/self.L
			R_ICC = (self.L/2.0)*((vl+vr)/(vl-vr))

			self.x = self.x - R_ICC*np.sin(self.theta) + R_ICC*np.sin(self.theta + Wz*self.period)
			self.y = self.y + R_ICC*np.cos(self.theta) - R_ICC*np.cos(self.theta + Wz*self.period)
			self.theta = self.theta - Wz*self.period

			path = "curve_right"

		elif abs(vl) < abs(vr):
			## curving CCW
			V = (vl + vr)/2.0
			Wz = (vr-vl)/self.L
			R_ICC = (self.L/2.0)*((vr+vl)/(vr-vl))

			self.x = self.x - R_ICC*np.sin(self.theta) + R_ICC*np.sin(self.theta + Wz*self.period)
			self.y = self.y + R_ICC*np.cos(self.theta) - R_ICC*np.cos(self.theta + Wz*self.period)
			self.theta = self.theta + Wz*self.period

			path = "curve_left"

		elif vl == vr:
			V = (vl + vr)/2.0
			Wz = 0.0
			self.x = self.x + V*np.cos(self.theta)*self.period
			self.y = self.y + V*np.sin(self.theta)*self.period
			self.theta = self.theta
			path = "straight"

		else:
			V = 0.0
			Wz = 0.0
			R_ICC = 0.0
			
		q = quaternion_from_euler(0,0, self.theta)
		## construct tf
		# t = TransformStamped()
		# t.header.frame_id = "odom" 
		# t.header.stamp = self.get_clock().now().to_msg()
		# t.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
		# t.transform.translation.x = self.x
		# t.transform.translation.y = self.y
		# t.transform.translation.z = 0.0

		# t.transform.rotation.x = q[0]
		# t.transform.rotation.y = q[1]
		# t.transform.rotation.z = q[2]
		# t.transform.rotation.w = q[3]
		# self.br.sendTransform(t)

		self.odom_msg.header.stamp = self.get_clock().now().to_msg()
		self.odom_msg.header.frame_id = "odom"
		self.odom_msg.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
		self.odom_msg.pose.pose.position.x = self.x
		self.odom_msg.pose.pose.position.y = self.y
		self.odom_msg.pose.pose.position.z = 0.0
		self.odom_msg.pose.pose.orientation.x = q[0]
		self.odom_msg.pose.pose.orientation.y = q[1]
		self.odom_msg.pose.pose.orientation.z = q[2]
		self.odom_msg.pose.pose.orientation.w = q[3]
		self.odom_msg.pose.covariance[0] = 0.0001
		self.odom_msg.pose.covariance[7] = 0.0001
		self.odom_msg.pose.covariance[14] = 0.000001	#1e12
		self.odom_msg.pose.covariance[21] = 0.000001	#1e12
		self.odom_msg.pose.covariance[28] = 0.000001	#1e12
		self.odom_msg.pose.covariance[35] = 0.0001
		self.odom_msg.twist.twist.linear.x = V
		self.odom_msg.twist.twist.linear.y = 0.0
		self.odom_msg.twist.twist.angular.z = Wz
		self.odom_pub.publish(self.odom_msg)


		#########################
		### cart mode command ###
		#########################
		if self.cart_mode_cmd_flag:
			self.write_atcart_mode(self.cart_mode_cmd)
			self.cart_mode_cmd_flag = False

		#####################
		### relay command ###
		#####################
		if self.relay_cb_flag:
			self.write_relay(self.relay_list)
			self.relay_cb_flag = False

		#####################
		### servo command ###
		#####################
		if self.servo_cb_flag:
			self.write_servo(self.servo_list)
			self.servo_cb_flag = False

		###################
		### led command ###
		###################
		if self.led_cb_flag:
			self.write_led(self.led_list)
			self.led_cb_flag = False	

		# print("period: {:.4f} | cart_mode: {:d} | zlac_mode: {:d} | rpmL: {:.2f} | rpmR: {:.2f}".format(\
		# 	self.period, atcart_mode, self.zlac8015d_mode, left_rpm, right_rpm))

		self.period = time.time() - start_time
		self.prev_l_meter = self.l_meter
		self.prev_r_meter = self.r_meter


def main(args=None):

	rclpy.init(args=None)
	node = JMOAB_ATCART8()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
