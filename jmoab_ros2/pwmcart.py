
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int16MultiArray, UInt8, Float32MultiArray, Int16
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import time
import numpy as np
from smbus2 import SMBus

## i2c's Address
JMOAB_I2C_ADDR1 = 0x70
JMOAB_I2C_ADDR2 = 0x71

## i2c's register
RY1 = 0x00
ADC_AC0 = 0x00
PWM1_H = 0x06
IN_SBUS_CH1_H = 0x0C
CARTIF_SBUS_CH1_H = 0x30
CARTIF_INPUT_SELECT = 0x52
AUX_LED1 = 0x54
DISABLE_SBUS_FAILSAFE = 0x57

HOLD = 0x00
MAN = 0x01
AUTO = 0x02

ENA_FS = 0x00
DIS_FS = 0x01


class JMOAB_ATCART_BASIC(Node):

	def __init__(self):

		super().__init__('jmoab_ros2_pwmcart')
		self.get_logger().info('Start JMOAB ROS2 node')

		self.bus = SMBus(1)

		##### ROS parameters
		# self.declare_parameter('sbus_left_max_db', 1142)
		# self.declare_parameter('sbus_right_max_db', 1151)
		# self.declare_parameter('sbus_left_min_db', 908)
		# self.declare_parameter('sbus_right_min_db', 902)
		
		self.declare_parameter('sbus_left_max_db', 1024)
		self.declare_parameter('sbus_right_max_db', 1024)
		self.declare_parameter('sbus_left_min_db', 1024)
		self.declare_parameter('sbus_right_min_db', 1024)

		self.declare_parameter('vx_max', 2.0)
		self.declare_parameter('wz_max', 2.0)

		self.add_on_set_parameters_callback(self.parameter_callback)

		self.sbus_left_max_db = self.get_parameter('sbus_left_max_db').get_parameter_value().integer_value
		self.sbus_right_max_db = self.get_parameter('sbus_right_max_db').get_parameter_value().integer_value
		self.sbus_left_min_db = self.get_parameter('sbus_left_min_db').get_parameter_value().integer_value
		self.sbus_right_min_db = self.get_parameter('sbus_right_min_db').get_parameter_value().integer_value
		self.vx_max = self.get_parameter('vx_max').get_parameter_value().double_value
		self.wz_max = self.get_parameter('wz_max').get_parameter_value().double_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("sbus_left_max_db: {}".format(self.sbus_left_max_db))
		self.get_logger().info("sbus_right_max_db: {}".format(self.sbus_right_max_db))
		self.get_logger().info("sbus_left_min_db: {}".format(self.sbus_left_min_db))
		self.get_logger().info("sbus_right_min_db: {}".format(self.sbus_right_min_db))
		self.get_logger().info("vx_max: {}".format(self.vx_max))
		self.get_logger().info("wz_max: {}".format(self.wz_max))

		### Class parameters ###
		#### cart_cmd
		##### mixing for steering/throttle control
		self.sbus_cmd_steering = 1024
		self.sbus_cmd_throttle = 1024
		self.prev_y = 0.0
		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0
		self.sbus_max_DB = self.sbus_mid + 1.0
		self.sbus_min_DB = self.sbus_mid - 1.0
		self.sbus_min_backward = 968	# a value before start rotating backward
		self.sbus_min_forward = 1093	# a value before start rotating forward

		# self.sbus_left_max_db = 1142
		# self.sbus_right_max_db = 1151

		# self.sbus_left_min_db = 908
		# self.sbus_right_min_db = 902

		self.last_stamp_log = time.time()

		### cmd_vel
		# self.vx_max = 2.0
		# self.wz_max = 2.0
		self.vx = 0.0
		self.wz = 0.0
		self.cmd_vel_cb_flag = False
		self.cmd_vel_timeout = 1.0
		self.cmd_vel_stamp = time.time()

		### wheels_cmd
		self.left_percent = 0.0
		self.right_percent = 0.0 
		self.wheels_cmd_cb_flag = False
		self.wheels_cmd_timeout = 1.0
		self.wheels_cmd_stamp = time.time()

		#### cart mode cmd
		self.cart_mode_cmd_flag = False
		self.cart_mode_cmd = 1

		#### relay cmd
		self.relay_cb_flag = False
		self.relay_list = []

		#### servo cmd
		self.servo_cb_flag = False
		self.pwm_min = 1120
		self.pwm_max = 1920
		self.pwm_mid = 1520
		self.servo_list = []
		self.servo_pwm = 1520

		#### led cmd
		self.led_cb_flag = False
		self.led_list = []

		### Pub/Sub ###
		self.sbus_rc_pub = self.create_publisher(Int16MultiArray, 'jmoab/sbus_rc_ch', 10)
		self.cart_mode_pub = self.create_publisher(UInt8, 'jmoab/cart_mode', 10)
		self.adc_pub = self.create_publisher(Float32MultiArray, 'jmoab/adc', 10)

		self.sbus_rc_msg = Int16MultiArray()
		self.cart_mode_msg = UInt8()
		self.adc_msg = Float32MultiArray()

		self.wheels_cmd_sub = self.create_subscription(Float32MultiArray, 'jmoab/wheels_cmd', self.wheels_cmd_callback, 10)
		self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
		self.cart_mode_cmd_sub = self.create_subscription(UInt8, 'jmoab/cart_mode_cmd', self.cart_mode_cmd_callback, 10)
		self.relay_sub = self.create_subscription(Int8MultiArray, 'jmoab/relays', self.relay_cmd_callback, 10)
		self.servo_sub = self.create_subscription(Int16MultiArray, 'jmoab/servos', self.servo_cmd_callback, 10)
		# self.led_sub = self.create_subscription(UInt8MultiArray, 'jmoab/led', self.led_cmd_callback, 10)

		self.get_logger().info('Publishing  to /jmoab/sbus_rc_ch [std_msgs/msg/Int16MultiArray]')
		self.get_logger().info('Publishing  to /jmoab/cart_mode [std_msgs/msg/UInt8]')
		self.get_logger().info('Publishing  to /jmoab/adc [std_msgs/msg/Float32MultiArray]')
		
		self.get_logger().info('Subscribing on /cmd_vel [geometry_msgs/msg/Twist] ex: linear.x angular.z')
		self.get_logger().info('Subscribing on /jmoab/wheels_cmd [std_msgs/msg/Float32MultiArray] ex: [30.0, -50.0] percentages -100.0 to 100.0 ranges')
		self.get_logger().info('Subscribing on /jmoab/cart_mode_cmd [std_msgs/msg/UInt8]  ex: 0=hold, 1=manual, 2=auto')
		self.get_logger().info('Subscribing on /jmoab/relays [std_msgs/msg/Int8MultiArray]  ex: [1,1] on both relays')
		self.get_logger().info('Subscribing on /jmoab/servo [std_msgs/msg/Int16]  ex: 1120-1520-1920 ranges ')
		# self.get_logger().info('Subscribing on /jmoab/led [std_msgs/msg/UInt8MultiArray]')

		### Uncomment this if want to turn off propo in auto mode ###
		# self.bypass_sbus_failsafe()

		### Loop spin ###
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.timer_callback)

	#######################################	
	############ ROS callbacks ############
	#######################################
	def parameter_callback(self, params):
		for param in params:
			# print(param.name, param.type_)
			if (param.name == 'sbus_left_max_db') and (param.type_ == Parameter.Type.INTEGER):
				self.sbus_left_max_db = param.value
			elif (param.name == 'sbus_right_max_db') and (param.type_ == Parameter.Type.INTEGER):
				self.sbus_right_max_db = param.value
			elif (param.name == 'sbus_left_min_db') and (param.type_ == Parameter.Type.INTEGER):
				self.sbus_left_min_db = param.value
			elif (param.name == 'sbus_right_min_db') and (param.type_ == Parameter.Type.INTEGER):
				self.sbus_right_min_db = param.value
			elif (param.name == 'vx_max') and (param.type_ == Parameter.Type.DOUBLE):
				self.vx_max = param.value
			elif (param.name == 'wz_max') and (param.type_ == Parameter.Type.DOUBLE):
				self.wz_max = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)

	def cmd_vel_callback(self, msg):
		if msg.linear.x > self.vx_max:
			vx = self.vx_max
		elif msg.linear.x < -self.vx_max:
			vx = -self.vx_max
		else:
			vx = msg.linear.x

		if msg.angular.z > self.wz_max:
			wz = self.wz_max
		elif msg.angular.z < -self.wz_max:
			wz = -self.wz_max
		else:
			wz = msg.angular.z

		self.vx = vx
		self.wz = wz

		self.cmd_vel_stamp = time.time()
		self.cmd_vel_cb_flag = True

	def wheels_cmd_callback(self, msg):
		if msg.data[0] > 100.0:
			left = 100.0
		elif msg.data[0] < -100.0:
			left = -100.0
		else:
			left = msg.data[0]

		if msg.data[1] > 100.0:
			right = 100.0
		elif msg.data[1] < -100.0:
			right = -100.0
		else:
			right = msg.data[1]

		self.left_percent = left
		self.right_percent = right

		self.wheels_cmd_cb_flag = True
		self.wheels_cmd_stamp = time.time()

	def cart_mode_cmd_callback(self, msg):
		self.cart_mode_cmd_flag = True
		self.cart_mode_cmd = msg.data

	def relay_cmd_callback(self, msg):
		self.relay_cb_flag = True
		self.relay_list = msg.data

	def servo_cmd_callback(self, msg):
		self.servo_cb_flag = True
		self.servo_pwm = msg.data

	# def led_cmd_callback(self, msg):
	# 	self.led_cb_flag = True
	# 	self.led_list = msg.data

	#######################################	
	################ Maths ################
	#######################################	

	def sbus2word(self, sbus_val):

		high_byte = sbus_val >> 8
		low_byte = (sbus_val & 0x00FF)

		return [high_byte, low_byte]

	def pwm2word(self, pwm_val):

		high_byte = pwm_val >> 8
		low_byte = (pwm_val & 0x00FF)

		return [high_byte, low_byte]

	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def convert2voltage(self, raw_list):
	
		raw_list = np.asarray(raw_list)
		voltage_array =  self.map(raw_list, 0.0, 255.0, 0.0, 40.96)
		return voltage_array.tolist()

	def over_limit_check(self, pwm):

		if pwm < self.pwm_min:
			pwm  = self.pwm_min
		elif pwm > self.pwm_max:
			pwm = self.pwm_max

		return pwm

	#######################################	
	######### SMBus2 read/write ###########
	#######################################

	def bypass_sbus_failsafe(self):
		## Disable SBUS Failsafe
		self.bus.write_byte_data(JMOAB_I2C_ADDR2, DISABLE_SBUS_FAILSAFE, DIS_FS)
		time.sleep(0.1)

		## Set back to hold mode
		self.write_atcart_mode(HOLD)
		time.sleep(0.1)

		## Set back to manual mode
		self.write_atcart_mode(MAN)
		time.sleep(0.1)
		self.write_atcart_mode(MAN)
		time.sleep(0.1)
		self.write_atcart_mode(MAN)
		time.sleep(0.1)

	def read_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(JMOAB_I2C_ADDR2, IN_SBUS_CH1_H, 20)

		SBUS_ch = [None]*10
		for i in range(10):
			hibyte = input_SBUS[i*2] 
			lobyte = input_SBUS[(2*i)+1]
			# needs only 3 bits from hibyte
			SBUS_ch[i] = ((lobyte & 0xFF) | ((hibyte&0x07) << 8))

		return SBUS_ch

	def read_atcart_mode(self):
		return self.bus.read_byte_data(JMOAB_I2C_ADDR2, CARTIF_INPUT_SELECT)

	def read_adc(self):
		raw = self.bus.read_i2c_block_data(JMOAB_I2C_ADDR2, ADC_AC0, 6)
		voltage_list = self.convert2voltage(raw)
		return voltage_list

	def write_left_right(self, sbus_left, sbus_right):

		steering_bytes = self.sbus2word(sbus_left)
		throttle_bytes = self.sbus2word(sbus_right)
		## combine as 4 elements [str_H, str_L, thr_H, thr_L]
		all_bytes = steering_bytes+throttle_bytes
		self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, CARTIF_SBUS_CH1_H, all_bytes)

	def write_atcart_mode(self, mode_num):
		## need to write multiple times to take effect
		## publisher side only sends one time is ok
		for i in range(10):
			self.bus.write_byte_data(JMOAB_I2C_ADDR2, CARTIF_INPUT_SELECT, mode_num)
		# self.bus.write_byte_data(JMOAB_I2C_ADDR2, CARTIF_INPUT_SELECT, mode_num)

	def write_relay(self, relay_list):
		all_bytes = [relay_list[1], relay_list[0]]
		self.bus.write_i2c_block_data(JMOAB_I2C_ADDR1, RY1, all_bytes)

	def write_servo(self, servo_list):

		if len(servo_list) > 0 and len(servo_list) <= 3:

			for servo_id in range(len(servo_list)):
				cmd_pwm = servo_list[servo_id]
				cmd_pwm = self.over_limit_check(cmd_pwm)
				self.write_pwm_bytes(servo_id, cmd_pwm)

	def write_pwm_bytes(self, servo_id, pwm):

		pwm_bytes = self.pwm2word(pwm)

		if servo_id == 0:
			self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM1_H, pwm_bytes)
		elif servo_id == 1:
			self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM2_H, pwm_bytes)
		elif servo_id == 2:
			self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM3_H, pwm_bytes)

	# def write_led(self, led_list):
	# 	all_bytes = [led_list[0], led_list[1], led_list[2]]
	# 	self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, AUX_LED1, all_bytes)

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

	def wheels_percent_to_wheels_sbus(self, left_per, right_per):

		if left_per > 0.0:
			left_sbus = self.map(left_per, 0.0, 200.0, self.sbus_left_max_db, self.sbus_max)
		elif left_per < 0.0:
			left_sbus = self.map(left_per, -200.0, 0.0, self.sbus_min, self.sbus_left_min_db)
		else:
			left_sbus = 1024

		if right_per > 0.0:
			right_sbus = self.map(right_per, 0.0, 200.0, self.sbus_right_max_db, self.sbus_max)
		elif right_per < 0.0:
			right_sbus = self.map(right_per, -200.0, 0.0, self.sbus_min, self.sbus_right_min_db)
		else:
			right_sbus = 1024

		return left_sbus, right_sbus

	#######################################	
	######## ROS2 timer callback ##########
	#######################################

	def timer_callback(self):

		#######################
		### data from JMOAB ###
		#######################
		self.adc_msg.data = self.read_adc()
		self.sbus_rc_msg.data = self.read_sbus_channel()
		atcart_mode = self.read_atcart_mode()
		self.cart_mode_msg.data = atcart_mode
		
		self.adc_pub.publish(self.adc_msg)
		self.sbus_rc_pub.publish(self.sbus_rc_msg)
		self.cart_mode_pub.publish(self.cart_mode_msg)

		############################
		### cart control command ###
		############################
		if atcart_mode == 2:
			cmd_vel_period = time.time() - self.cmd_vel_stamp
			wheels_cmd_period = time.time() - self.wheels_cmd_stamp

			## 1st priority is cmd_vel 
			if (cmd_vel_period < self.cmd_vel_timeout) and self.cmd_vel_cb_flag:

				if abs(self.vx) > 0.0 and abs(self.wz) > 0.0:
					y_percent = self.map(self.vx, -self.vx_max, self.vx_max, -100.0, 100.0)
					x_percent = self.map(self.wz, -self.wz_max, self.wz_max, y_percent, -y_percent)

					# if abs(x_percent) > abs(y_percent):
					# 	if x_percent > 0.0:
					# 		x_percent = y_percent
					# 	elif x_percent < 0.0:
					# 		x_percent = -y_percent
					
				else:
					x_percent = self.map(self.wz, -self.wz_max, self.wz_max, 100.0, -100.0)
					y_percent = self.map(self.vx, -self.vx_max, self.vx_max, -100.0, 100.0)

				left_200_per, right_200_per = self.xy_mixing(x_percent, y_percent)
				left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_200_per, right_200_per)
				self.write_left_right(int(left_sbus), int(right_sbus))

			## 2nd priority is wheels_cmd 
			elif (wheels_cmd_period < self.wheels_cmd_timeout) and self.wheels_cmd_cb_flag:
				x_percent = 0.0
				y_percent = 0.0

				left_200_per = self.left_percent*2.0
				right_200_per = self.right_percent*2.0

				left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_200_per, right_200_per)
				self.write_left_right(int(left_sbus), int(right_sbus))

			else:
				self.write_left_right(1024, 1024)
				self.cmd_vel_cb_flag = False
				self.wheels_cmd_cb_flag = False
				self.vx = 0.0
				self.wz = 0.0
				self.left_percent = 0.0
				self.right_percent = 0.0
				left_sbus = 1024
				right_sbus = 1024
				x_percent = 0.0
				y_percent = 0.0

			if time.time() - self.last_stamp_log > 1.0:
				self.get_logger().info("vx: {:.2f} wz: {:.2f} left: {:.2f} right: {:.2f} x: {:.2f} y: {:.2f} l_sbus: {:d} r_sbus: {:d}".format(\
					self.vx, self.wz, self.left_percent, self.right_percent, x_percent, y_percent, int(left_sbus), int(right_sbus)))
				self.last_stamp_log = time.time()


		elif atcart_mode == 0:
			self.write_left_right(1024, 1024)
			self.cmd_vel_cb_flag = False
			self.wheels_cmd_cb_flag = False
			self.last_stamp_log = time.time()

		else:
			self.last_stamp_log = time.time()

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
			cmd_pwm = self.over_limit_check(self.servo_pwm)
			self.write_pwm_bytes(2, cmd_pwm)
			self.servo_cb_flag = False

		###################
		### led command ###
		###################
		# if self.led_cb_flag:
		# 	self.write_led(self.led_list)
		# 	self.led_cb_flag = False


def main(args=None):

	rclpy.init(args=args)

	node = JMOAB_ATCART_BASIC()

	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdownn()


if __name__ == "__main__":

	main()