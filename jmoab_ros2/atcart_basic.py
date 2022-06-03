
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, UInt8, Float32MultiArray
import time
import numpy as np
from smbus2 import SMBus

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



class JMOAB_ATCART_BASIC(Node):

	def __init__(self):

		super().__init__('jmoab_ros2_atcart_basic')
		self.get_logger().info('Start JMOAB ROS2 node')

		self.bus = SMBus(1)

		### Class parameters ###
		#### cart_cmd
		##### mixing for steering/throttle control
		self.sbus_cmd_steering = 1024
		self.sbus_cmd_throttle = 1024
		self.cart_cmd_cb_timeout = 1.0 # second
		self.cart_cmd_cb_timestamp = time.time()
		self.cart_cmd_flag = False
		self.prev_y = 0.0
		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0
		self.sbus_max_DB = self.sbus_mid + 1.0
		self.sbus_min_DB = self.sbus_mid - 1.0
		self.sbus_min_backward = 968	# a value before start rotating backward
		self.sbus_min_forward = 1093	# a value before start rotating forward
		##### non-mixing for left/right control
		self.cmd_left = 1024
		self.cmd_right = 1024
		self.cart_lr_cmd_cb_timeout = 1.0
		self.cart_lr_cmd_cb_timestamp = time.time()
		self.cart_lr_cmd_flag = False

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

		### Pub/Sub ###
		self.sbus_rc_pub = self.create_publisher(UInt16MultiArray, 'jmoab/sbus_rc_ch', 10)
		self.atcart_mode_pub = self.create_publisher(UInt8, 'jmoab/atcart_mode', 10)
		self.adc_pub = self.create_publisher(Float32MultiArray, 'jmoab/adc', 10)

		self.sbus_rc_msg = UInt16MultiArray()
		self.atcart_mode_msg = UInt8()
		self.adc_msg = Float32MultiArray()

		self.cart_cmd_sub = self.create_subscription(UInt16MultiArray, 'jmoab/cart_cmd', self.cart_cmd_callback, 10)
		self.cart_lr_cmd_sub = self.create_subscription(UInt16MultiArray, 'jmoab/cart_lr_cmd', self.cart_lr_cmd_callback, 10)
		self.atcart_mode_cmd_sub = self.create_subscription(UInt8, 'jmoab/atcart_mode_cmd', self.atcart_mode_cmd_callback, 10)
		self.relay_sub = self.create_subscription(UInt8MultiArray, 'jmoab/relay', self.relay_cmd_callback, 10)
		self.servo_sub = self.create_subscription(UInt16MultiArray, 'jmoab/servo', self.servo_cmd_callback, 10)
		self.led_sub = self.create_subscription(UInt8MultiArray, 'jmoab/led', self.led_cmd_callback, 10)

		self.get_logger().info('Publishing to jmoab/sbus_rc_ch [std_msgs/msg/UInt16MultiArray]')
		self.get_logger().info('Publishing to jmoab/atcart_mode [std_msgs/msg/UInt8]')
		self.get_logger().info('Publishing to jmoab/adc [std_msgs/msg/Float32MultiArray]')
		self.get_logger().info('Subscribing on jmoab/cart_cmd [std_msgs/msg/UInt16MultiArray]')
		self.get_logger().info('jmoab/cart_cmd ex. [sbus_steering, sbus_throttle] 368-1024-1680 as sbus range')
		self.get_logger().info('Subscribing on jmoab/cart_lr_cmd [std_msgs/msg/UInt16MultiArray]')
		self.get_logger().info('jmoab/cart_lr_cmd ] ex. [sbus_left, sbus_right], min_fwd: 1093, min_bwd: 968')
		self.get_logger().info('Subscribing on jmoab/atcart_mode_cmd [std_msgs/msg/UInt8]')
		self.get_logger().info('Subscribing on jmoab/relay [std_msgs/msg/UInt8MultiArray]')
		self.get_logger().info('Subscribing on jmoab/servo [std_msgs/msg/UInt16MultiArray]')
		self.get_logger().info('Subscribing on jmoab/led [std_msgs/msg/UInt8MultiArray]')

		### Uncomment this if want to turn off propo in auto mode ###
		# self.bypass_sbus_failsafe()

		### Loop spin ###
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.timer_callback)

	#######################################	
	############ ROS callbacks ############
	#######################################	
	def cart_cmd_callback(self, msg):
		# str/thr cmd is first priority for cart control
		if len(msg.data) > 0:
			self.cmd_steering = msg.data[0]
			self.cmd_throttle = msg.data[1]
			self.cart_cmd_cb_timestamp = time.time()
			self.cart_cmd_flag = True

	def cart_lr_cmd_callback(self, msg):
		# if str/thr cmd is receiving, we ignore this left/right cmd
		if len(msg.data) > 0 and not self.cart_cmd_flag:
			self.cmd_left = msg.data[0]
			self.cmd_right = msg.data[1]
			self.cart_lr_cmd_cb_timestamp = time.time()
			self.cart_lr_cmd_flag = True

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

	def write_left_right(self, sbus_left, sbus_right):

		steering_bytes = self.sbus2word(sbus_left)
		throttle_bytes = self.sbus2word(sbus_right)
		## combine as 4 elements [str_H, str_L, thr_H, thr_L]
		all_bytes = steering_bytes+throttle_bytes
		self.bus.write_i2c_block_data(JMOAB_MAIN_ADDR, CARTIF_SBUS_CH1_H, all_bytes)

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
			left_sbus = self.map(left_per, 0.0, 200.0, self.sbus_min_forward, self.sbus_max)
		elif left_per < 0.0:
			left_sbus = self.map(left_per, -200.0, 0.0, self.sbus_min, self.sbus_min_backward)
		else:
			left_sbus = 1024

		if right_per > 0.0:
			right_sbus = self.map(right_per, 0.0, 200.0, self.sbus_min_forward, self.sbus_max)
		elif right_per < 0.0:
			right_sbus = self.map(right_per, -200.0, 0.0, self.sbus_min, self.sbus_min_backward)
		else:
			right_sbus = 1024

		return left_sbus, right_sbus

	def check_sbus_range(self, sbus):
		if 1000 < sbus < 1048:
			return 1024
		elif self.sbus_min_backward <= sbus <= 1000:
			return self.sbus_min_backward
		elif 1048 <= sbus <= self.sbus_min_forward:
			return self.sbus_min_forward
		elif sbus < 368:
			return 368
		elif sbus > 1680:
			return 1680
		else:
			return sbus

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
		self.atcart_mode_msg.data = atcart_mode
		
		self.adc_pub.publish(self.adc_msg)
		self.sbus_rc_pub.publish(self.sbus_rc_msg)
		self.atcart_mode_pub.publish(self.atcart_mode_msg)

		############################
		### cart control command ###
		############################
		if atcart_mode == 2:
			## command are steering, throttle ##
			if ((time.time() - self.cart_cmd_cb_timestamp) < self.cart_cmd_cb_timeout) and self.cart_cmd_flag and not self.cart_lr_cmd_flag:
				x_percent = self.sbus2percent(float(self.cmd_steering))
				y_percent = self.sbus2percent(float(self.cmd_throttle))
				left_percent, right_percent = self.xy_mixing(x_percent, y_percent)
				left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_percent, right_percent)
				self.write_left_right(int(left_sbus), int(right_sbus))

			## command are left, right ##
			elif ((time.time() - self.cart_lr_cmd_cb_timestamp) < self.cart_lr_cmd_cb_timeout) and self.cart_lr_cmd_flag and not self.cart_cmd_flag:
				self.write_left_right(int(self.check_sbus_range(self.cmd_left)), int(self.check_sbus_range(self.cmd_right)))

			## str/thr cmd timeout ##
			elif (time.time() - self.cart_cmd_cb_timestamp) > self.cart_cmd_cb_timeout: 
				self.write_left_right(1024, 1024)
				self.cart_cmd_flag = False

			## left/right cmd timeout ##
			elif (time.time() - self.cart_lr_cmd_cb_timestamp) > self.cart_lr_cmd_cb_timeout: 
				self.write_left_right(1024, 1024)
				self.cart_lr_cmd_flag = False

		elif atcart_mode == 0:
			self.write_left_right(1024, 1024)

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


def main(args=None):

	rclpy.init(args=args)

	node = JMOAB_ATCART_BASIC()

	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdownn()


if __name__ == "__main__":

	main()