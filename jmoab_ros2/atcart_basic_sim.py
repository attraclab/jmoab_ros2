
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int16MultiArray, UInt8, Float32MultiArray, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import time
import numpy as np

class JMOAB_ATCART_BASIC(Node):

	def __init__(self):

		super().__init__('jmoab_ros2_atcart_basic_sim')
		self.get_logger().info('Start JMOAB ROS2 node')

		self.declare_parameter('show_log', False)
		self.add_on_set_parameters_callback(self.parameter_callback)
		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("show_log: {}".format(self.show_log))

		### Class parameters ###
		#### cart_cmd
		##### mixing for steering/throttle control
		self.sbus_cmd_steering = 1024
		self.sbus_cmd_throttle = 1024

		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0

		self.prev_y = 0.0

		self.wheels_cmd_max = 10.0
		self.cart_mode = 1

		### cmd_vel
		self.vx_max = 2.0
		self.wz_max = 2.0
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

		#### led cmd
		self.led_cb_flag = False
		self.led_list = []

		### sbus channels sim ###
		self.str_percent = 0.0
		self.thr_percent = 0.0
		self.ch1_sim = 1024
		self.ch2_sim = 1024
		self.ch5_sim = 1024
		self.ch7_sim = 1024
		self.ch8_sim = 1024
		self.ch9_sim = 1024

		self.last_log_stamp = time.time()

		### JMOAB Pub/Sub ###
		self.sbus_rc_pub = self.create_publisher(Int16MultiArray, 'jmoab/sbus_rc_ch', 10)
		self.cart_mode_pub = self.create_publisher(UInt8, 'jmoab/cart_mode', 10)
		self.adc_pub = self.create_publisher(Float32MultiArray, 'jmoab/adc', 10)

		self.sbus_rc_msg = Int16MultiArray()
		self.cart_mode_msg = UInt8()
		self.adc_msg = Float32MultiArray()

		self.wheels_cmd_sub = self.create_subscription(Float32MultiArray, 'jmoab/wheels_cmd', self.wheels_cmd_callback, 10)
		self.cart_mode_cmd_sub = self.create_subscription(UInt8, 'jmoab/cart_mode_cmd', self.cart_mode_cmd_callback, 10)
		self.relay_sub = self.create_subscription(Int8MultiArray, 'jmoab/relays', self.relay_cmd_callback, 10)
		self.servo_sub = self.create_subscription(Int16MultiArray, 'jmoab/servos', self.servo_cmd_callback, 10)

		### General Pub/Sub ###
		self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
		self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

		### Gazebo Pub/Sub ###
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
		self.gazebo_cmd_vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', qos_profile=qos_policy)

		self.get_logger().info('Publishing  to /jmoab/sbus_rc_ch [std_msgs/msg/Int16MultiArray]')
		self.get_logger().info('Publishing  to /jmoab/cart_mode [std_msgs/msg/UInt8]')
		self.get_logger().info('Publishing  to /jmoab/adc [std_msgs/msg/Float32MultiArray]')
		
		self.get_logger().info('Subscribing on /cmd_vel [geometry_msgs/msg/Twist] ex: linear.x angular.z')
		self.get_logger().info('Subscribing on /jmoab/wheels_cmd [std_msgs/msg/Float32MultiArray] ex: [30.0, -50.0] percentages -100.0 to 100.0 ranges')
		self.get_logger().info('Subscribing on /jmoab/cart_mode_cmd [std_msgs/msg/UInt8]  ex: 0=hold, 1=manual, 2=auto')
		self.get_logger().info('Subscribing on /jmoab/relays [std_msgs/msg/Int8MultiArray]  ex: [1,1] on both relays')
		self.get_logger().info('Subscribing on /jmoab/servos [std_msgs/msg/Int16MultiArray]  ex: [19200, 1120, 1520] ')

		### Loop spin ###
		timer_period = 0.05
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
		self.servo_list = msg.data

	def joy_callback(self, msg):

		if msg.buttons[0] == 1:
			self.cart_mode = 2
			self.mode_name = "AUTO"
			self.ch5_sim = 1904
		elif msg.buttons[1] == 1:
			self.cart_mode = 0
			self.mode_name = "HOLD"
			self.ch5_sim = 144
		elif msg.buttons[2] == 1:
			self.cart_mode = 1
			self.mode_name = "MANUAL"
			self.ch5_sim = 1024

		## back
		if msg.buttons[7] == 1:
			self.ch7_sim = 144
		## Logicool
		elif msg.buttons[8] == 1:
			self.ch7_sim = 1024
		## start
		elif msg.buttons[6] == 1:
			self.ch7_sim = 1904

		## Y
		if msg.buttons[3] == 1:
			self.ch8_sim = 1904
		else:
			self.ch8_sim = 144

		## RB
		if msg.buttons[5] == 1:
			self.ch9_sim = 1904
		## LB
		elif msg.buttons[4] == 1:
			self.ch9_sim = 144


		self.ch1_sim = int(self.map(msg.axes[3], -1.0, 1.0, 1680.0, 368.0))
		self.ch2_sim = int(self.map(msg.axes[1], -1.0, 1.0, 368.0, 1680.0))

		self.str_percent = self.map(msg.axes[3], -1.0, 1.0, 100.0, -100.0)
		self.thr_percent = self.map(msg.axes[1], -1.0, 1.0, -100.0, 100.0)


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

	def over_limit_check(self, pwm):

		if pwm < self.pwm_min:
			pwm  = self.pwm_min
		elif pwm > self.pwm_max:
			pwm = self.pwm_max

		return pwm

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

	#######################################	
	######## ROS2 timer callback ##########
	#######################################

	def timer_callback(self):

		#######################
		### data from JMOAB ###
		#######################
		self.adc_msg.data = np.random.uniform(0.0, 40.96, size=(6,)).tolist()
		self.sbus_rc_msg.data = [self.ch1_sim, self.ch2_sim, 1024, 1024, self.ch5_sim, 1024, self.ch7_sim, self.ch8_sim, self.ch9_sim, 1024]
		self.cart_mode_msg.data = self.cart_mode
		
		self.adc_pub.publish(self.adc_msg)
		self.sbus_rc_pub.publish(self.sbus_rc_msg)
		self.cart_mode_pub.publish(self.cart_mode_msg)

		############################
		### cart control command ###
		############################
		if self.cart_mode == 1:

			left_200, right_200 = self.xy_mixing(self.str_percent, self.thr_percent)
			cmd_left = self.map(left_200, -200.0, 200.0,  -self.wheels_cmd_max, self.wheels_cmd_max)
			cmd_right = self.map(right_200, -200.0, 200.0,  -self.wheels_cmd_max, self.wheels_cmd_max)

		elif self.cart_mode == 2:
			cmd_vel_period = time.time() - self.cmd_vel_stamp
			wheels_cmd_period = time.time() - self.wheels_cmd_stamp

			## 1st priority is cmd_vel 
			if (cmd_vel_period < self.cmd_vel_timeout) and self.cmd_vel_cb_flag:

				## go straight
				if (abs(self.vx) > 0.0) and (self.wz == 0.0):
					cmd_left = self.map(self.vx, -self.vx_max, self.vx_max, -self.wheels_cmd_max, self.wheels_cmd_max)
					cmd_right = cmd_left

				## skid
				elif (self.vx == 0.0) and (abs(self.wz) > 0.0):
					if self.wz > 0.0:
						cmd_left = self.map(self.wz, 0.0, self.wz_max, 0.0, -self.wheels_cmd_max) #0.0
						cmd_right = self.map(self.wz, 0.0, self.wz_max, 0.0, self.wheels_cmd_max)
					elif self.wz < 0.0:
						cmd_left = self.map(self.wz, -self.wz_max, 0.0, self.wheels_cmd_max, 0.0)
						cmd_right = self.map(self.wz, -self.wz_max, 0.0, -self.wheels_cmd_max, 0.0)  #0.0
					else:
						cmd_left = 0.0
						cmd_right = 0.0

				## curving
				elif abs(self.vx) > 0.0 and abs(self.wz) > 0.0:
					## curve front
					if self.vx > 0.0:
						## curve to the left direction, left_wheel slow, right wheel constant
						if self.wz > 0.0:
							cmd_right = self.map(self.vx, 0.0, self.vx_max, 0.0, self.wheels_cmd_max)
							cmd_left = self.map(self.wz, 0.0, self.wz_max, cmd_right, 0.0)

						## curve to the right direction, left_wheel constant, right wheel slow down
						elif self.wz < 0.0:
							cmd_left = self.map(self.vx, 0.0, self.vx_max, 0.0, self.wheels_cmd_max)
							cmd_right = self.map(self.wz, -self.wz_max, 0.0, 0.0, cmd_left)

					## curve backward
					elif self.vx < 0.0:
						## cruve to the right direction, left wheel constant, right wheel slow down
						if self.wz > 0.0:
							cmd_left = self.map(self.vx, -self.vx_max, 0.0, -self.wheels_cmd_max, 0.0)
							cmd_right = self.map(self.wz, 0.0, self.wz_max, cmd_left, 0.0)

						## curve to the left direction, left wheel slow donw, right wheel constant
						elif self.wz < 0.0:
							cmd_right = self.map(self.vx, -self.vx_max, 0.0, -self.wheels_cmd_max, 0.0)
							cmd_left = self.map(self.wz, -self.wz_max, 0.0, 0.0, cmd_right)
								## no motion

				## no motion
				else:
					cmd_left = 0.0
					cmd_right = 0.0
					

			## 2nd priority is wheels_cmd 
			elif (wheels_cmd_period < self.wheels_cmd_timeout) and self.wheels_cmd_cb_flag:
				
				cmd_left = self.map(self.left_percent, -100.0, 100.0, -self.wheels_cmd_max, self.wheels_cmd_max)
				cmd_right = self.map(self.right_percent, -100.0, 100.0, -self.wheels_cmd_max, self.wheels_cmd_max)

			else:
				self.cmd_vel_cb_flag = False
				self.wheels_cmd_cb_flag = False
				self.vx = 0.0
				self.wz = 0.0
				self.left_percent = 0.0
				self.right_percent = 0.0
				cmd_left = 0.0
				cmd_right = 0.0

		else:
			cmd_left = 0.0
			cmd_right = 0.0
			self.cmd_vel_cb_flag = False
			self.wheels_cmd_cb_flag = False

		if ((time.time() - self.last_log_stamp) > 1.0) and self.show_log:
			self.get_logger().info("mode: {:d} str: {:.1f} thr: {:.1f} vx: {:.2f} wz: {:.2f} %_left: {:.2f} %_right: {:.2f} cmd_l: {:.2f} cmd_r: {:.2f}".format(\
				self.cart_mode, self.str_percent, self.thr_percent, self.vx, self.wz, self.left_percent, self.right_percent, cmd_left, cmd_right))
			self.last_log_stamp = time.time()

		##############################
		## publish cmd_vel to robot ##
		##############################
		wheels_cmd_msg = Float64MultiArray()
		wheels_cmd_msg.data = [cmd_left, cmd_right]

		self.gazebo_cmd_vel_pub.publish(wheels_cmd_msg)

		# #########################
		# ### cart mode command ###
		# #########################
		# if self.cart_mode_cmd_flag:
		# 	self.write_atcart_mode(self.cart_mode_cmd)
		# 	self.cart_mode_cmd_flag = False

		# #####################
		# ### relay command ###
		# #####################
		# if self.relay_cb_flag:
		# 	self.write_relay(self.relay_list)
		# 	self.relay_cb_flag = False

		# #####################
		# ### servo command ###
		# #####################
		# if self.servo_cb_flag:
		# 	self.write_servo(self.servo_list)
		# 	self.servo_cb_flag = False



def main(args=None):

	rclpy.init(args=args)

	node = JMOAB_ATCART_BASIC()

	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdownn()


if __name__ == "__main__":

	main()