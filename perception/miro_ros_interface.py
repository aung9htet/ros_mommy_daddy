# MiRo-E ROS message types
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Image, Imu, Range, CompressedImage
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from miro2_msg import msg

# MiRo-E modules and parameters
try:
	from . import miro_constants as con
except ImportError:
	import miro_constants as con
import miro2 as miro

# Other packages
import cv2
import datetime
import math
import numpy as np
import os
import rosnode
import rospy


class MiRo:
	def __init__(self):
		name = 'MiRo_ROS_interface'
		# Initialise ROS node ('disable_rostime=True' needed to work in PyCharm)
		if name not in rosnode.get_node_names():
			rospy.init_node(name, anonymous=True, disable_rostime="PYCHARM_HOSTED" in os.environ)
			
		# ROS topic root
		self.tr = '/' + os.getenv('MIRO_ROBOT_NAME') + '/'

		# Publisher queue size
		self.qs = 2

		# Initialisation sleep duration
		self.sleep_time = 0.5

	@staticmethod
	def ros_sleep(time):
		# Sleep after init to prevent accessing data before a topic is subscribed
		rospy.sleep(time)


class MiRoCore(MiRo):
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		# TODO: Add init test to check if demo code is running
		MiRo.__init__(self)

		# Topic subscriptions
		# State
		rospy.Subscriber(self.tr + 'core/animal/state', miro.msg.animal_state, self.callback_core_state)
		# rospy.Subscriber(topic_root + '/core/detect_objects_l', miro.msg.objects, self.callback_detect_objects_l)
		# rospy.Subscriber(topic_root + '/core/detect_objects_r', miro.msg.objects, self.callback_detect_objects_r)
		# rospy.Subscriber(topic_root + '/core/detect_ball_l', UInt16MultiArray, self.callback_detect_ball_l)
		# rospy.Subscriber(topic_root + '/core/detect_ball_r', UInt16MultiArray, self.callback_detect_ball_r)
		# rospy.Subscriber(topic_root + '/core/detect_face_l', Float32MultiArray, self.callback_detect_face_l)
		# rospy.Subscriber(topic_root + '/core/detect_face_r', Float32MultiArray, self.callback_detect_face_r)
		# Salience maps
		rospy.Subscriber(self.tr + 'core/pril', Image, self.callback_pril)
		rospy.Subscriber(self.tr + 'core/prir', Image, self.callback_prir)
		rospy.Subscriber(self.tr + 'core/priw', Image, self.callback_priw)
		# Selection
		rospy.Subscriber(self.tr + 'core/selection/inhibition', Float32MultiArray, self.callback_selection_inhibition)
		rospy.Subscriber(self.tr + 'core/selection/priority', Float32MultiArray, self.callback_selection_priority)
		# Motivation
		rospy.Subscriber(self.tr + 'motivation', Float32MultiArray, self.callback_motivation)

		# Default data
		# self.core_detect_objects_l = None
		# self.core_detect_objects_r = None
		# self.core_detect_ball_l = None
		# self.core_detect_ball_r = None
		# self.core_detect_face_l = None
		# self.core_detect_face_r = None
		self.emotion = None
		self.mood = None
		self.motivation = None
		self.pril = None
		self.prir = None
		self.priw = None
		self.selection_inhibition = None
		self.selection_priority = None
		self.sleep = None
		self.time = None
		self.time_raw = None

		# Sleep for ROS initialisation
		self.ros_sleep(self.sleep_time)

	def callback_core_state(self, data):
		self.emotion = data.emotion
		self.mood = data.mood
		self.sleep = data.sleep
		self.time_raw = data.time_of_day
		timedelta = datetime.timedelta(self.time_raw)
		try:
			self.time = datetime.datetime.strptime(str(timedelta), '%H:%M:%S.%f').time()
		except ValueError:
			# Catch errors when microseconds == 0
			self.time = datetime.datetime.strptime(str(timedelta), '%H:%M:%S').time()

	def callback_motivation(self, data):
		self.motivation = data

	def callback_pril(self, frame):
		self.pril = self.process_pri(frame, height=con.PRI['height'], width=con.PRI['width'])

	def callback_prir(self, frame):
		self.prir = self.process_pri(frame, height=con.PRI['height'], width=con.PRI['width'])

	def callback_priw(self, frame):
		self.priw = self.process_pri(frame, height=con.PRIW['height'], width=con.PRIW['width'])

	def callback_selection_inhibition(self, data):
		self.selection_inhibition = data

	def callback_selection_priority(self, data):
		self.selection_priority = data

	@staticmethod
	def process_pri(frame, height, width):
		# Resize frame data to form an OpenCV image array
		image_array = np.frombuffer(frame.data, np.uint8)
		image_array.resize((height, width))
		return image_array


class MiRoPerception(MiRo):
	# Asynchronous sensors
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

		# TODO: Add test for physical or simulated robot to switch this flag
		self.opt = {'Uncompressed': False}

		# Topic subscriptions
		if self.opt['Uncompressed']:
			# TODO: Uncompressed callbacks are untested
			rospy.Subscriber(self.tr + 'sensors/caml', Image, self.callback_caml)
			rospy.Subscriber(self.tr + 'sensors/camr', Image, self.callback_camr)
		else:
			rospy.Subscriber(self.tr + 'sensors/caml/compressed', CompressedImage, self.callback_caml)
			rospy.Subscriber(self.tr + 'sensors/camr/compressed', CompressedImage, self.callback_camr)
		rospy.Subscriber(self.tr + 'sensors/mics', Int16MultiArray, self.callback_mics)

		# Default data
		self.caml = None
		self.caml_undistorted = None
		self.camr = None
		self.camr_undistorted = None
		self.mics = None

		# Sleep for ROS initialisation
		self.ros_sleep(self.sleep_time)

	# TODO: Image stitching
	def callback_caml(self, frame):
		[self.caml, self.caml_undistorted] = self.process_frame(frame)

	def callback_camr(self, frame):
		[self.camr, self.camr_undistorted] = self.process_frame(frame)

	def callback_mics(self, msg):
		# Rescale data to be between -1 and +1
		data = np.asarray(msg.data, 'float32') * (1.0 / 32768.0)

		# Separate out samples from each microphone
		# Samples in the order [LEFT, RIGHT, CENTRE, TAIL]
		data = data.reshape((con.MICS, con.BLOCK_SAMPLES))
		self.mics = {
			'left'  : data[0],
			'right' : data[1],
			'centre': data[2],
			'tail'  : data[3]
		}

	@staticmethod
	def process_frame(frame):
		# Convert frame data to an OpenCV image array
		frame_array = np.frombuffer(frame.data, np.uint8)
		image_array = cv2.imdecode(frame_array, cv2.IMREAD_UNCHANGED)
		# image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
		image_undistorted = cv2.undistort(image_array, con.MTX, con.DIST, None)

		return image_array, image_undistorted


class MiRoSensors(MiRo):
	# Synchronous 50Hz sensors
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

		# Topic subscriptions
		rospy.Subscriber(self.tr + 'sensors/package', msg.sensors_package, self.callback_sensors)

		# Initialise data
		self.sensors = None
		self.kinematic_joints = None
		self.light = None
		self.cliff = None
		self.sonar = None

		# Sleep for ROS initialisation
		self.ros_sleep(self.sleep_time)

	def callback_sensors(self, sensors):
		self.sensors = sensors

		# Internal degrees-of-freedom configuration (radians)
		# In the order [TILT, LIFT, YAW, PITCH]
		self.kinematic_joints = {
			'tilt' : sensors.kinematic_joints.position[0],
			'lift' : sensors.kinematic_joints.position[1],
			'yaw'  : sensors.kinematic_joints.position[2],
			'pitch': sensors.kinematic_joints.position[3]
		}

		# Normalised light level at each of the four light sensors
		# 0 = dark, 1 = lit
		# In the order [FRONT LEFT, FRONT RIGHT, REAR LEFT, REAR RIGHT]
		self.light = {
			'front_left' : sensors.light.data[0],
			'front_right': sensors.light.data[1],
			'rear_left'  : sensors.light.data[2],
			'rear_right' : sensors.light.data[3],
		}

		# Normalised reading from each of the two cliff sensors
		# 0 = no surface, 1 = surface
		# In the order [LEFT, RIGHT].
		self.cliff = {
			'left' : sensors.cliff.data[0],
			'right': sensors.cliff.data[1],
		}

		# Range to strongest sonar reflector (metres)
		# Normal values are in the interval [0.03, 1.0]
		self.sonar = sensors.sonar.range


class MiRoPublishers(MiRo):
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

		# Topics
		self.cmd_vel = rospy.Publisher(self.tr + 'control/cmd_vel', TwistStamped, queue_size=self.qs)
		self.kinematic_joints = rospy.Publisher(self.tr + 'control/kinematic_joints', JointState, queue_size=self.qs)
		self.cosmetic_joints = rospy.Publisher(self.tr + 'control/cosmetic_joints', Float32MultiArray, queue_size=self.qs)
		self.illum = rospy.Publisher(self.tr + 'control/illum', UInt32MultiArray, queue_size=self.qs)
		self.tone = rospy.Publisher(self.tr + 'control/tone', UInt16MultiArray, queue_size=self.qs)

		# Initialise messages
		self.cmd_vel_msg = TwistStamped()
		self.kinematic_joints_msg = JointState()
		self.cosmetic_joints_msg = Float32MultiArray()
		self.illum_msg = UInt32MultiArray()
		self.tone_msg = UInt16MultiArray()

		# Sleep for ROS initialisation
		self.ros_sleep(self.sleep_time)

	# Publish wheel speeds (m/s)
	def pub_cmd_vel_ms(self, left=0, right=0):
		try:
			# Python 2 MDK
			(dr, dtheta) = miro.utils.wheel_speed2cmd_vel([left, right])
		except AttributeError:
			# Python 3 MDK
			(dr, dtheta) = miro.lib.miro_utils.wheel_speed2cmd_vel([left, right])
		self.pub_cmd_vel_rad(dr, dtheta)

	# Publish wheel speeds (radians)
	def pub_cmd_vel_rad(self, dr, dtheta):
		self.cmd_vel_msg.twist.linear.x = dr
		self.cmd_vel_msg.twist.angular.z = dtheta
		self.cmd_vel.publish(self.cmd_vel_msg)

	# Publish cosmetic joint positions
	def pub_cosmetic_joints(
			self,
			droop=miro.constants.DROOP_DEFAULT,
			wag=miro.constants.WAG_DEFAULT,
			eye_left=miro.constants.EYE_DEFAULT,
			eye_right=miro.constants.EYE_DEFAULT,
			ear_left=miro.constants.EAR_DEFAULT,
			ear_right=miro.constants.EAR_DEFAULT,
			**kwargs
	):
		# Normalised configuration of cosmetic joints
		# Six joints are commanded in the order [DROOP, WAG, L EYE, R EYE, L EAR, R EAR]
		# Joint positions:
		# Droop: 0=up       1=down
		# Wag:   0=left     1=right
		# Eyes:  0=open     1=closed
		# Ears:  0=inwards  1=outwards

		# Named joint positions
		if droop == 'up': droop = 0
		if droop == 'down': droop = 1
		if wag == 'left': wag = 0
		if wag == 'right': wag = 1
		if eye_left == 'open': eye_left = 0
		if eye_left == 'closed': eye_left = 1
		if eye_right == 'open': eye_right = 0
		if eye_right == 'closed': eye_right = 1
		if ear_left in ('inwards', 'in'): ear_left = 0
		if ear_left in ('outwards', 'out'): ear_left = 1
		if ear_right in ('inwards', 'in'): ear_right = 0
		if ear_right in ('outwards', 'out'): ear_right = 1

		# Set multiple joints at once
		if 'eyes' in kwargs.keys():
			if kwargs['eyes'] == 'open': kwargs['eyes'] = 0
			if kwargs['eyes'] == 'closed': kwargs['eyes'] = 1
			eye_left = eye_right = kwargs['eyes']

		if 'ears' in kwargs.keys():
			if kwargs['ears'] in ('inwards', 'in'): kwargs['ears'] = 0
			if kwargs['ears'] in ('outwards', 'out'): kwargs['ears'] = 1
			ear_left = ear_right = kwargs['ears']

		self.cosmetic_joints_msg.data = [droop, wag, eye_left, eye_right, ear_left, ear_right]
		self.cosmetic_joints.publish(self.cosmetic_joints_msg)

	# Publish kinematic joint positions
	def pub_kinematic_joints(
			self,
			lift=math.degrees(miro.constants.LIFT_RAD_CALIB),
			yaw=math.degrees(miro.constants.YAW_RAD_CALIB),
			pitch=math.degrees(miro.constants.PITCH_RAD_CALIB)
	):
		# Internal DOF configuration (radians) in the order [TILT, LIFT, YAW, PITCH]
		self.kinematic_joints_msg.position = [
			miro.constants.TILT_RAD_CALIB,
			np.clip(
				math.radians(lift),
				miro.constants.LIFT_RAD_MIN,
				miro.constants.LIFT_RAD_MAX
			),
			np.clip(
				math.radians(yaw),
				miro.constants.YAW_RAD_MIN,
				miro.constants.YAW_RAD_MAX
			),
			np.clip(
				math.radians(pitch),
				miro.constants.PITCH_RAD_MIN,
				miro.constants.PITCH_RAD_MAX
			)
		]

		self.kinematic_joints.publish(self.kinematic_joints_msg)

	# Publish illumination
	def pub_illum(
			self,
			left_front=con.ARGB_WORD['off'],
			left_mid=con.ARGB_WORD['off'],
			left_rear=con.ARGB_WORD['off'],
			right_front=con.ARGB_WORD['off'],
			right_mid=con.ARGB_WORD['off'],
			right_rear=con.ARGB_WORD['off'],
			**kwargs
	):
		# Commanded pattern for the six LEDs in the order [L FRONT, L MIDDLE, L REAR, R FRONT, R MIDDLE, R REAR]
		# Each element is an ARGB word (0xAARRGGBB) where A is a brightness channel that scales the other three
		for key in kwargs:
			try:
				# If a named colour is specified try to retrieve it from the colour list
				kwargs[key] = con.ARGB_WORD[kwargs[key]]
			except KeyError:
				if isinstance(kwargs[key], str):
					print('Unknown colour name provided')
				else:
					# Assume we received an ARGB word
					# TODO: Add better error checking here (eg. if user tries to pass an RGB tuple)
					pass

		# Set multiple lights at once
		if kwargs.get('front'):
			left_front = right_front = kwargs['front']
		if kwargs.get('mid'):
			left_mid = right_mid = kwargs['mid']
		if kwargs.get('rear'):
			left_rear = right_rear = kwargs['rear']
		if kwargs.get('left_all'):
			left_front = left_mid = left_rear = kwargs['left_all']
		if kwargs.get('right_all'):
			right_front = right_mid = right_rear = kwargs['right_all']
		if kwargs.get('all'):
			left_front = left_mid = left_rear = right_front = right_mid = right_rear = kwargs['all']

		self.illum_msg.data = [left_front, left_mid, left_rear, right_front, right_mid, right_rear]
		self.illum.publish(self.illum_msg)

	# Publish audio tone
	# TODO: Use * to force keyword arguments in Python 3
	def pub_tone(
			self,
			duration,
			volume,
			frequency=None,
			note=None,
	):
		# Convert a string note (eg. "A4") to a frequency (eg. 440)
		# From https://gist.github.com/CGrassin/26a1fdf4fc5de788da9b376ff717516e - MIT license
		def get_frequency(n, a4=440):
			notes = ['A', 'A#', 'B', 'C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#']
			octave = int(n[2]) if len(n) == 3 else int(n[1])
			key_number = notes.index(n[0:-1])

			if key_number < 3:
				key_number = key_number + 12 + ((octave - 1) * 12) + 1
			else:
				key_number = key_number + ((octave - 1) * 12) + 1

			return a4 * 2 ** ((key_number - 49) / 12)

		# A note value overrides the specified frequency
		try:
			frequency = get_frequency(note)
		except IndexError:
			print('Please specify both note and octave value, eg. C4')
		except ValueError:
			print('{} is not a recognised note'.format(note))
		except TypeError:
			pass

		# Frequency in Hz (values between 50 and 2000)
		frequency = int(np.clip(
			frequency,
			con.TONE_FREQUENCY['min'],
			con.TONE_FREQUENCY['max']
		))

		# Volume from 0 to 255
		volume = np.clip(
			volume,
			con.TONE_VOLUME['min'],
			con.TONE_VOLUME['max']
		)

		# Duration in seconds (20ms platform ticks * 50)
		duration = np.maximum(0, duration * miro.constants.PLATFORM_TICK_HZ)

		self.tone_msg.data = [frequency, volume, duration]
		self.tone.publish(self.tone_msg)
