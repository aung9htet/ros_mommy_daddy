from colour import Color
from miro_ros_interface import MiRoPublishers

miro_pub = MiRoPublishers()


class IlluminationActions:
	@staticmethod
	def pulse(pulses, colour, step: int = 5, speed: int = 100):
		# Convert colour name to full RGB hex value (removing # character)
		# TODO: Skip this if provided colour is already an RGB value
		rgb = Color(colour).hex_l[1:]

		def pub_colour(alpha):
			# Append colour to alpha value for complete ARGB word
			argb = int(hex(alpha) + rgb, 16)

			# TODO: Add code to specify individual lights
			miro_pub.pub_illum(all=argb)

			# Sleep time is just reciprocal of speed
			miro_pub.ros_sleep(1 / speed)

		for _ in range(pulses):
			for a in range(0, 255, step):
				pub_colour(a)
			for a in range(255, 0, -step):
				pub_colour(a)

	# TODO: Create 'sweep' action that sweeps a pulse forwards or backwards


class CosmeticActions:
	@staticmethod
	def wag(wags, speed: int = 5):
		for _ in range(wags):
			miro_pub.pub_cosmetic_joints(wag=0)
			miro_pub.ros_sleep(1 / speed)
			miro_pub.pub_cosmetic_joints(wag=1)
			miro_pub.ros_sleep(1 / speed)

	@staticmethod
	def blink(blinks, speed: int = 5):
		for _ in range(blinks):
			miro_pub.pub_cosmetic_joints(eyes=0)
			miro_pub.ros_sleep(1 / speed)
			miro_pub.pub_cosmetic_joints(eyes=1)
			miro_pub.ros_sleep(1 / speed)
