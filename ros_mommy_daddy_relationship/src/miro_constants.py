import numpy as np

#####
# Audio
# See http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS
BLOCK_SAMPLES = 500         # Sample rate of 20kHz and packages arriving at 40Hz == 500 samples per package
BLOCK_RATE = 40             # Package arrival rate in Hz
MICS = 4                    # Number of microphones on the robot
TONE_FREQUENCY = {          # Published tone frequency range
	'min': 50,
	'max': 2000
}
TONE_VOLUME = {             # Published tone volume range
	'min': 0,
	'max': 255
}

#####
# Perception
# Visual salience map dimensions
PRI = {
	'width' : 178,
	'height': 100
}
# Audio salience map dimensions
PRIW = {
	'width' : 256,
	'height': 1
}

#####
# Basic colours (as https://pypi.org/project/colour/ is not installed on MiRo)
BGR_TUPLE = {
	'red'    : (000, 000, 255),
	'green'  : (000, 255, 000),
	'blue'   : (255, 000, 000),
	'yellow' : (000, 255, 255),
	'cyan'   : (255, 255, 000),
	'magenta': (255, 000, 255),
	'white'  : (255, 255, 255),
	'black'  : (000, 000, 000),
}
ARGB_WORD = {
	'red'    : 0xFFFF0000,
	'green'  : 0xFF00FF00,
	'blue'   : 0xFF0000FF,
	'yellow' : 0xFFFFFF00,
	'cyan'   : 0xFF00FFFF,
	'magenta': 0xFFFF00FF,
	'white'  : 0xFFFFFFFF,
	'black'  : 0x00000000,
	'off'    : 0x00000000,
}

#####
# Image distortion
# (It may be possible to derive better values by performing your own calibration!)
MTX = np.array([
	[1.04358065e+03, 0, 3.29969935e+02],
	[0, 1.03845278e+03, 1.68243114e+02],
	[0, 0, 1]
])
DIST = np.array([[-3.63299415e+00, 1.52661324e+01, -7.23780207e-03, -7.48630198e-04, -3.20700124e+01]])
FOCAL_LENGTH = 330
