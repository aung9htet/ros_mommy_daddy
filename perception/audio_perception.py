import numpy as np
import miro_constants as con

# TODO: Rewrite to take data as outputted from MRI


class FrequencyPerception:
	def __init__(self, frequencies, buffer_length=20, frequency_threshold=0.2):
		# Frequencies to detect should be multiples of 40
		# Audio frequency parameters
		if not isinstance(frequencies, list):
			frequencies = [frequencies]
		self.frequencies = frequencies

		self.buffer_length = buffer_length      # At 40Hz a buffer of 20 == 0.5s
		self.freq_thresh = frequency_threshold  # Minimum power at which to register frequency detection

		# TODO: Allow frequencies to be defined when creating class object
		# self.frequencies = [440, 600]       # Frequencies to detect (multiples of 40 recommended)
		# self.buffer_length = 20             # At 40Hz a buffer of 20 == 0.5s
		# self.f_thresh = 0.2                 # Minimum power at which to register frequency detection
		self.c_power = 10                    # Frequency power at which MiRo is 'close' to audio source

		# FFT array positions of defined frequencies
		# self.f_pos = [int(frq / con.BLOCK_RATE) for frq in self.frequencies]
		self.freq_position = [int(frq / con.BLOCK_RATE) for frq in self.frequencies]
		# self.f_buffer = np.zeros((len(self.frequencies), self.buffer_length, con.MICS))
		self.freq_buffer = np.zeros((len(self.frequencies), self.buffer_length, con.MICS))
		# self.f_mean = np.zeros((len(self.frequencies), con.MICS))
		self.freq_mean_power = np.zeros((len(self.frequencies), con.MICS))

	def frequency_power(self, data):
		# Return an instantaneous measure of the power of each frequency from each microphone

		# Mic data must be list
		if not isinstance(data, list):
			data = [data]

		# Perform Fast Fourier Transform on audio sample from each microphone, dropping mirrored and imaginary portions
		fft_data = [np.abs(np.fft.fft(mic))[: con.BLOCK_SAMPLES / 2] for mic in data]

		# Get power of specified frequencies from each microphone
		freq_power = np.array([
			[fft_data[mic][self.freq_position[frq]] for mic, _ in enumerate(fft_data)]
			for frq, _ in enumerate(self.frequencies)
		])

		return freq_power

	# def locate_frequencies(self, data):
	def locate_frequencies(self, left='', right='', **mics):
		# TODO: Allow function to accomodate being called with data in different order
		# TODO: -OR- allow function to be called without specifying data and function will fetch data itself

		# Can maybe call using locate_frequencies(**mics?) https://stackoverflow.com/questions/51751929/python-can-i-pass-a-defined-dictionary-to-kwargs

		# if all(k in kwargs for k in ("foo", "bar")):
		data = [left, right]

		# Get instantaneous frequency power
		freq_power = self.frequency_power(data)

		# TODO: Update this to specify left / right instead of 0/1, and take care of freq_buffer being 4 mics if we only pass data for 2 etc
		# Append frequency power from this sample to the buffer and store the mean power to smooth out noise
		for frq, mic in enumerate(freq_power):
			# Shift frequency buffer and append new values
			self.freq_buffer[frq] = np.roll(self.freq_buffer[frq], 1, axis=0)
			self.freq_buffer[frq][0] = freq_power[frq]

			for m, _ in enumerate(mic):
				# Calculate mean power of each frequency across entire buffer
				self.freq_mean_power[frq][m] = np.mean(self.freq_buffer[frq][:, m])

		# Get leftwards bias (i.e. positive == left, negative == right) for each frequency if mean values are above threshold
		f_left = np.zeros(len(freq_power))
		for frq, _ in enumerate(freq_power):
			if self.freq_mean_power[frq][0] > self.freq_thresh and self.freq_mean_power[frq][1] > self.freq_thresh:
				# Leftwards bias is just mean left power - mean right power
				f_left[frq] = self.freq_mean_power[frq][0] - self.freq_mean_power[frq][1]
			else:
				f_left[frq] = None

		# TODO: Get front/back bias from centre/tail mics (needs further calibration due to differing sensitivities)

		return f_left

	# def isClose(self, data):
	# 	data = self.shape_data(data)
	# 	freq_power = self.do_fft(data)
	#
	# 	close_val = np.zeros(len(freq_power))
	#
	# 	for frq, _ in enumerate(freq_power):
	# 		# Use centre mic to gauge closeness
	# 		close_val[frq] = freq_power[frq][2]
	# 		if close_val[frq] >= self.c_power:
	# 			close_val[frq] = 1
	# 		else:
	# 			close_val[frq] = 0
	#
	# 	return close_val




