uint64 timestamp			# time since system start (microseconds)

uint8 RC_INPUT_SOURCE_UNKNOWN = 0
uint8 RC_INPUT_SOURCE_TCFMU_PPM = 1
uint8 RC_INPUT_SOURCE_TCIO_PPM = 2
uint8 RC_INPUT_SOURCE_TCIO_SPEKTRUM = 3
uint8 RC_INPUT_SOURCE_TCIO_SBUS = 4
uint8 RC_INPUT_SOURCE_TCIO_ST24 = 5
uint8 RC_INPUT_SOURCE_MAVLINK = 6
uint8 RC_INPUT_SOURCE_QURT = 7
uint8 RC_INPUT_SOURCE_TCFMU_SPEKTRUM = 8
uint8 RC_INPUT_SOURCE_TCFMU_SBUS = 9
uint8 RC_INPUT_SOURCE_TCFMU_ST24 = 10
uint8 RC_INPUT_SOURCE_TCFMU_SUMD = 11
uint8 RC_INPUT_SOURCE_TCFMU_DSM = 12
uint8 RC_INPUT_SOURCE_TCIO_SUMD = 13
uint8 RC_INPUT_SOURCE_TCFMU_CRSF = 14
uint8 RC_INPUT_SOURCE_TCFMU_GHST = 15

uint8 RC_INPUT_MAX_CHANNELS = 18 	# Maximum number of R/C input channels in the system. S.Bus has up to 18 channels.

uint64 timestamp_last_signal		# last valid reception time

uint8 channel_count			# number of channels actually being seen

int8 RSSI_MAX = 100
int32 rssi				# receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 100: full reception

bool rc_failsafe			# explicit failsafe flag: true on TX failure or TX out of range , false otherwise. Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
bool rc_lost				# RC receiver connection status: True,if no frame has arrived in the expected time, false otherwise. True usually means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems. Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.

uint16 rc_lost_frame_count		# Number of lost RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.
uint16 rc_total_frame_count		# Number of total RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.
uint16 rc_ppm_frame_length		# Length of a single PPM frame. Zero for non-PPM systems

uint8 input_source			# Input source
uint16[18] values			# measured pulse widths for each of the supported channels

int8 link_quality			# link quality. Percentage 0-100%. -1 = invalid
float32 rssi_dbm			# Actual rssi in units of dBm. NaN = invalid
