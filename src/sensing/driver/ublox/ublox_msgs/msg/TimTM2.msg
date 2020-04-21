# TIM-TM2 (0x0D, 0x03)
# Time mark data 
#
# Description for details.
# 
# Supported on:
#  - u-blox 8 / u-blox M8 with protocol version 22 (only with Timing Products)
#

uint8 CLASS_ID = 13 
uint8 MESSAGE_ID = 3

uint8 ch                     # Channel (i.e. EXTINT) upon which the pulse was measured

uint8 flags                   # Bitmask [newRisingEdge, time, utc, timeBase, , newFallingEdge, run, mode]
uint8 FLAGS_MODE_RUNNING = 1  # single = 0, running = 1
uint8 FLAGS_RUN = 2		# armed = 0, stopped = 1
uint8 FLAGS_NEWFALLINGEDGE = 4 	# new falling edge detected
uint8 FLAGS_TIMEBASE_GNSS = 8	# 0 = time base is receiver time, 1 = time base is GNSS Time (according to the configuration in CFG-TP5 for tpldx= 0)
uint8 FLAGS_TIMEBASE_UTC = 16	# Time Base is UTC (the variant according to the configuration in CFG-NAV5
uint8 FLAGS_UTC_AVAIL = 32	# 0 = utc not available, 1 = utc available
uint8 FLAGS_TIME_VALID = 64	# 0 = time is not valid, 1 time is valid
uint8 FLAGS_NEWRISINGEDGE = 128	# new rising edge detected

uint16 risingEdgeCount		# rising edge count
uint16 wnR			# week number of last rising edge
uint16 wnF			# week number of last falling edge
uint32 towMsR			# Tow of rising edge 
uint32 towSubMsR		# Millisecond Fraction of Tow of rising edge in nanoseconds
uint32 towMsF			# tow of falling edge
uint32 towSubMsF		# millisecond fraction of tow of falling edge in nanoseconds
uint32 accEst			# Accuracy estimate
