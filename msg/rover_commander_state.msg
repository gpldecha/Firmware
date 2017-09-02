# Main state, i.e. what user wants. Controlled by RC or from ground station via telemetry link.
uint8 MAIN_STATE_MANUAL = 0
uint8 MAIN_STATE_OFFBOARD = 1
uint8 MAIN_STATE_AUTONOMOUS = 2
uint8 main_state		    	# main state machine
