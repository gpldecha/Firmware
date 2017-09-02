# If you change the order, add or remove arming_state_t states make sure to update the arrays
# in state_machine_helper.cpp as well.
uint8 ARMING_STATE_ARMED = 1
uint8 ARMING_STATE_REBOOT = 2
uint8 ARMING_STATE_MAX = 3

# Navigation state, i.e. "what should vehicle do".
uint8 NAVIGATION_STATE_MANUAL = 0		# Manual mode
uint8 NAVIGATION_STATE_OFFBOARD = 1
uint8 NAVIGATION_STATE_AUTONOMOUS = 2
uint8 NAVIGATION_STATE_MAX = 3

uint8 RC_IN_MODE_DEFAULT = 0
uint8 RC_IN_MODE_OFF = 1
uint8 RC_IN_MODE_GENERATED = 2

# state machine / state of vehicle.
# Encodes the complete system state and is set by the commander app.
uint8 nav_state				# set navigation state machine to specified value
uint8 arming_state			# current arming state
bool failsafe				# true if system is in failsafe state

uint8 system_type			# system type, contains mavlink MAV_TYPE
uint32 system_id			# system id, contains MAVLink's system ID field
uint32 component_id			# subsystem / component id, contains MAVLink's component ID field

bool rc_signal_lost				# true if RC reception lost
uint8 rc_input_mode				# set to 1 to disable the RC input, 2 to enable manual control to RC in mapping.

bool data_link_lost				# datalink to GCS lost
uint8 data_link_lost_counter			# counts unique data link lost events
bool engine_failure				# Set to true if an engine failure is detected
bool engine_failure_cmd				# Set to true if an engine failure mode is commanded


# see SYS_STATUS mavlink message for the following
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health
