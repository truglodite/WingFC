package main

// WingFC Configuration
// All user-configurable parameters and hardware mappings
//
// All the configurable values are defined here, making it easy to tune the
// flight controller without changing the main application logic.

// --- Protocol Settings ---
const (
	// Number of supported RC channels (default 12)
	// elrs 50hz, 150hz, 250hz, 500hz packet rates w/ wide switch mode = 12ch
	// elrs 100hz/full, 333hz/full packet rates = 8ch or 16ch

	NumChannels = 12
)

// --- Aircraft Type ---
const (
	// Set only one to true (default TYPE_5 = true)
	TYPE_1 = false // Single aileron T tail configuration
	TYPE_2 = false // Dual aileron T tail configuration
	TYPE_3 = false // Single aileron V tail configuration
	TYPE_4 = false // Dual aileron V tail configuration
	TYPE_5 = true  // Elevon delta configuration
)

// --- Board Orientation ---
const (
	// default defined as chip on top, usb to the left.?
	// Flip is defined along roll axis, cwX defined along yaw axis; just yaw, or flip first then yaw.
	// default=0, cw90=1, cw180=2, cw270=3, flip=4, flipcw90=5, flipcw180=6, flipcw270=7
	orientation = 0
)

// --- Receiver Channel Mapping ----
const (
	AileronChannel    = 0 // CH1
	ElevatorChannel   = 1 // CH2
	ThrottleChannel   = 2 // CH3
	RudderChannel     = 3 // CH4
	ArmChannel        = 4 // CH5
	ManualModeChannel = 6 // CH7
)

// --- Servo reverse ---
const (
	// Set each to true if the servo should be reversed
	servo1reverse = false
	servo2reverse = false
	servo4reverse = false
	servo5reverse = false
	servo6reverse = false
)

// --- Hardware Output Configuration ---
const (
	// Servo output frequency (default 50Hz)
	// Analog servos use 50Hz, digital servos may use 100Hz 250Hz 333Hz etc.
	SERVO_PWM_FREQUENCY = 50

	// ESC Frequency (default 50)
	// Common analog esc's use 50Hz. Another common ESC frequency is 400Hz
	ESC_PWM_FREQUENCY = 50

	// DShot ESC Configuration (default false)
	// Set USE_DSHOT = true to enable DShot output, false to enable PWM output
	USE_DSHOT = false

	// DSHOT rate: 150, 300, 600, 1200 (kHz). (default 300)
	// Please use <= 300... lower values are easier to bit-bang.
	DSHOT_RATE = 300

	// Microseconds of deadband around neutral to use for control stick inputs (default 10)
	DEADBAND = 10

	// RX output microseconds above which binary logic evaluates as true (arming and flight mode, default 1980)
	HIGH_RX_VALUE = 1980
)

// --- Flight Control Parameters ---
const (
	// Maximum desired pitch rate in degrees/sec (default 200)
	MAX_PITCH_RATE_DEG = 600

	// Maximum desired roll rate in degrees/sec (default 600)
	MAX_ROLL_RATE_DEG = 600

	// Maximum desired yaw rate in degrees/sec (default 100)
	MAX_YAW_RATE_DEG = 100

	// Weighting for combining gyro/accel with input (default 0.5)
	PID_WEIGHT = .7

	// LPF alpha for gyro/accel fusion (default 0.2)
	LPF_ALPHA = 0.2

	// PID gains (P, I, D) for the roll, pitch, and yaw controllers
	pP, pI, pD = 2., 0.5, 0.01  // default 2., 0.5, 0.01
	rP, rI, rD = 2., 0.5, 0.01  // default 2., 0.5, 0.01
	yP, yI, yD = 1.0, 0.4, 0.01 // default 1., 0.4, 0.01
)
