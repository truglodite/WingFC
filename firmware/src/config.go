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
	// Aircraft types
	// 1 = Single aileron T tail
	// 2 = Dual aileron T tail
	// 3 = Single aileron V tail
	// 4 = Dual aileron V tail confurtion
	// 5 = Elevon delta (with or without rudder)
	AircraftType = 5
)

// --- Board Orientation ---
const (
	// The default orientation is defined as "chip on top, usb port aft".
	// Orientations are relative to a fixed frame of reference (the airframe).
	// Rotate CW only, or flip first then rotate CW (rotate with chip facing down).
	// Flip is defined along roll axis, cwX is defined along yaw axis.
	// default=0, cw90=1, cw180=2, cw270=3, flip=4, flipcw90=5, flipcw180=6, flipcw270=7
	orientation = 6
)

// --- Receiver Channel Mapping ----
const (
	AileronChannel    = 0  // CH1
	ElevatorChannel   = 1  // CH2
	ThrottleChannel   = 2  // CH3
	RudderChannel     = 3  // CH4
	ArmChannel        = 4  // CH5
	ManualModeChannel = 6  // CH7
	TuningChannelA    = 7  // CH8
	TuningChannelB    = 8  // CH9
	TuningChannelC    = 9  // CH10
	TuningChannelD    = 10 // CH11
)

// --- Servo Reverse ---
const (
	// Set each to true if the servo should be reversed (default = false)
	servo1reverse = true
	servo2reverse = false
	servo4reverse = false
	servo5reverse = false
	servo6reverse = false
)

// --- Servo Trim ---
// default 1500
const (
	servo1trim = 1498
	servo2trim = 1498
	servo4trim = 1500
	servo5trim = 1500
	servo6trim = 1500
)

// --- Servo Endpoints ---
// default [1000,2000]
const (
	servo1min = 886
	servo1max = 2114
	servo2min = 989
	servo2max = 2012
	servo4min = 1080
	servo4max = 2176
	servo5min = 1000
	servo5max = 2000
	servo6min = 1000
	servo6max = 2000
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
	MAX_PITCH_RATE_DEG = 200

	// Maximum desired roll rate in degrees/sec (default 500)
	MAX_ROLL_RATE_DEG = 500

	// Maximum desired yaw rate in degrees/sec (default 100)
	MAX_YAW_RATE_DEG = 100

	// Weighting for combining gyro/accel with input (default 0.5)
	PID_WEIGHT = .7

	// LPF alpha for gyro/accel fusion (default 0.2)
	LPF_ALPHA = 0.2
)

var (
	// PID gains (P, I, D) for the roll, pitch, and yaw controllers
	pP, pI, pD = 2., 0.5, 0.01  // default 2., 0.5, 0.01
	rP, rI, rD = 2., 0.5, 0.01  // default 2., 0.5, 0.01
	yP, yI, yD = 1.0, 0.4, 0.01 // default 1., 0.4, 0.01
)

// --- In Flight Tuning Parameters ---
// In flight tune parameters override PID values set above.
// TuneParameterX = Parameter to tune
// 0: None (default)
// 1: Pitch P
// 2: Roll P
// 3: Yaw P
// 4: Pitch I
// 5: Roll I
// 6: Yaw I
// 7: Pitch D
// 8: Roll D
// 9: Yaw D
// DO NOT set any tuning parameter more than once!!!
// TuneParameterXmax/min = max and min values available using full TuningChannelX range (988-2012)
const (
	TuneParameterA    = 1
	TuneParameterAmin = 1.0
	TuneParameterAmax = 3.0

	TuneParameterB    = 2
	TuneParameterBmin = 1.0
	TuneParameterBmax = 3.0

	TuneParameterC    = 3
	TuneParameterCmin = 0.5
	TuneParameterCmax = 2.0

	TuneParameterD    = 0
	TuneParameterDmin = 1.0
	TuneParameterDmax = 3.0
)
