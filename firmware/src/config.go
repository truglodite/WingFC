package main

// WingFC Configuration
// All user-configurable parameters (and hardware mappings?)
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

// --- Servo Trim and Endpoints ---
// [default] servoXmin, servoXtrim, servoXmax = 1000, 1500, 2000
const (
	servo1min, servo1trim, servo1max = 886, 1498, 2114
	servo2min, servo2trim, servo2max = 989, 1498, 2012
	servo4min, servo4trim, servo4max = 1080, 1500, 2176
	servo5min, servo5trim, servo5max = 1000, 1500, 2000
	servo6min, servo6trim, servo6max = 1000, 1500, 2000
)

// --- Hardware Output Configuration ---
const (
	// Servo output frequency (default_0 = 50Hz, default_2 = 50)
	// FREQUENCY_0 for servos 1, 2, 4, and 5
	// FREQUENCY_2 for servo 6
	// Analog servos use 50Hz, digital servos may use 100Hz 250Hz 333Hz etc.
	SERVO_PWM_FREQUENCY_0 = 50
	SERVO_PWM_FREQUENCY_2 = 50

	// PWM ESC Frequency (default 50)
	// Common analog esc's use 50Hz. Another common ESC frequency is 400Hz
	ESC_PWM_FREQUENCY = 50

	// DShot ESC Configuration (default false)
	// Set USE_DSHOT = true to use DShot esc output, false to use PWM esc output
	USE_DSHOT = false

	// DSHOT rate: 150, 300, 600, 1200 (kHz). (default 300)
	// Please use <= 300... lower values are easier to bit-bang and adequate for fixed wing.
	DSHOT_RATE = 300

	// Microseconds of deadband around neutral to use for control stick inputs (default 3)
	DEADBAND = 3

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
	TuneParameterA, TuneParameterAmin, TuneParameterAmax = 1, 1.0, 3.0
	TuneParameterB, TuneParameterBmin, TuneParameterBmax = 2, 1.0, 3.0
	TuneParameterC, TuneParameterCmin, TuneParameterCmax = 3, 0.5, 2.0
	TuneParameterD, TuneParameterDmin, TuneParameterDmax = 0, 1.0, 3.0
)
