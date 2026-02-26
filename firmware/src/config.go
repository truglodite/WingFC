package main

// WingFC Configuration
// All user-configurable parameters and hardware mappings
//
// All the configurable values are defined here, making it easy to tune the
// flight controller without changing the main application logic.

// --- Protocol Settings ---
const (
	// Number of supported RC channels
	// elrs 50hz packet rate, wide switch mode = 12ch
	// elrs 100hz packet rate, wide switch mode = 8ch
	// elrs packet rates 150hz and higher show crc errors
	NumChannels = 12
)

// --- Receiver Configuration ---
// --- Channel Mappings ---

const (
	AileronChannel    = 0 // CH1
	ElevatorChannel   = 1 // CH2
	ThrottleChannel   = 2 // CH3
	RudderChannel     = 3 // CH4
	ArmChannel        = 4 // CH5
	ManualModeChannel = 6 // CH7
)

// --- Aircraft Type Configuration ---
const (
	// Set only one to true
	TYPE_1 = false // Single aileron T tail configuration
	TYPE_2 = false // Dual aileron T tail configuration
	TYPE_3 = false // Single aileron V tail configuration
	TYPE_4 = false // Dual aileron V tail configuration
	TYPE_5 = true  // Elevon delta configuration
)

// --- PWM Configuration ---
const (
	// Analog servo frequency 50Hz
	// Digital servo frequency 100Hz 250Hz 333Hz etc.
	SERVO_PWM_FREQUENCY = 50

	// ESC frequency set at analog servo frequency 50Hz
	// another common ESC frequency is 400Hz
	ESC_PWM_FREQUENCY = 50

	// DShot configuration: enable DShot digital ESC protocol and choose rate.
	// Set USE_DSHOT to true to use DShot instead of PWM for ESC outputs.
	// Default is false to use legacy PWM.
	USE_DSHOT = false

	// Supported values: 150, 300, 600, 1200 (kHz). Lower values are easier to bit-bang.
	DSHOT_RATE = 300

	// Deadband around neutral for stick input
	DEADBAND = 10

	// High Rx channel value for arming/calibration
	HIGH_RX_VALUE = 1980
)

// --- Flight Control Parameters ---
const (
	// Maximum desired pitch rate in degrees/sec
	MAX_PITCH_RATE_DEG = 600 //trug 200

	// Maximum desired roll rate in degrees/sec
	MAX_ROLL_RATE_DEG = 600

	// Maximum desired yaw rate in degrees/sec
	MAX_YAW_RATE_DEG = 100

	// Weighting for combining gyro/accel with input
	PID_WEIGHT = .7 //trug 0.5

	// LPF alpha for gyro/accel fusion
	LPF_ALPHA = 0.2

	// PID gains (P, I, D) for the pitch and roll controllers
	//pP, pI, pD = 1., 0.1, 0.01  //trug defaults
	//rP, rI, rD = 1., 0.1, 0.01
	pP, pI, pD = 2., 0.5, 0.01
	rP, rI, rD = 2., 0.5, 0.01
	// Yaw PID gains
	yP, yI, yD = 1.5, 0.4, 0.01
)
