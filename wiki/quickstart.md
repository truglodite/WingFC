# WingFC Quickstart Guide
## config.go
User configurable parameters are generally located in the file `firmware\src\config.go`. You should read over this and edit `config.go` according to your needs before compiling and flashing to your FC.
### --- Protocol Settings ---
- `NumChannels = 12`

Users must select the number of channels that will be output by their specific radio receiver using the `NumChannels` parameter. The value should include all possible channels to be used, rather than just the channels utilized by the user. For example when using a CRSF receiver configured for 50hz packet rate using "wide" switch mode, this value should be set to 12 even if the user will only program 5 pilot inputs on their transmitter. The Protocol Settings section includes some helpful comments to guide users in setting an appropriate `NumChannels` for more commonly used radio configurations.

### --- Aircraft Type ---
- `AircraftType = 5`

WingFC supports servo mixing for a variety of fixed wing aircraft configurations. Use the `AircraftType` paramenter to configure WingFC for your needs. For example if you have a standard T-tail aircraft with one elevator servo, one rudder servo, and 2 aileron servos, use `AircraftType = 2`. The table below lists the currently available options.

Aircraft Type   |   Value
----------------|----------
Single aileron T tail | 1
Dual aileron T tail | 2
Single aileron V tail |3
Dual aileron V tail | 4
Elevon delta (with or without rudder) | 5

*Document User Servo nomenclature!!!*

switch AircraftType {
case 1: // Single aileron T tail
    servo1 = rollOutput
    servo2 = pitchOutput
    servo4 = yawOutput
    servo5 = 0
case 2: // Dual aileron T tail
    servo1 = rollOutput
    servo2 = pitchOutput
    servo4 = yawOutput
    servo5 = -rollOutput
case 3: // Single aileron V tail
    servo1 = rollOutput
    servo2 = pitchOutput + yawOutput - NEUTRAL_RX_VALUE
    servo4 = pitchOutput - yawOutput + NEUTRAL_RX_VALUE
    servo5 = 0
case 4: // Dual aileron V tail
    servo1 = rollOutput
    servo2 = pitchOutput + yawOutput - NEUTRAL_RX_VALUE
    servo4 = pitchOutput - yawOutput + NEUTRAL_RX_VALUE
    servo5 = -rollOutput
default: // Elevon delta
    servo1 = rollOutput + pitchOutput - NEUTRAL_RX_VALUE
    servo2 = -rollOutput + pitchOutput + NEUTRAL_RX_VALUE
    servo4 = yawOutput
    servo5 = 0
}

### --- Board Orientation ---
`orientation = 0`

WingFC supports mounting the flight controller in a variety of orientations on an aircraft for a more convenient installation. You must set the `orientation` parameter to match your specific installation. The default orientation (`orientation = 0`) for WingFC is defined as "USB port on top of the PCB and facing backward". If you have mounted your FC with a different orientation, you must carefully determine the correct `orientation` value to use as follows.

Imagine standing over the top of your aircraft as it rests on the ground, with the top of the wing facing up, and nose facing forward (away from you). Now imagine the FC in your hands with with the USB on top and facing toward you. If your installed FC has the USB on the bottom, you first flip the FC over along the aircraft's roll axis. Then if your USB is not facing toward you, you rotate the FC clockwise as needed to match how your FC is installed on the aircraft.

Using the number of degrees you had to rotate and whether or not you had to "flip" it over, select the correct value for `orientation` using the table below.

Orientation | Value
----|----
Default | 0
CW90 | 1
CW180 | 2
CW270 | 3
Flip | 4
Flip CW90 | 5
Flip CW180 | 6
Flip CW270 | 7

To verify you have set the correct orientation, first go through the `Receiver Channel Mapping ` and `Servo Reverse` sections below to configure your firmware for flashing and ensure your servos are reversed properly. For safety please disconnect your motor for the following procedures!

Power up your transmitter and aircraft and wait for WingFC to finish booting up (indicated by either a solid green or solid blue LED). Now enable "acro mode" by flipping the `ManualModeChannel` switch so the channel outputs high (~2000uSec). You should see a blue LED, indicating you are in acro mode.

Similar to the board orientation excercise, hold the aircraft in front of you, top facing up, tail facing toward you. Now give a rapid pitch up movement to the airframe. You should see the elevator surface(s) move upward in response to the gyro. Next give a rapid left yaw movement to the airframe, and you should see the rudder surface(s) move to the right in response to the gyro. Lastly, give a rapid left roll movement to the airframe. You should see the left aileron move down and right aileron move up in response to the gyro.

If any of this is not true for your aircraft, either `orientation` or `servoXreverse` parameters have not been set correctly. Do not attempt to fly your aircraft until it responds correctly to the gyro, or it will be uncontrollable and likely crash.

If you are not seeing any surface reactions when you move the airframe around in "acro mode", you can temporarily increase the values of `MAX_PITCH_RATE_DEG`, `MAX_ROLL_RATE_DEG`, and `MAX_YAW_RATE_DEG` for this test. This will make control surface reactions to gyro input more obvious for this part of bench testing. If you do increase these parameters to verify orientation, please set them back to defaults for your first test flight to help avoid an uncontrollable aircraft.

### --- Receiver Channel Mapping ----
This section of `config.go` is used to map your transmitter configuration for WingFC's input processing. The default mapping uses "AETR" mapping, with arming on channel 5, flight mode selection on channel 6, and optional in flight tuning controls on channels 8-11. If your transmitter is configured differently, you will have to edit this section to match. Note the values for each channel are indexed starting at 0. So transmitter channel 1 corresponds to a value of `0`, channel 2 is `1`, and so on.

### --- Servo Reverse ---
WingFC supports servo reversing for more convenient servo installation. Editing this section is best done after initial installation and bench testing on the aircraft. For safety please disconnect your motor for the following procedures!

First flash the FC with all of the above parameters configured for your aircraft. Install and connect your electronics, then power up your transmitter and aircraft on the bench. When WingFC has booted up and is receiving control inputs from your transmitter, you should see either a green or a blue LED lit up and control surfaces should respond to your stick inputs. If not, revisit your wiring and configuration of the above parameters until this is the case.

Similar to the orientation excercise, imagine standing behind your aircraft, looking down on it with the nose pointed away from you. Add up elevator input on your transmitter (typically right stick downward). Your elevator(s) or elevons should move upwards. If any of the servos move downwards, reverse that servo by setting `servoXreverse = true`. Next if you have yaw control surfaces, add left yaw input. Your yaw surface(s) should move to the left. If not, reverse the servos in the `Servo Reverse` section accordingly.

### --- Servo Trim and Endpoints ---
WingFC uses `servoXmin`, `servoXmax`, and `servoXtrim` parameters to configure servo endpoints and center trim on an aircraft. Refer to `servoX` nomenclature to determine which parameters are used to configure any installed servo. To setup min and max, power the aircraft up in manual mode, and move the sticks to full deflection in both directions. Adjust max and min values such that full surface deflection is acheived without mechanical binding. To setup trim, first fly the aircraft in manual mode, using the transmitter trims to achieve steady level flight at a preferred cruising speed with no or minimal stick input needed. After landing, use transmitter menus to determine the trimmed centerpoint.
// [default] servoXmin, servoXtrim, servoXmax = 1000, 1500, 2000
const (
	servo1min, servo1trim, servo1max = 886, 1498, 2114
	servo2min, servo2trim, servo2max = 989, 1498, 2012
	servo4min, servo4trim, servo4max = 1080, 1500, 2176
	servo5min, servo5trim, servo5max = 1000, 1500, 2000
	servo6min, servo6trim, servo6max = 1000, 1500, 2000
)

### --- Hardware Output Configuration ---
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
	DSHOT_RATE = 150

	// Microseconds of deadband around neutral to use for control stick inputs (default 3)
	DEADBAND = 3

	// RX output microseconds above which binary logic evaluates as true (arming and flight mode, default 1980)
	HIGH_RX_VALUE = 1980
)

### --- Flight Control Parameters ---
var (
	// Warning, parameters that are setup for in flight tuning will be overwritten accordingly.
	// Maximum desired pitch rate in degrees/sec (default 200)
	MAX_PITCH_RATE_DEG float64 = 200

	// Maximum desired roll rate in degrees/sec (default 500)
	MAX_ROLL_RATE_DEG float64 = 500

	// Maximum desired yaw rate in degrees/sec (default 100)
	MAX_YAW_RATE_DEG float64 = 100

	// Weighting for combining gyro/accel with input (default 0.5)
	PID_WEIGHT = .7

	// LPF alpha for gyro/accel fusion (default 0.2)
	LPF_ALPHA = 0.2

	// PID gains (P, I, D) for the roll, pitch, and yaw controllers
	pP, pI, pD = 2., 0.5, 0.01  // default 2., 0.5, 0.01
	rP, rI, rD = 2., 0.5, 0.01  // default 2., 0.5, 0.01
	yP, yI, yD = 1.0, 0.4, 0.01 // default 1., 0.4, 0.01
)

### --- In Flight Tuning Parameters ---
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
// 10: Max Pitch Rate
// 11: Max Roll Rate
// 12: Max Yaw Rate
// 13: PID Weight
// DO NOT set any tuning parameter more than once!!!
// TuneParameterXmax/min = max and min values available using full TuningChannelX range (988-2012)
const (
	TuneParameterA, TuneParameterAmin, TuneParameterAmax = 1, 0.5, 1.5
	TuneParameterB, TuneParameterBmin, TuneParameterBmax = 2, 0.5, 1.5
	TuneParameterC, TuneParameterCmin, TuneParameterCmax = 3, 0.2, 1.0
	TuneParameterD, TuneParameterDmin, TuneParameterDmax = 10, 200, 500
)