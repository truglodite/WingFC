package main

import (
	// "fmt"
	"machine"
	"math"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

// Version of the flight controller software.
const Version = "0.2.3"

// Global variables for hardware interfaces, controllers, and filters.
var (
	// Hardware interfaces
	uart     = machine.DefaultUART
	i2c      = machine.I2C0
	lsm      *lsm6ds3tr.Device
	watchdog = machine.Watchdog
	redLED   = machine.LED_RED
	greenLED = machine.LED_GREEN
	blueLED  = machine.LED_BLUE

	// PWM controllers and channels
	pwm0          = machine.PWM0
	pwm1          = machine.PWM1
	pwmCh1        uint8
	pwmCh2        uint8
	pwmCh3        uint8
	pwmCh4        uint8
	pwmCh5        uint8
	pwmCh6        uint8
	escPin        machine.Pin
	servoPeriodNs uint64
	escPeriodNs   uint64

	// Control system components
	pitchPID       *PIDController
	rollPID        *PIDController
	yawPID         *PIDController
	dt             = 0.01
	kf             *KalmanFilter
	imuData        IMU
	imuYawMeas     float64
	imuOrientation = orientation

	// IMU calibration
	accelXSum, accelYSum, accelZSum, accelBiasX, accelBiasY, accelBiasZ float64 = 0., 0., 0., 0., 0., 0.
	gyroXSum, gyroYSum, gyroZSum, gyroBiasX, gyroBiasY, gyroBiasZ       float64 = 0., 0., 0., 0., 0., 0.
	xA, yA, zA, xG, yG, zG                                              int32
	desiredPitchRate, desiredRollRate, desiredYawRate                   float64
	pitchOutput, rollOutput, yawOutput                                  float64
	escPulse                                                            uint32

	// RC Channels
	Channels        [NumChannels]uint16
	lastFlightState flightState
	LastPacketTime  time.Time
	calibStartTime  time.Time
	armed           bool
	manualMode      bool
	err             error
)

// Define constants for sensor value conversions and PWM.
const (
	// Convert sensor values to radians for calculations
	microGToMS2    = 9.80665 / 1e6
	microDPSToRadS = math.Pi / (180 * 1e6)

	// PWM pulse width constants
	MIN_PULSE_WIDTH_US = 988
	MAX_PULSE_WIDTH_US = 2012

	MIN_RX_VALUE     = 988
	MAX_RX_VALUE     = 2012
	NEUTRAL_RX_VALUE = 1500

	// Calculated constants for PID control
	MAX_ROLL_RATE  = MAX_ROLL_RATE_DEG * math.Pi / 180
	MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * math.Pi / 180
	MAX_YAW_RATE   = MAX_YAW_RATE_DEG * math.Pi / 180

	// --- Hardware Mappings ---
	PWM_CH1_PIN = machine.D0 // Servo 1
	PWM_CH2_PIN = machine.D1 // Servo 2
	PWM_CH3_PIN = machine.D2 // Servo 3
	PWM_CH4_PIN = machine.D3 // Servo 4
	PWM_CH5_PIN = machine.D4 // Servo 5
	PWM_CH6_PIN = machine.D5 // Servo 6

	// Fail-safe constants
	// for CSRF, we need to wait at least 1second
	FAILSAFE_TIMEOUT_MS = 1000

	// State machine states
	CALIBRATION flightState = iota
	FLIGHT_MODE
	FAILSAFE
)

type flightState int

// main is the entry point for the TinyGo program.
func main() {

	//time.Sleep(2 * time.Second) // Wait for hardware to stabilize
	println("WingFC Flight Controller - Version", Version)
	println("A TinyGo Flight Controller for Flying Wing Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	println("Initializing...")

	// configure the onboard RGB LED (Low=on, High=off)
	redLED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	greenLED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	blueLED.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// --- Hardware Setup ---
	uart.Configure(machine.UARTConfig{
		BaudRate: BAUD_RATE,
		TX:       machine.UART_TX_PIN,
		RX:       machine.UART_RX_PIN,
	})
	println("UART configured for receiver.")

	setLED(1) // G for servo config

	var retries = 0
servoPWMInit:
	servoPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY,
	}
	if err := pwm0.Configure(servoPWMConfig); err != nil {
		setLED(4) // RG on pwm0 init error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto servoPWMInit
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: Servo PWM Init Failed")
		return
	}
	// Reset retries for the next component
	setLED(2) // G for servo inits
	retries = 0
servoCh1Init:
	servoPeriodNs = servoPWMConfig.Period
	pwmCh1, err = pwm0.Channel(PWM_CH1_PIN)
	if err != nil {
		setLED(6) // GB on servo init error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto servoCh1Init
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: Servo PWM Channel 1 Init Failed")
		return
	}
	// Reset retries for the next component
	setLED(2)
	retries = 0
servoCh2Init:
	pwmCh2, err = pwm0.Channel(PWM_CH2_PIN)
	if err != nil {
		setLED(6) // GB on servo error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto servoCh2Init
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: Servo PWM Channel 2 Init Failed")
		return
	}
	// Reset retries for the next component
	setLED(2)
	retries = 0
servoCh4Init:
	pwmCh4, err = pwm0.Channel(PWM_CH4_PIN)
	if err != nil {
		setLED(6) // GB on servo error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto servoCh4Init
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: Servo PWM Channel 4 Init Failed")
		return
	}
	// Reset retries for the next component
	setLED(2)
	retries = 0
servoCh5Init:
	pwmCh5, err = pwm0.Channel(PWM_CH5_PIN)
	if err != nil {
		setLED(6) // GB on servo error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto servoCh5Init
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: Servo PWM Channel 5 Init Failed")
		return
	}
servoCh6Init:
	pwmCh6, err = pwm0.Channel(PWM_CH6_PIN)
	if err != nil {
		setLED(6) // GB on servo error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto servoCh6Init
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: Servo PWM Channel 5 Init Failed")
		return
	}
	// set servos 1, 2, 4, 5, and 6 to subtrim values
	setServo(servo1trim, servo2trim, servo4trim, servo5trim, servo6trim)
	println("PWM configured for servos.")

	// ESC init right away to avoid leaving some esc's in a bad state
	// Reset retries for the next component
	setLED(1) // R for esc init
	retries = 0
escInit:
	if USE_DSHOT {
		escPin = PWM_CH3_PIN
		escPin.Configure(machine.PinConfig{Mode: machine.PinOutput}) // error checking is apparently not needed for PinConfig
		setESC(MIN_PULSE_WIDTH_US)
		println("DShot configured for ESC.")
	} else {
		escPWMConfig := machine.PWMConfig{
			Period: machine.GHz * 1 / ESC_PWM_FREQUENCY,
		}
		if err = pwm1.Configure(escPWMConfig); err != nil {
			setLED(7) // RGB on esc init error
			retries++
			if retries < 5 {
				time.Sleep(100 * time.Millisecond)
				goto escInit
			}
			// Fallback or panic if max retries exceeded
			println("CRITICAL: Servo PWM Channel 5 Init Failed")
			return
		}
		escPeriodNs = escPWMConfig.Period
		pwmCh3, err = pwm1.Channel(PWM_CH3_PIN)
		if err != nil {
			setLED(7) // RGB on esc pwm init error
			retries++
			if retries < 5 {
				time.Sleep(100 * time.Millisecond)
				goto escInit
			}
			// Fallback or panic if max retries exceeded
			println("CRITICAL: ESC PWM Init Failed")
			return
		}
		// need to add way to calibrate pwm esc's
		setESC(MIN_PULSE_WIDTH_US)
		println("PWM configured for ESC.")
	}

	i2c.Configure(machine.I2CConfig{
		Frequency: 400 * machine.KHz,
	})
	println("I2C configured for IMU.")
	setLED(3) // B for IMU init
	retries = 0

	// --- IMU Setup ---
imuInit:
	lsm = lsm6ds3tr.New(i2c)
	err = lsm.Configure(lsm6ds3tr.Configuration{
		AccelRange:      lsm6ds3tr.ACCEL_8G,
		AccelSampleRate: lsm6ds3tr.ACCEL_SR_416,
		GyroRange:       lsm6ds3tr.GYRO_1000DPS,
		GyroSampleRate:  lsm6ds3tr.GYRO_SR_416,
	})
	if err != nil {
		retries++
		setLED(4) // RG for imu init error
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto imuInit
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: IMU Init Failed")
		return
	}
	// Reset retries for the next component
	setLED(1) // red for IMU check
	retries = 0

imuCheck:
	if !lsm.Connected() {
		setLED(5) // RB for imu check error
		retries++
		if retries < 5 {
			time.Sleep(100 * time.Millisecond)
			goto imuCheck
		}
		// Fallback or panic if max retries exceeded
		println("CRITICAL: IMU Not Connected")
		return
	}
	setLED(0) // OFF after boot checks

	println("LSM6DS3TR IMU configured and initialized.")

	// --- Filter and Controller Setup ---
	kf = NewKalmanFilter(dt)
	pitchPID = NewPIDController(pP, pI, pD)
	rollPID = NewPIDController(rP, rI, rD)
	yawPID = NewPIDController(yP, yI, yD)
	println("Control system initialized.")

	// Calibrate gyro to find bias
	println("Initial calibration")
	println("Calibrating Gyro... Keep gyro still!")
	// Keep outputs at neutral and ESC at zero
	setServo(servo1trim, servo2trim, servo4trim, servo5trim, servo6trim)
	setESC(MIN_PULSE_WIDTH_US)
	calibrate()

	// --- Watchdog Setup ---
	watchdog.Configure(machine.WatchdogConfig{
		TimeoutMillis: 1000,
	})

	flightState := FLIGHT_MODE
	lastFlightState = FLIGHT_MODE

	// Start the goroutine to read receiver packets asynchronously.
	go readReceiver(packetChan)

	// ticker to run the control loop at a fixed frequency matching Kalman filter.
	ticker := time.NewTicker(time.Duration(dt * float64(time.Second)))
	defer ticker.Stop()

	watchdog.Start()

	// Main application loop using select.
	// --- Main Loop ---
	for {

		select {
		case packet := <-packetChan:
			LastPacketTime = time.Now()
			// A complete packet has been received.
			processReceiverPacket(packet)
			//println("Received and processed a new receiver packet.")

		default:
			// Control loop at fixed intervals
			<-ticker.C

			// Always check for failsafe condition before the state machine logic
			// This provides a quick response to signal loss
			if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE {
				flightState = FAILSAFE
			}

			// Read and process IMU data every loop to have the freshest data available.
			readLSMData()
			processLSMData()

			// The state machine from previous versions is now the default case
			switch flightState {

			case FLIGHT_MODE:
				// Switch to armed mode if CH5 is high
				// Check for arm/disarm first every loop
				if Channels[ArmChannel] <= HIGH_RX_VALUE {
					//println("Disarmed.")
					setLED(2) // G while disarmed
					armed = false
				} else {
					//println("Armed!")
					setLED(3) // B while armed
					armed = true
				}

				// Check for manual mode every loop
				if Channels[ManualModeChannel] <= HIGH_RX_VALUE {
					//println("Manual Mode")
					manualMode = true
				} else {
					//println("Stab Mode")
					manualMode = false
				}

				// Handle failsafe and manual mode checks within the flight loop
				if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS {
					flightState = FAILSAFE
					break
				}

				// In stabilized mode, use IMU, Kalman filter and PID controllers to stabilize the aircraft.

				// Use the Kalman filter to fuse sensor data and get a stable attitude estimate.
				kf.Predict(imuData.GyroX, imuData.GyroY, imuData.GyroZ)
				// Integrate gyro Z to produce a yaw angle measurement (simple dead-reckoning)
				imuYawMeas += imuData.GyroZ * dt
				kf.Update(imuData.Pitch, imuData.Roll, imuYawMeas)

				// Trug: In case near crash disarm we may still want directional control.
				// In armed mode, use RC inputs to set desired rates.
				// Get desired pitch, roll, and yaw rates from the RC receiver.
				desiredPitchRate = mapRange(float64(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
				desiredRollRate = mapRange(float64(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
				// Standard RX Z is opposite of IMU Z
				desiredYawRate = mapRange(float64(Channels[RudderChannel]), MAX_RX_VALUE, MIN_RX_VALUE, -MAX_YAW_RATE, MAX_YAW_RATE)

				// Apply deadband to avoid small unwanted movements
				if math.Abs(desiredPitchRate) < DEADBAND*math.Pi/180 {
					desiredPitchRate = 0
				}
				if math.Abs(desiredRollRate) < DEADBAND*math.Pi/180 {
					desiredRollRate = 0
				}
				if math.Abs(desiredYawRate) < DEADBAND*math.Pi/180 {
					desiredYawRate = 0
				}

				// Calculate the error for PID controllers.
				pitchError := desiredPitchRate - imuData.GyroY
				rollError := desiredRollRate - imuData.GyroX
				yawError := desiredYawRate - imuData.GyroZ

				// handle manual mode
				if manualMode == false {
					// Update PID controllers and get the control outputs.
					pitchOutput = pitchPID.Update(pitchError, dt) * PID_WEIGHT
					rollOutput = rollPID.Update(rollError, dt) * PID_WEIGHT
					yawOutput = yawPID.Update(yawError, dt) * PID_WEIGHT
				} else { // use rc inputs if in manual mode
					pitchPID.integral = 0 // reset integral term in manual mode to prevent windup
					rollPID.integral = 0
					yawPID.integral = 0
					pitchOutput = desiredPitchRate
					rollOutput = desiredRollRate
					yawOutput = desiredYawRate
				}

				// Convert control outputs to servo pulse widths.
				rollOutput = mapRange(float64(rollOutput), -MAX_ROLL_RATE, MAX_ROLL_RATE, float64(MIN_PULSE_WIDTH_US), float64(MAX_PULSE_WIDTH_US))
				pitchOutput = mapRange(float64(pitchOutput), -MAX_ROLL_RATE, MAX_ROLL_RATE, float64(MIN_PULSE_WIDTH_US), float64(MAX_PULSE_WIDTH_US))
				yawOutput = mapRange(float64(yawOutput), -MAX_YAW_RATE, MAX_YAW_RATE, float64(MIN_PULSE_WIDTH_US), float64(MAX_PULSE_WIDTH_US))

				// Mix servos based on aircraft type configuration
				var servo1, servo2, servo4, servo5, servo6 float64
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
					servo5 = rollOutput
				case 3: // Single aileron V tail
					servo1 = rollOutput
					servo2 = pitchOutput - yawOutput
					servo4 = pitchOutput + yawOutput
					servo5 = 0
				case 4: // Dual aileron V tail
					servo1 = rollOutput
					servo2 = pitchOutput - yawOutput
					servo4 = pitchOutput + yawOutput
					servo5 = rollOutput
				default: // Elevon delta
					servo1 = rollOutput + pitchOutput
					servo2 = -rollOutput + pitchOutput
					servo4 = yawOutput
					servo5 = 0
				}

				// Reverse servo if required
				if servo1reverse {
					servo1 = servo1max + servo1min - servo1
				}
				if servo2reverse {
					servo2 = servo2max + servo2min - servo2
				}
				if servo4reverse {
					servo4 = servo4max + servo4min - servo4
				}
				if servo5reverse {
					servo5 = servo5max + servo5min - servo5
				}
				if servo6reverse {
					servo6 = servo6max + servo6min - servo6
				}

				// Handle servo midpoint trims
				servo1 = servo1 + servo1trim - 1500
				servo2 = servo2 + servo2trim - 1500
				servo4 = servo4 + servo4trim - 1500
				servo5 = servo5 + servo5trim - 1500
				servo6 = servo6 + servo6trim - 1500

				// Constrain servo pulse widths to a valid range.
				servo1pulse := uint32(constrain(servo1, servo1min, servo1max))
				servo2pulse := uint32(constrain(servo2, servo2min, servo2max))
				servo4pulse := uint32(constrain(servo4, servo4min, servo4max))
				servo5pulse := uint32(constrain(servo5, servo5min, servo5max))
				servo6pulse := uint32(constrain(servo6, servo6min, servo6max))

				// Set the PWM signals for the servos
				setServo(servo1pulse, servo2pulse, servo4pulse, servo5pulse, servo6pulse)

				// Arming engages throttle control Disarming disengages throttle control
				// Stabilization takes place regardless
				// In armed mode, set the ESC from ThrottleChannel
				if armed {
					// Handle ESC signal from ThrottleChannel
					escPulse = uint32(mapRange(float64(Channels[ThrottleChannel]), MIN_RX_VALUE, MAX_RX_VALUE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
				} else {
					// This is disarmed mode, set ESC to minimum
					escPulse = MIN_PULSE_WIDTH_US
				}
				setESC(escPulse)

				switch TuneParameterA {
				case 1: // pitch P
					pP = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, float64(TuneParameterAmin), float64(TuneParameterAmax))
				default:
					return
				}
				switch TuneParameterB {
				case 2: // roll P
					rP = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, float64(TuneParameterBmin), float64(TuneParameterBmax))
				default:
					return
				}
				switch TuneParameterC {
				case 3: // yaw P
					yP = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, float64(TuneParameterCmin), float64(TuneParameterCmax))
				default:
					return
				}
				switch TuneParameterD {
				case 1: // pitch P
					pP = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, float64(TuneParameterDmin), float64(TuneParameterDmax))
				default:
					return
				}
				// Print status and sensor data for debugging
				// Adding these statements can lead to the control loop crashing to failsafe if higher packet rates are used.
				//println("    Pin       ,    Pout      ,    Rin       ,     Rout     , armed")
				//println(desiredPitchRate, pitchOutput, desiredRollRate, rollOutput, armed)
				//println("RX: ele, ail, thr, mode, arm")
				//println("    ", Channels[ElevatorChannel], Channels[AileronChannel], Channels[ThrottleChannel], Channels[ManualModeChannel], Channels[ArmChannel])

				//println("Left, Right, ESC")
				//println(leftPulse, rightPulse, escPulse)
				//println()

			case FAILSAFE:
				setServo(servo1trim, servo2trim, servo4trim, servo5trim, servo6trim)
				setESC(MIN_PULSE_WIDTH_US)
				print(time.Now().UnixMilli())
				println(" ---------------- Receiver failsafe")
				setLED(1) // R during failsafe

				if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
					lastFlightState = flightState
					flightState = FLIGHT_MODE
				}
			}

			// Keep the watchdog happy
			watchdog.Update()
		}
	}
}
