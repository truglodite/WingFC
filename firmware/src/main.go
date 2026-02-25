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

	// PWM controllers and channels
	pwm0          = machine.PWM0
	pwm1          = machine.PWM1
	pwmCh1        uint8
	pwmCh2        uint8
	pwmCh4        uint8
	escCh         uint8
	escPin        machine.Pin
	servoPeriodNs uint64
	escPeriodNs   uint64

	// Control system components
	pitchPID   *PIDController
	rollPID    *PIDController
	yawPID     *PIDController
	dt         = 0.01
	kf         *KalmanFilter
	imuData    IMU
	imuYawMeas float64

	// IMU calibration
	accelXSum, accelYSum, accelZSum, accelBiasX, accelBiasY, accelBiasZ float64 = 0., 0., 0., 0., 0., 0.
	gyroXSum, gyroYSum, gyroZSum, gyroBiasX, gyroBiasY, gyroBiasZ       float64 = 0., 0., 0., 0., 0., 0.
	xA, yA, zA, xG, yG, zG                                              int32
	desiredPitchRate, desiredRollRate                                   float64
	pitchOutput, rollOutput                                             float64
	desiredYawRate, yawOutput                                           float64
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
	MIN_PULSE_WIDTH_US = 1000
	MAX_PULSE_WIDTH_US = 2000

	MIN_RX_VALUE     = 988
	MAX_RX_VALUE     = 2012
	NEUTRAL_RX_VALUE = 1500

	// Calculated constants for PID control
	MAX_ROLL_RATE  = MAX_ROLL_RATE_DEG * math.Pi / 180
	MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * math.Pi / 180
	MAX_YAW_RATE   = MAX_YAW_RATE_DEG * math.Pi / 180

	// --- Hardware Mappings ---
	PWM_CH1_PIN = machine.D0 // Aileron Servo
	PWM_CH2_PIN = machine.D1 // Elevator Servo
	PWM_CH3_PIN = machine.D2 // ESC (Electronic Speed Controller)
	PWM_CH4_PIN = machine.D3 // Yaw Servo (Rudder)

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
	time.Sleep(2 * time.Second) // Wait for hardware to stabilize
	println("WingFC Flight Controller - Version", Version)
	println("A TinyGo Flight Controller for Flying Wing Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	println("Initializing...")

	// --- Hardware Setup ---
	uart.Configure(machine.UARTConfig{
		BaudRate: BAUD_RATE,
		TX:       machine.UART_TX_PIN,
		RX:       machine.UART_RX_PIN,
	})
	println("UART configured for receiver.")

	servoPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY,
	}
	if err := pwm0.Configure(servoPWMConfig); err != nil {
		println("could not configure PWM for servos:", err)
		return
	}
	servoPeriodNs = servoPWMConfig.Period
	pwmCh1, err = pwm0.Channel(PWM_CH1_PIN)
	if err != nil {
		println("could not get PWM channel 1:", err)
		return
	}
	pwmCh2, err = pwm0.Channel(PWM_CH2_PIN)
	if err != nil {
		println("could not get PWM channel 2:", err)
		return
	}
	pwmCh4, err = pwm0.Channel(PWM_CH4_PIN)
	if err != nil {
		println("could not get PWM channel 4:", err)
		return
	}
	setServo(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
	println("PWM configured for servos.")

	if USE_DSHOT {
		escPin = PWM_CH3_PIN
		escPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
		setESC(MIN_PULSE_WIDTH_US)
		println("DShot configured for ESC.")
	} else {
		escPWMConfig := machine.PWMConfig{
			Period: machine.GHz * 1 / ESC_PWM_FREQUENCY,
		}
		if err = pwm1.Configure(escPWMConfig); err != nil {
			println("could not configure PWM for ESC:", err)
			return
		}
		escPeriodNs = escPWMConfig.Period
		escCh, err = pwm1.Channel(PWM_CH3_PIN)
		if err != nil {
			println("could not get PWM channel for ESC:", err)
			return
		}
		setESC(MIN_PULSE_WIDTH_US)
		println("PWM configured for ESC.")
	}

	i2c.Configure(machine.I2CConfig{
		Frequency: 400 * machine.KHz,
	})
	println("I2C configured for IMU.")

	// --- IMU Setup ---
	lsm = lsm6ds3tr.New(i2c)
	err = lsm.Configure(lsm6ds3tr.Configuration{
		AccelRange:      lsm6ds3tr.ACCEL_8G,
		AccelSampleRate: lsm6ds3tr.ACCEL_SR_416,
		GyroRange:       lsm6ds3tr.GYRO_1000DPS,
		GyroSampleRate:  lsm6ds3tr.GYRO_SR_416,
	})
	if err != nil {
		for {
			println("Failed to configure LSM6DS3TR:", err.Error())
			time.Sleep(time.Second)
		}
	}
	if !lsm.Connected() {
		println("LSM6DS3TR not connected")
		time.Sleep(time.Second)
		return
	}
	println("LSM6DS3TR configured and initialized.")

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
	setServo(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
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
					armed = false
				} else {
					//println("Armed!")
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
				// Get desired roll and pitch rates from the RC receiver.
				desiredPitchRate = mapRange(float64(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
				desiredRollRate = mapRange(float64(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)

				// Apply deadband to avoid small unwanted movements
				if math.Abs(desiredPitchRate) < DEADBAND*math.Pi/180 {
					desiredPitchRate = 0
				}
				if math.Abs(desiredRollRate) < DEADBAND*math.Pi/180 {
					desiredRollRate = 0
				}

				// Desired yaw rate from rudder channel
				desiredYawRate = mapRange(float64(Channels[RudderChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_YAW_RATE, MAX_YAW_RATE)
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
					rollPID.integral = 0  // reset integral term in manual mode to prevent windup
					pitchOutput = desiredPitchRate
					rollOutput = desiredRollRate
					yawPID.integral = 0
					yawOutput = desiredYawRate
				}

				// Combine PID outputs with a mix of raw RC input.
				leftElevon := pitchOutput + rollOutput
				rightElevon := pitchOutput - rollOutput

				// Convert control outputs to PWM pulse widths.
				leftElevon = mapRange(float64(leftElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
				rightElevon = mapRange(float64(rightElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

				// Rudder (yaw) pulse
				rudder := mapRange(float64(yawOutput), -MAX_YAW_RATE, MAX_YAW_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

				// Constrain pulse widths to a valid range.
				leftPulse := uint32(constrain(leftElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
				rightPulse := uint32(constrain(rightElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
				rudderPulse := uint32(constrain(rudder, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))

				// Set the PWM signals for the servos (left, right, rudder).
				setServo(leftPulse, rightPulse, rudderPulse)

				// Arming engages throttle control Disarming disengages throttle control
				// Stabilization takes place regardless
				// In armed mode, set the ESC from ThrottleChannel

				//trug
				if armed {
					// Handle ESC signal from ThrottleChannel

					escPulse = uint32(mapRange(float64(Channels[ThrottleChannel]), MIN_RX_VALUE, MAX_RX_VALUE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
				} else {
					// This is disarmed mode, set ESC to minimum
					escPulse = MIN_PULSE_WIDTH_US
				}
				setESC(escPulse)
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
				setServo(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)
				println("Receiver failsafe")

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
