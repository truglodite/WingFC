package main

import "golang.org/x/exp/constraints"

// Read raw IMU data from the LSM6DS3TR sensor, remap for board orientation, and apply a low-pass filter.
func readLSMData() {
	var rawAccelXo, rawAccelYo, rawAccelZo, rawGyroXo, rawGyroYo, rawGyroZo int32

	rawAccelX, rawAccelY, rawAccelZ, err := lsm.ReadAcceleration()
	if err != nil {
		println("Error reading acceleration:", err)
	}
	rawGyroX, rawGyroY, rawGyroZ, err := lsm.ReadRotation()
	if err != nil {
		println("Error reading rotation:", err)
	}

	// Map imu data based on board orientation configuration
	switch orientation {
	case 1: // CW90
		rawAccelXo, rawAccelYo, rawAccelZo = rawAccelY, -rawAccelX, rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = rawGyroY, -rawGyroX, rawGyroZ
	case 2: // CW180
		rawAccelXo, rawAccelYo, rawAccelZo = -rawAccelX, -rawAccelY, rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = -rawGyroX, -rawGyroY, rawGyroZ
	case 3: // CW270
		rawAccelXo, rawAccelYo, rawAccelZo = -rawAccelY, rawAccelX, rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = -rawGyroY, rawGyroX, rawGyroZ
	case 4: // flip
		rawAccelXo, rawAccelYo, rawAccelZo = rawAccelX, -rawAccelY, -rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = rawGyroX, -rawGyroY, -rawGyroZ
	case 5: // flipCW90
		rawAccelXo, rawAccelYo, rawAccelZo = -rawAccelY, -rawAccelX, -rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = -rawGyroY, -rawGyroX, -rawGyroZ
	case 6: // flipCW180
		rawAccelXo, rawAccelYo, rawAccelZo = -rawAccelX, rawAccelY, -rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = -rawGyroX, rawGyroY, -rawGyroZ
	case 7: // flipCW270
		rawAccelXo, rawAccelYo, rawAccelZo = rawAccelY, rawAccelX, -rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = rawGyroY, rawGyroX, -rawGyroZ
	default: // default
		rawAccelXo, rawAccelYo, rawAccelZo = rawAccelX, rawAccelY, rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = rawGyroX, rawGyroY, rawGyroZ
	}

	// Apply filtering to remapped IMU data
	imuData.AccelX += LPF_ALPHA * (float64(rawAccelXo)*microGToMS2 - imuData.AccelX)
	imuData.AccelY += LPF_ALPHA * (float64(rawAccelYo)*microGToMS2 - imuData.AccelY)
	imuData.AccelZ += LPF_ALPHA * (float64(rawAccelZo)*microGToMS2 - imuData.AccelZ)

	imuData.GyroX += LPF_ALPHA * (float64(rawGyroXo)*microDPSToRadS - imuData.GyroX)
	imuData.GyroY += LPF_ALPHA * (float64(rawGyroYo)*microDPSToRadS - imuData.GyroY)
	imuData.GyroZ += LPF_ALPHA * (float64(rawGyroZo)*microDPSToRadS - imuData.GyroZ)
}

// Process the raw IMU data by applying calibration offsets and computing roll/pitch angles.
func processLSMData() {
	imuData.AccelX -= accelBiasX
	imuData.AccelY -= accelBiasY
	imuData.AccelZ -= accelBiasZ
	imuData.GyroX -= gyroBiasX
	imuData.GyroY -= gyroBiasY
	imuData.GyroZ -= gyroBiasZ
	imuData.Roll = imuData.rollAccel()
	imuData.Pitch = imuData.pitchAccel()
}

// Calibrate the IMU by averaging a number of samples to determine bias offsets.
// This function should be called when the aircraft is stationary and level.
func calibrate() {
	const sampleSize = 10000
	for i := 0; i < sampleSize; i++ {
		readLSMData()
		accelXSum += imuData.AccelX
		accelYSum += imuData.AccelY
		accelZSum += imuData.AccelZ
		gyroXSum += imuData.GyroX
		gyroYSum += imuData.GyroY
		gyroZSum += imuData.GyroZ
		if i%1000 == 0 {
			println(i / 1000)
		}
	}
	// println(accelXSum, accelYSum, accelZSum, gyroXSum, gyroYSum, gyroZSum)

	accelBiasX = (accelXSum / sampleSize) * microGToMS2
	accelBiasY = (accelYSum / sampleSize) * microGToMS2
	accelBiasZ = (accelZSum / sampleSize) * microGToMS2
	println("Accel calibration complete. Bias X:", accelBiasX, "Bias Y:", accelBiasY, "Bias Z:", accelBiasZ)
	gyroBiasX = (gyroXSum / sampleSize) * microDPSToRadS
	gyroBiasY = (gyroYSum / sampleSize) * microDPSToRadS
	gyroBiasZ = (gyroZSum / sampleSize) * microDPSToRadS
	println("Gyro calibration complete. Bias X:", gyroBiasX, "Bias Y:", gyroBiasY, "Bias Z:", gyroBiasZ)
}

// Helper function to constrain a value within min and max bounds.
func constrain(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// Helper function to map a value from one range to another.
func mapRange[T constraints.Float](value, fromMin, fromMax, toMin, toMax T) T {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}

// setServo sets the PWM duty cycle for the aileron and elevator servos.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setServo(servo1pulse, servo2Pulse, servo4Pulse, servo5pulse, servo6pulse uint32) {
	// The Period() function is not available. We use the saved period instead.
	top_value0 := pwm0.Top()
	top_value2 := pwm2.Top()

	// Calculate the duty cycle for the left servo.
	servo1 := uint32(uint64(servo1pulse) * 1000 * uint64(top_value0) / uint64(pwm0periodNs))
	pwm0.Set(pwmCh1, servo1)

	// Calculate the duty cycle for the right servo.
	servo2 := uint32(uint64(servo2Pulse) * 1000 * uint64(top_value0) / uint64(pwm0periodNs))
	pwm0.Set(pwmCh2, servo2)

	// Calculate the duty cycle for the rudder servo.
	servo4 := uint32(uint64(servo4Pulse) * 1000 * uint64(top_value0) / uint64(pwm0periodNs))
	pwm0.Set(pwmCh4, servo4)

	// Calculate the duty cycle for the fifth servo.
	servo5 := uint32(uint64(servo5pulse) * 1000 * uint64(top_value0) / uint64(pwm0periodNs))
	pwm0.Set(pwmCh5, servo5)

	// Calculate the duty cycle for the 6th servo.
	servo6 := uint32(uint64(servo6pulse) * 1000 * uint64(top_value2) / uint64(pwm2periodNs))
	pwm2.Set(pwmCh6, servo6)
}

// setESC sets the PWM duty cycle for the ESC.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setESC(throttlePulse uint32) {
	if USE_DSHOT {
		// Map pulse width (microseconds) to DShot throttle range (0..2047)
		// 0 for disarm, 48-2047 throttle steps
		var throttle uint16
		if throttlePulse <= MIN_PULSE_WIDTH_US {
			throttle = 0
		} else {
			val := uint16(mapRange(float64(throttlePulse), MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, 48, 2047))
			if val > 2047 {
				val = 2047
			}
			throttle = val
		}
		SendDShot(throttle)
		return
	}

	// The Period() function is not available. We use the saved period instead.
	top_value := pwm1.Top()

	// Calculate the duty cycle for the ESC.
	duty := uint32(uint64(throttlePulse) * 1000 * uint64(top_value) / uint64(escPeriodNs))
	pwm1.Set(pwmCh3, duty)
}

// setLED sets the output state of the built in RGB LED.
// High() each color off.
func setLED(color uint8) {
	switch color {
	case 0: // off
		redLED.High()
		greenLED.High()
		blueLED.High()
	case 1: // R
		redLED.Low()
		greenLED.High()
		blueLED.High()
	case 2: // G
		redLED.High()
		greenLED.Low()
		blueLED.High()
	case 3: // B
		redLED.High()
		greenLED.High()
		blueLED.Low()
	case 4: // RG
		redLED.Low()
		greenLED.Low()
		blueLED.High()
	case 5: // RB
		redLED.Low()
		greenLED.High()
		blueLED.Low()
	case 6: // GB
		redLED.High()
		greenLED.Low()
		blueLED.Low()
	case 7: // RGB
		redLED.Low()
		greenLED.Low()
		blueLED.Low()
	}
}

// Function to handle in flight tuning
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
func updateTuning() {
	switch TuneParameterA {
	case 1: // pitch P
		pP = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 2: // roll P
		rP = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 3: // yaw P
		yP = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 4: // pitch I
		pI = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 5: // roll I
		rI = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 6: // yaw I
		yI = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 7: // pitch D
		pD = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 8: // roll I
		rD = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 9: // yaw I
		yD = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 10: // pitch rate
		MAX_PITCH_RATE_DEG = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 11: // roll rate
		MAX_ROLL_RATE_DEG = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 12: // yaw rate
		MAX_YAW_RATE_DEG = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	case 13: // pid weight
		PID_WEIGHT = mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterAmin, TuneParameterAmax)
	default:
	}
	switch TuneParameterB {
	case 1: // pitch P
		pP = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 2: // roll P
		rP = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 3: // yaw P
		yP = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 4: // pitch I
		pI = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 5: // roll I
		rI = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 6: // yaw I
		yI = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 7: // pitch D
		pD = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 8: // roll I
		rD = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 9: // yaw I
		yD = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 10: // pitch rate
		MAX_PITCH_RATE_DEG = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 11: // roll rate
		MAX_ROLL_RATE_DEG = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 12: // yaw rate
		MAX_YAW_RATE_DEG = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	case 13: // pid weight
		PID_WEIGHT = mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterBmin, TuneParameterBmax)
	default:
	}
	switch TuneParameterC {
	case 1: // pitch P
		pP = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 2: // roll P
		rP = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 3: // yaw P
		yP = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 4: // pitch I
		pI = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 5: // roll I
		rI = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 6: // yaw I
		yI = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 7: // pitch D
		pD = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 8: // roll I
		rD = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 9: // yaw I
		yD = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 10: // pitch rate
		MAX_PITCH_RATE_DEG = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 11: // roll rate
		MAX_ROLL_RATE_DEG = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 12: // yaw rate
		MAX_YAW_RATE_DEG = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	case 13: // pid weight
		PID_WEIGHT = mapRange(float64(Channels[TuningChannelC]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterCmin, TuneParameterCmax)
	default:
	}
	switch TuneParameterD {
	case 1: // pitch P
		pP = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 2: // roll P
		rP = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 3: // yaw P
		yP = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 4: // pitch I
		pI = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 5: // roll I
		rI = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 6: // yaw I
		yI = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 7: // pitch D
		pD = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 8: // roll I
		rD = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 9: // yaw I
		yD = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 10: // pitch rate
		MAX_PITCH_RATE_DEG = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 11: // roll rate
		MAX_ROLL_RATE_DEG = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 12: // yaw rate
		MAX_YAW_RATE_DEG = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	case 13: // yaw P
		PID_WEIGHT = mapRange(float64(Channels[TuningChannelD]), MIN_RX_VALUE, MAX_RX_VALUE, TuneParameterDmin, TuneParameterDmax)
	default:
	}
}
