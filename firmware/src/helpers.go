package main

import "golang.org/x/exp/constraints"

// Read raw IMU data from the LSM6DS3TR sensor and apply a low-pass filter.
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
	if imuOrientation == 0 {	// default
		rawAccelXo = rawAccelX
		rawAccelYo = rawAccelY
		rawAccelZo = rawAccelZ
		rawGyroXo = rawGyroX
		rawGyroYo = rawGyroY
		rawGyroZo = rawGyroZ
	}
	else if imuOrientation == 1 {	// CW90
		rawAccelXo = rawAccelY
		rawAccelYo = -rawAccelX
		rawAccelZo = rawAccelZ
		rawGyroXo = rawGyroY
		rawGyroYo = -rawGyroX
		rawGyroZo = rawGyroZ
	}
	else if imuOrientation == 2 {	// CW180
		rawAccelXo = -rawAccelX
		rawAccelYo = -rawAccelY
		rawAccelZo = rawAccelZ
		rawGyroXo = -rawGyroX
		rawGyroYo = -rawGyroY
		rawGyroZo = rawGyroZ
	} 
	else if imuOrientation == 3 {	// CW270
		rawAccelXo = -rawAccelY
		rawAccelYo = rawAccelX
		rawAccelZo = rawAccelZ
		rawGyroXo = -rawGyroY
		rawGyroYo = rawGyroX
		rawGyroZo = rawGyroZ
	} 
	// need to flesh out the rest of these orientations
	else if imuOrientation == 4 {	// flip
				rawAccelXo = rawAccelX
		rawAccelYo = rawAccelY
		rawAccelZo = rawAccelZ
		rawGyroXo = rawGyroX
		rawGyroYo = rawGyroY
		rawGyroZo = rawGyroZ
	} 
	else if imuOrientation == 5 {	// flipCW90
				rawAccelXo = rawAccelX
		rawAccelYo = rawAccelY
		rawAccelZo = rawAccelZ
		rawGyroXo = rawGyroX
		rawGyroYo = rawGyroY
		rawGyroZo = rawGyroZ
	} 
	else if imuOrientation == 6 {	// flipCW180
				rawAccelXo = rawAccelX
		rawAccelYo = rawAccelY
		rawAccelZo = rawAccelZ
		rawGyroXo = rawGyroX
		rawGyroYo = rawGyroY
		rawGyroZo = rawGyroZ
	} 
	else if imuOrientation == 7 {	// flipCW270
				rawAccelXo = rawAccelX
		rawAccelYo = rawAccelY
		rawAccelZo = rawAccelZ
		rawGyroXo = rawGyroX
		rawGyroYo = rawGyroY
		rawGyroZo = rawGyroZ
	} else {
		// Go requires these to be initialized.
		// If imuOrientation isn't 0, we'll just pass raw data through for now.
		rawAccelXo, rawAccelYo, rawAccelZo = rawAccelX, rawAccelY, rawAccelZ
		rawGyroXo, rawGyroYo, rawGyroZo = rawGyroX, rawGyroY, rawGyroZ
	}

	// 3. Apply Filter using the 'o' variables
	// This fixes the "declared and not used" error and applies your orientation logic.
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
func setServo(servo1pulse, servo2Pulse, servo4Pulse, servo5pulse uint32) {
	// The Period() function is not available. We use the saved period instead.
	top_value := pwm0.Top()

	// Calculate the duty cycle for the left servo.
	servo1 := uint32(uint64(servo1pulse) * 1000 * uint64(top_value) / uint64(servoPeriodNs))
	pwm0.Set(pwmCh1, servo1)

	// Calculate the duty cycle for the right servo.
	servo2 := uint32(uint64(servo2Pulse) * 1000 * uint64(top_value) / uint64(servoPeriodNs))
	pwm0.Set(pwmCh2, servo2)

	// Calculate the duty cycle for the rudder servo.
	servo4 := uint32(uint64(servo4Pulse) * 1000 * uint64(top_value) / uint64(servoPeriodNs))
	pwm0.Set(pwmCh4, servo4)

	// Calculate the duty cycle for the fifth servo.
	servo5 := uint32(uint64(servo5pulse) * 1000 * uint64(top_value) / uint64(servoPeriodNs))
	pwm0.Set(pwmCh5, servo5)
}

// setESC sets the PWM duty cycle for the ESC.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setESC(servo3pulse uint32) {
	if USE_DSHOT {
		// Map pulse width (microseconds) to DShot throttle range (0..2047)
		var throttle uint16
		if servo3pulse <= MIN_PULSE_WIDTH_US {
			throttle = 0
		} else {
			val := uint16(mapRange(float64(servo3pulse), MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, 0, 2047))
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
	duty := uint32(uint64(servo3pulse) * 1000 * uint64(top_value) / uint64(escPeriodNs))
	pwm1.Set(escCh, duty)
}
