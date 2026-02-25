package main

// KalmanFilter represents a multivariate Kalman Filter.
// State vector X: [pitch, roll, yaw]
// Measurement vector Z: [pitch_accel, roll_accel]
type KalmanFilter struct {
	// State Vector
	X *Matrix // (3x1) estimated state vector [pitch, roll, yaw]

	// Covariance Matrices
	P *Matrix // (3x3) Estimate error covariance
	Q *Matrix // (3x3) Process noise covariance
	R *Matrix // (3x3) Measurement noise covariance

	// System Matrices
	F *Matrix // (3x3) State transition matrix
	H *Matrix // (3x3) Observation matrix
	B *Matrix // (3x3) Control input matrix (unused)

	dt float64 // Time step
}

// NewKalmanFilter creates and initializes a new KalmanFilter.
func NewKalmanFilter(dt float64) *KalmanFilter {
	// State vector: [pitch, roll, yaw] (3x1)
	x := NewMatrix(3, 1)

	// Process Noise Covariance (Q): small values for trusted gyro rates.
	q := Identity(3)
	q.Set(0, 0, 0.01) // Pitch noise
	q.Set(1, 1, 0.01) // Roll noise
	q.Set(2, 2, 0.02) // Yaw noise (slightly larger)

	// Measurement Noise Covariance (R): Accel is noisy (pitch, roll); yaw from gyro integration
	r := Identity(3)
	r.Set(0, 0, 0.5) // Pitch noise
	r.Set(1, 1, 0.5) // Roll noise
	r.Set(2, 2, 0.2) // Yaw measurement noise

	// State Transition Matrix (F): start as identity
	f := Identity(3)

	// Observation Matrix (H): We observe pitch, roll from accel and yaw from integrated gyro
	h := Identity(3)

	return &KalmanFilter{
		X:  x,
		P:  Identity(3), // P starts as an identity matrix
		Q:  q,
		R:  r,
		F:  f,
		H:  h,
		dt: dt,
	}
}

// Predict updates the state and covariance using the control inputs from the gyroscope.
func (kf *KalmanFilter) Predict(gyroX, gyroY, gyroZ float64) {
	// State transition remains identity; we use gyro rates for integration

	// Use gyro rates to predict the next state (angle integration)
	gyroVector := NewMatrix(3, 1)
	gyroVector.Set(0, 0, gyroY*kf.dt) // pitch integrates from gyro Y
	gyroVector.Set(1, 0, gyroX*kf.dt) // roll integrates from gyro X
	gyroVector.Set(2, 0, gyroZ*kf.dt) // yaw integrates from gyro Z
	kf.X = kf.X.Add(gyroVector)

	// Predict the next covariance: P_pred = F * P_prev * F^T + Q
	fT := kf.F.Transpose()
	kf.P = kf.F.Multiply(kf.P).Multiply(fT).Add(kf.Q)
}

// Update corrects the state and covariance with a new measurement from the accelerometer.
func (kf *KalmanFilter) Update(accelPitch, accelRoll, yawMeas float64) {
	// Measurement vector Z (3x1)
	z := NewMatrix(3, 1)
	z.Set(0, 0, accelPitch)
	z.Set(1, 0, accelRoll)
	z.Set(2, 0, yawMeas)

	// Innovation y = z - H * x_pred
	y := z.Subtract(kf.H.Multiply(kf.X))

	// Innovation covariance S = H * P_pred * H^T + R
	hT := kf.H.Transpose()
	S := kf.H.Multiply(kf.P).Multiply(hT).Add(kf.R)
	Sinv := S.Inverse()

	// Kalman gain K = P_pred * H^T * S^-1
	K := kf.P.Multiply(hT).Multiply(Sinv)

	// Updated state estimate x = x_pred + K * y
	kf.X = kf.X.Add(K.Multiply(y))

	// Updated estimate covariance P = (I - K * H) * P_pred
	I := Identity(3)
	kf.P = I.Subtract(K.Multiply(kf.H)).Multiply(kf.P)
}
