package main

import "tinygo.org/x/drivers/ws2812"

type LEDState int

const (
	LEDOFF LEDState = iota
	PWMCONFIG
	PWMERROR
	SERVOINIT
	SERVOERROR
	ESCINIT
	ESCERROR
	IMUCONFIG
	IMUINIT
	IMUERROR
	CALIBRATE
	DISARMED
	ARMED
	FAILSAFED
)

// LEDController handles the onboard RGB LED
type LEDController struct {
	neo   ws2812.Device
	state LEDState
}

// SetState safely updates LED state
func (l *LEDController) SetState(s LEDState) {
	l.state = s
}

func (l *LEDController) updateLED() {
	switch l.state {
	case LEDOFF:
		l.setColor(0, 0, 0)
	case PWMCONFIG: // R
		l.setColor(255, 0, 0)
	case PWMERROR: // RG
		l.setColor(255, 255, 0)
	case SERVOINIT: // G
		l.setColor(0, 255, 0)
	case SERVOERROR: // GB
		l.setColor(0, 255, 255)
	case ESCINIT: // R
		l.setColor(255, 0, 0)
	case ESCERROR: // RGB
		l.setColor(255, 255, 255)
	case IMUCONFIG: // B
		l.setColor(0, 0, 255)
	case IMUINIT: // R
		l.setColor(255, 0, 0)
	case IMUERROR: // RB
		l.setColor(255, 0, 255)
	case CALIBRATE: // RGB
		l.setColor(255, 255, 255)
	case DISARMED: // G
		l.setColor(0, 255, 0)
	case ARMED: // B
		l.setColor(0, 0, 255)
	case FAILSAFED: // R
		l.setColor(255, 0, 0)
	}
}

// Set RGB color (WS2812 uses GRB order)
func (l *LEDController) setColor(r, g, b uint8) {
	l.neo.WriteByte(g)
	l.neo.WriteByte(r)
	l.neo.WriteByte(b)
}
