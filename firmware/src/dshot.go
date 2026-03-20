package main

import (
	"machine"
	"runtime"
	"runtime/interrupt"
)

// CALIBRATION: Adjust these numbers until your scope shows 6.6µs total bit time.
const (
	// Previous (7.5µs total): 35 high + 10 low = 45 total (for '1')
	// New target (6.6µs): Aiming for ~38 total units
	countHigh1 = 30 // Reduced from 35
	countLow1  = 8  // Reduced from 10

	// Previous (7.5µs total): 15 high + 30 low = 45 total (for '0')
	countHigh0 = 13 // Reduced from 15
	countLow0  = 25 // Reduced from 30
)

// delay is a simple loop that the compiler cannot optimize away
func delay(n int) {
	for i := 0; i < n; i++ {
		runtime.KeepAlive(i) // Forces the CPU to actually perform the loop
	}
}

func SendDShot(throttle uint16) {
	if throttle > 2047 {
		throttle = 2047
	}

	// Build Packet (11-bit throttle, 1-bit telemetry, 4-bit checksum)
	payload := uint32(throttle << 1)
	csum := uint32(0)
	csum_data := payload
	for i := 0; i < 3; i++ {
		csum ^= (csum_data & 0xF)
		csum_data >>= 4
	}
	packet := (payload << 4) | (csum & 0xF)

	// Lock interrupts to prevent the 40µs "jitter"
	mask := interrupt.Disable()
	for bit := 15; bit >= 0; bit-- {
		machine.D2.High()
		if ((packet >> uint(bit)) & 1) == 1 {
			delay(countHigh1)
			machine.D2.Low()
			delay(countLow1)
		} else {
			delay(countHigh0)
			machine.D2.Low()
			delay(countLow0)
		}
	}
	interrupt.Restore(mask)
}
