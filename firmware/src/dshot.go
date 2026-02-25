package main

import (
	"machine"
	"time"
)

// SendDShot sends a 16-bit DShot packet (11-bit throttle, 1-bit telemetry, 4-bit checksum)
// over the ESC pin using bit-banged timing. This is a minimal implementation
// intended for slower DShot rates (e.g., 150 or 300 kHz).
func SendDShot(throttle uint16) {
	if !USE_DSHOT {
		return
	}
	if throttle > 2047 {
		throttle = 2047
	}

	// Build packet: 11-bit throttle, 1-bit telemetry (0), 4-bit checksum
	payload := uint32(throttle << 1) // telemetry bit = 0
	// checksum: XOR of the three nibbles
	csum := uint32(0)
	csum_data := payload
	for i := 0; i < 3; i++ {
		csum ^= (csum_data & 0xF)
		csum_data >>= 4
	}
	csum &= 0xF
	packet := (payload << 4) | csum

	// Timing calculation (nanoseconds per bit)
	periodNs := int64(1000000000 / DSHOT_RATE)
	// High times for '1' and '0' (use ~67% and ~33%)
	high1 := time.Duration(periodNs*67/100) * time.Nanosecond
	high0 := time.Duration(periodNs*33/100) * time.Nanosecond
	period := time.Duration(periodNs) * time.Nanosecond

	// Ensure ESC pin is configured as output
	escPin.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Send MSB first
	for bit := 15; bit >= 0; bit-- {
		if ((packet >> uint(bit)) & 1) == 1 {
			escPin.High()
			time.Sleep(high1)
			escPin.Low()
			time.Sleep(period - high1)
		} else {
			escPin.High()
			time.Sleep(high0)
			escPin.Low()
			time.Sleep(period - high0)
		}
	}

	// Small gap after packet to allow ESC to latch
	time.Sleep(100 * time.Microsecond)
}

// For tests or direct pin toggling (not used externally)
func rawPulse(pin machine.Pin, high time.Duration, low time.Duration) {
	pin.High()
	time.Sleep(high)
	pin.Low()
	time.Sleep(low)
}
