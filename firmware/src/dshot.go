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
	periodNs := int64(1000000000 / (DSHOT_RATE * 1000))
	// High times for '1' and '0' (use ~67% and ~33%)
	high1 := time.Duration(periodNs*75/100) * time.Nanosecond
	high0 := time.Duration(periodNs*38/100) * time.Nanosecond
	period := time.Duration(periodNs) * time.Nanosecond

	// Ensure ESC pin is configured as output
	escPin.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Send 16-bit DSHOT packet (MSB first)
	for bit := 15; bit >= 0; bit-- {
		targetHigh := high0
		if ((packet >> uint(bit)) & 1) == 1 {
			targetHigh = high1
		}

		// Start bit period
		start := time.Now()
		escPin.High()

		// Busy-wait for High duration
		for time.Since(start) < targetHigh {
		}
		escPin.Low()

		// Busy-wait for remainder of the Period
		for time.Since(start) < period {
		}
	}

	// IMPORTANT: Inter-frame gap (Minimum 30us)
	// This allows the ESC to process the frame before the next one starts.
	time.Sleep(35 * time.Microsecond)
}

// For tests or direct pin toggling (not used externally)
func rawPulse(pin machine.Pin, high time.Duration, low time.Duration) {
	pin.High()
	time.Sleep(high)
	pin.Low()
	time.Sleep(low)
}
