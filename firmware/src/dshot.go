package main

import (
	"machine"
	"runtime"
	"runtime/interrupt"
	"time"
	_ "unsafe" // REQUIRED for go:linkname to work
)

// Import the internal nanotime function
//
//go:linkname nanotime runtime.nanotime
func nanotime() int64

func SendDShot(throttle uint16) {
	if throttle > 2047 {
		throttle = 2047
	}

	// 1. Build Packet (11-bit throttle, 1-bit telemetry, 4-bit checksum)
	payload := uint32(throttle << 1)
	csum := uint32(0)
	csum_data := payload
	for i := 0; i < 3; i++ {
		csum ^= (csum_data & 0xF)
		csum_data >>= 4
	}
	packet := (payload << 4) | (csum & 0xF)

	// 2. Timing (DSHOT150 = 6666ns period)
	const (
		period = 6666
		high1  = 5000 // 75%
		high0  = 2500 // 37.5%
	)

	// 3. Safety: Lock OS thread and disable interrupts
	runtime.LockOSThread()
	mask := interrupt.Disable()

	for bit := 15; bit >= 0; bit-- {
		targetHigh := int64(high0)
		if ((packet >> uint(bit)) & 1) == 1 {
			targetHigh = high1
		}

		start := nanotime()
		escPin.High()
		// Busy-wait for high pulse
		for (nanotime() - start) < targetHigh {
		}

		escPin.Low()
		// Busy-wait for full bit period
		for (nanotime() - start) < period {
		}
	}

	interrupt.Restore(mask)
	runtime.UnlockOSThread()

	// Inter-frame gap (Minimum 30us)
	time.Sleep(35 * time.Microsecond)
}

// For tests or direct pin toggling (not used externally)
func rawPulse(pin machine.Pin, high time.Duration, low time.Duration) {
	pin.High()
	time.Sleep(high)
	pin.Low()
	time.Sleep(low)
}
