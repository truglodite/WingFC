package main

import (
	"time"
)

const (
	ESC_FRAME_GAP = 35 * time.Microsecond // ≥30 µs gap between packets
)

// buildPacket creates a 16-bit DShot packet (11-bit throttle + 1-bit telemetry + 4-bit checksum)
func buildPacket(throttle uint16) uint16 {
	if throttle > 2047 {
		throttle = 2047
	}

	payload := uint32(throttle << 1) // telemetry bit = 0
	csum := uint32(0)
	csumData := payload
	for i := 0; i < 3; i++ {
		csum ^= (csumData & 0xF)
		csumData >>= 4
	}
	csum &= 0xF
	packet := uint16((payload << 4) | csum)
	return packet
}

// encodeDShot encodes 16-bit DShot packet into 64 SPI bits (8 bytes)
func encodeDShot(packet uint16, buf []byte) {
	// Clear buffer
	for i := range buf {
		buf[i] = 0
	}

	idx := 0
	for bit := 15; bit >= 0; bit-- {
		var pattern byte
		if (packet>>bit)&1 == 1 {
			pattern = 0b1110 // DShot "1"
		} else {
			pattern = 0b1000 // DShot "0"
		}

		// Pack 4 bits into buffer
		for i := 3; i >= 0; i-- {
			if (pattern>>i)&1 == 1 {
				buf[idx/8] |= 1 << (7 - (idx % 8))
			}
			idx++
		}
	}
}

// SendDShot sends a throttle command to ESC via SPI
func SendDShot(throttle uint16) {
	if !USE_DSHOT {
		return
	}

	packet := buildPacket(throttle)

	// 16 DShot bits * 4 SPI bits per bit = 64 bits = 8 bytes
	var buf [8]byte
	encodeDShot(packet, buf[:])

	// Transmit buffer
	spi.Tx(buf[:], nil)

	// Inter-frame gap
	time.Sleep(ESC_FRAME_GAP)
}
