# Copilot / AI Agent Instructions for WingFC

Purpose: give an AI coding agent the minimal, concrete knowledge to be productive in this TinyGo flight-controller repository.

Summary
- TinyGo-based embedded flight controller for a Seeed XIAO nRF52840 Sense (onboard LSM6DS3TR IMU). Main loop and control logic live in `firmware/src`.
- Receiver driver selection is done at build time via Go build tags (`ibus` or `crsf`). Only one receiver implementation is compiled.

Quick starts (build / flash)
- Build & flash for CRSF receiver:
```
tinygo flash -target=xiao-ble -tags=crsf firmware/src
```
- Build & flash for iBus receiver:
```
tinygo flash -target=xiao-ble -tags=ibus firmware/src
```

Architecture & key components
- `firmware/src/main.go`: program entry, hardware init (UART, I2C, PWM, watchdog), main control loop, state machine (CALIBRATION, FLIGHT_MODE, FAILSAFE).
- `firmware/src/config.go`: all hardware mappings and tunables (channels, PWM frequencies, PID gains, limits). Change here for parameter updates.
- `firmware/src/*receiver*.go` (`crsf.go`, `ibus.go`): receiver implementations guarded by build tags. They provide `readReceiver(packetChan)` and `processReceiverPacket()` and expose a `packetChan` for the main loop.
- `firmware/src/imu.go`, `kalman.go`, `matrix.go`: sensor fusion and attitude math. `KalmanFilter` operates on pitch/roll state.
- `firmware/src/pid.go`: simple PID controller used for pitch/roll rate control.
- `firmware/src/helpers.go`: IMU read, calibration, PWM-to-pulse helpers and mapping utilities.

Important patterns & gotchas
- Build tags select receiver code: don't try to compile both `crsf.go` and `ibus.go` at once — they declare the same symbols (e.g., `packetChan`) but are intended to be mutually exclusive via `-tags`.
- Global state is used per embedded constraints: `Channels` (global array), `imuData`, PID and Kalman filter instances. Changes to these files must respect concurrency between `readReceiver` goroutine and main loop.
- `readReceiver` runs as a goroutine and sends full packets on a channel (see `crsf.go` and `ibus.go`). The main loop reads from that channel and calls `processReceiverPacket()` which updates the global `Channels` array.
- Calibration (`calibrate()` in `helpers.go`) is blocking and samples many IMU readings; call only when device is stationary.
- Hardware specifics: PWM channels and pins are defined in `main.go` and `config.go`; the code assumes specific TinyGo `machine` peripherals (I2C0, PWM0/PWM1, DefaultUART). Changing board targets may require pin mapping updates.

Developer workflows
- Flashing: use `tinygo flash` as shown above. The README includes this command and target.
- Debugging on-host: `println()` is used throughout for simple telemetry. There's no serial console abstraction beyond `machine.UART`.
- Tests: there are tests under `tests/` (e.g., `tests/crsf_test.go`) — look there for protocol-specific expectations.

Conventions to follow
- Tunables belong in `config.go`. Keep hardware mapping and constants there.
- Use build tags for feature/driver selection. Follow the existing `//go:build ibus` / `//go:build crsf` pattern.
- Keep real-time code (control loop, ISR-like readers) concise and allocation-free where possible — this is TinyGo for microcontrollers.

Where to look first when asked to change behavior
- To change receiver parsing / add protocol: add a new `*_receiver.go` with appropriate `//go:build` tag and mirror `readReceiver(packetChan)` + `processReceiverPacket()` signatures.
- To adjust control behavior: edit PID values in `config.go` and `pid.go` usage in `main.go`.
- To update sensor fusion: edit `kalman.go` and `matrix.go` (both used by `main.go`'s `kf` instance).

References (quick links)
- [firmware/src/main.go](firmware/src/main.go)
- [firmware/src/config.go](firmware/src/config.go)
- [firmware/src/crsf.go](firmware/src/crsf.go)
- [firmware/src/ibus.go](firmware/src/ibus.go)
- [firmware/src/kalman.go](firmware/src/kalman.go)
- [firmware/src/pid.go](firmware/src/pid.go)
- [firmware/src/helpers.go](firmware/src/helpers.go)
- [README.md](README.md)

If anything above is unclear or you want more examples (e.g., where to add unit tests, or how to run TinyGo builds locally vs CI), tell me which area to expand. 
