#pragma once

// serial_hal — 50 Hz USB-CDC JSON telemetry stream for the Pi ROS2 bridge.
// Day 2 will add an RX path for setpoint commands; leave Serial RX open.
void serial_hal_init();
void serial_hal_update();
