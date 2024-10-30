#!/usr/bin/env python3
"""
A simple script to check for read/write access on GPIO pins
"""
from periphery import GPIO

# Open pins
gpio_in = GPIO("/dev/gpiochip1", 88, "in")
gpio_out = GPIO("/dev/gpiochip1", 87, "out")

# Read/write pins
value = gpio_in.read()
gpio_out.write(not value)

# Close pins
gpio_in.close()
gpio_out.close()
