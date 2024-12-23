#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import signal
import sys

# Pin definitions (BCM numbering)
TEST_STROBE_PIN = 26  # Connected to GPIO19
TEST_DATA_PIN = 20    # Connected to GPIO16

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TEST_STROBE_PIN, GPIO.IN)
    GPIO.setup(TEST_DATA_PIN, GPIO.IN)
    print("GPIO pins initialized for testing")

def cleanup(signum, frame):
    GPIO.cleanup()
    sys.exit(0)

def monitor_pins():
    print("Monitoring GPIO pins...")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            strobe_state = GPIO.input(TEST_STROBE_PIN)
            data_state = GPIO.input(TEST_DATA_PIN)
            
            print(f"\rStrobe Pin (GPIO26): {'HIGH' if strobe_state else 'LOW'}, "
                  f"Data Pin (GPIO20): {'HIGH' if data_state else 'LOW'}", end='', flush=True)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    setup()
    monitor_pins()
    GPIO.cleanup()