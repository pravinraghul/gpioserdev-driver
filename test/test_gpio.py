#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import signal
import sys

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
    print("Monitoring GPIO pins... Press Ctrl+C to exit")
    
    prev_strobe_state = GPIO.input(TEST_STROBE_PIN)
    prev_data_state = GPIO.input(TEST_DATA_PIN)
    
    try:
        while True:
            strobe_state = GPIO.input(TEST_STROBE_PIN)
            data_state = GPIO.input(TEST_DATA_PIN)
            
            if strobe_state != prev_strobe_state:
                print(f"Strobe Pin (GPIO26): {'HIGH' if strobe_state else 'LOW'}")
                prev_strobe_state = strobe_state
                
            if data_state != prev_data_state:
                print(f"Data Pin (GPIO20): {'HIGH' if data_state else 'LOW'}")
                prev_data_state = data_state
                
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    setup()
    monitor_pins()
    GPIO.cleanup()