# import RPi.GPIO as GPIO
# from time import sleep

# GPIO.setmode(GPIO.BCM)
# LED_PIN = 17

# GPIO.setup(LED_PIN, GPIO.OUT)

# try:
#     while True:
#         GPIO.output(LED_PIN, GPIO.HIGH)
#         print("LED on")
#         sleep(1)
#         GPIO.output(LED_PIN, GPIO.LOW)
#         print("LED off")
#         sleep(1)
# except KeyboardInterrupt:
#     print("\nExiting...")
#     GPIO.cleanup()
# from gpiozero import LED
# from time import sleep

# led = LED(17)  # BCM pin 17

# try:
#     while True:
#         led.on()
#         print("LED on")
#         sleep(1)
#         led.off()
#         print("LED off")
#         sleep(1)
# except KeyboardInterrupt:
#     print("\nExiting...")

# import lgpio
# import time

# h = lgpio.gpiochip_open(0)                                                                                                                                                                                                                                                                                                                                                                                            
# lgpio.gpio_claim_output(h, 17)

# try:
#     while True:
#         lgpio.gpio_write(h, 17, 1)
#         time.sleep(1)
#         lgpio.gpio_write(h, 17, 0)
#         time.sleep(1)
# except KeyboardInterrupt:
#     lgpio.gpiochip_close(h)

# import gpiod
# import time
# LED_PIN = 18

# chip = gpiod.Chip('gpiochip4')
# led_line = chip.get_line(LED_PIN) 

#!/usr/bin/env python3

# import pigpio
# import time

# # GPIO pin number
# LED_PIN = 17

# print("LED Blink Program Started (Press CTRL+C to exit)")

# try:
#     # Connect to the pigpio daemon
#     pi = pigpio.pi()
    
#     # Blink the LED
#     while True:
#         pi.write(LED_PIN, 1)  # Turn LED on
#         print("LED ON")
#         time.sleep(1)
        
#         pi.write(LED_PIN, 0)  # Turn LED off
#         print("LED OFF")
#         time.sleep(1)
        
# except KeyboardInterrupt:
#     print("Program stopped by user")
#     # Ensure LED is off when exiting
#     if 'pi' in locals():
#         pi.write(LED_PIN, 0)
# finally:
#     # Release resources
#     if 'pi' in locals():
#         pi.stop()


#!/usr/bin/env python3

import gpiod
import time

# For Raspberry Pi 5, the GPIO pins are on gpiochip4
# This may be different depending on your Ubuntu setup
CHIP_NAME = "gpiochip4"
LED_PIN = 17

print("LED Blink Program Started (Press CTRL+C to exit)")

try:
    # Open the GPIO chip
    chip = gpiod.Chip(CHIP_NAME)
    
    # Get the GPIO line (older API style)
    line = chip.get_line(LED_PIN)
    
    # Configure the line as output
    line.request(consumer="LED_Blink", type=gpiod.LINE_REQ_DIR_OUT)
    
    # Blink the LED
    while True:
        line.set_value(1)  # Turn LED on
        print("LED ON")
        time.sleep(1)
        
        line.set_value(0)  # Turn LED off
        print("LED OFF")
        time.sleep(1)
        
except KeyboardInterrupt:
    print("Program stopped by user")
    # Ensure LED is off when exiting
    if 'line' in locals():
        line.set_value(0)
except Exception as e:
    print(f"Error: {e}")
finally:
    # Release resources
    if 'line' in locals():
        line.release()
    if 'chip' in locals():
        chip.close()