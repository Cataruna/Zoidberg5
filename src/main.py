import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Initialize I2C interface
i2c = busio.I2C(SCL, SDA)

# Initialize PCA9685
pca = PCA9685(i2c)
pca.frequency = 50  # Set PWM frequency for servos (50Hz is typical for servos)

# Define servo channels
servo1 = pca.channels[0]
servo2 = pca.channels[1]

# Helper function to map angle (0-180) to PWM duty cycle
def angle_to_pwm(angle):
    min_pwm = 102  # Minimum PWM for 0 degrees (adjust for your servo)
    max_pwm = 512  # Maximum PWM for 180 degrees (adjust for your servo)
    return int(min_pwm + (angle / 180.0) * (max_pwm - min_pwm))

# Initialize variables
servo1_angle = 0  # Starting angle for Servo 1
servo2_angle = 0  # Starting angle for Servo 2
servo1_speed = 5  # Degrees per step for Servo 1
servo2_speed = 10  # Degrees per step for Servo 2
servo1_direction = 1  # 1 for increasing angle, -1 for decreasing
servo2_direction = 1

last_change_servo1 = time.monotonic()  # Track time for Servo 1
last_change_servo2 = time.monotonic()  # Track time for Servo 2

# Timing intervals (seconds)
servo1_interval = 2  # Change direction every 2 seconds
servo2_interval = 1  # Change direction every 1 second

# Main loop
try:
    while True:
        current_time = time.monotonic()

        # Update Servo 1
        if current_time - last_change_servo1 >= servo1_interval:
            servo1_direction *= -1  # Change direction
            last_change_servo1 = current_time
        
        servo1_angle += servo1_speed * servo1_direction
        if servo1_angle > 180:
            servo1_angle = 180
        elif servo1_angle < 0:
            servo1_angle = 0
        
        servo1.duty_cycle = angle_to_pwm(servo1_angle)

        # Update Servo 2
        if current_time - last_change_servo2 >= servo2_interval:
            servo2_direction *= -1  # Change direction
            last_change_servo2 = current_time
        
        servo2_angle += servo2_speed * servo2_direction
        if servo2_angle > 180:
            servo2_angle = 180
        elif servo2_angle < 0:
            servo2_angle = 0
        
        servo2.duty_cycle = angle_to_pwm(servo2_angle)

        # Small delay for smoother updates
        time.sleep(0.05)

except KeyboardInterrupt:
    # Graceful exit
    pca.deinit()
    print("Program stopped.")
