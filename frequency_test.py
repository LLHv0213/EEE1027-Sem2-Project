import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ================= GPIO PINS (BCM) =================
IN1 = 17
IN2 = 18
IN3 = 22
IN4 = 23
ENA = 12
ENB = 13

GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

# ================= CONSTANTS =================
FIXED_DUTY = 100      # duty cycle (%)
MOVE_TIME = 1.5      # seconds

# ================= PWM SETUP =================
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)

pwmA.start(0)
pwmB.start(0)

# ================= MOTOR FUNCTIONS =================
def stop():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

def forward():
    # Forward = L H L H
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    pwmA.ChangeDutyCycle(FIXED_DUTY)
    pwmB.ChangeDutyCycle(FIXED_DUTY)

    time.sleep(MOVE_TIME)
    stop()

def backward():
    # Backward = H L H L
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    pwmA.ChangeDutyCycle(FIXED_DUTY)
    pwmB.ChangeDutyCycle(FIXED_DUTY)

    time.sleep(MOVE_TIME)
    stop()

# ================= MAIN LOOP =================
try:
    while True:
        freq_input = input(
            "\nEnter PWM frequency in Hz (press Enter to quit): "
        )

        if freq_input == "":
            break

        try:
            freq = int(freq_input)
        except ValueError:
            print("Invalid input. Enter integer frequency.")
            continue

        if freq <= 0:
            print("Frequency must be positive.")
            continue

        pwmA.ChangeFrequency(freq)
        pwmB.ChangeFrequency(freq)

        print(f"Forward: {freq} Hz, {FIXED_DUTY}% duty, 1 s")
        forward()

        choice = input("Move backward same magnitude? (y/n): ").lower()
        if choice == "y":
            print("Backward motion")
            backward()
        else:
            print("Backward skipped")

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")

