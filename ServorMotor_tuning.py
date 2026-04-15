import cv2
from picamera2 import Picamera2
import pigpio
import time

# ================= SERVO SETUP =================
pi = pigpio.pi()
SERVO_PIN = 24

# Servo pulse width limits
SERVO_CENTER = 1300
SERVO_LEFT = 2000
SERVO_RIGHT = 600

# Start at center
current_pulse = SERVO_CENTER
pi.set_servo_pulsewidth(SERVO_PIN, current_pulse)

# ================= CAMERA SETUP =================
picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(main={"size": (320, 240)})
)
picam2.start()

print("Manual Servo + Camera Test")
print("Use LEFT / RIGHT arrow keys to move servo")
print("Use UP arrow to center servo")
print("Press 'q' to quit")

try:
    while True:
        frame = picam2.capture_array()
        
        # Draw camera center line
        h, w, _ = frame.shape
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 0, 0), 2)  # blue = center

        cv2.imshow("Camera View", frame)

        # Keyboard input for servo
        key = cv2.waitKey(1) & 0xFF

        if key == 81:  # LEFT arrow
            current_pulse += 50
            if current_pulse > SERVO_LEFT:
                current_pulse = SERVO_LEFT
            pi.set_servo_pulsewidth(SERVO_PIN, current_pulse)
            print(f"LEFT ? pulse width: {current_pulse}")

        elif key == 83:  # RIGHT arrow
            current_pulse -= 50
            if current_pulse < SERVO_RIGHT:
                current_pulse = SERVO_RIGHT
            pi.set_servo_pulsewidth(SERVO_PIN, current_pulse)
            print(f"RIGHT ? pulse width: {current_pulse}")

        elif key == 82:  # UP arrow ? center
            current_pulse = SERVO_CENTER
            pi.set_servo_pulsewidth(SERVO_PIN, current_pulse)
            print(f"CENTER ? pulse width: {current_pulse}")

        elif key == ord('q'):  # quit
            break

finally:
    # ================= CLEANUP =================
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()
    cv2.destroyAllWindows()
    picam2.stop()
    print

