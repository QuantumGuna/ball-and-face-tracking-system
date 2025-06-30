import cv2
import serial
import time

# Set your Arduino COM port
ser = serial.Serial('COM3', 9600, timeout=1)  # Change 'COM3' if needed
time.sleep(2)  # Wait for Arduino to reset

# Load Haar Cascade face detector
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Open webcam
cap = cv2.VideoCapture(0)

# Start with servo at center
current_angle = 90
ser.write(f'{current_angle}\n'.encode())

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.1, 5)
    frame_center_x = frame.shape[1] // 2

    if len(faces) > 0:
        (x, y, w, h) = faces[0]  # Take the first detected face
        face_center_x = x + w // 2

        # Map face x-position to servo angle
        angle = int((face_center_x / frame.shape[1]) * 180)
        angle = max(0, min(180, angle))  # Clamp between 0 and 180

        # Send only if change > 2 degrees (to avoid jitter)
        if abs(angle - current_angle) > 2:
            ser.write(f'{angle}\n'.encode())
            print(f"Sent angle: {angle}")
            current_angle = angle

        # Draw on frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (face_center_x, y + h // 2), 5, (0, 0, 255), -1)

    cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (255, 0, 0), 1)

    cv2.imshow('Face Tracking Servo', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
