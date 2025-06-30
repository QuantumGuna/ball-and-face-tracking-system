import cv2
import serial
import time
import numpy as np
from threading import Thread
import queue

class FaceTracker:
    def __init__(self, com_port='COM3', baud_rate=9600):
        # Serial communication
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Servo parameters
        self.current_x = 90  # Initial horizontal angle
        self.current_y = 90  # Initial vertical angle
        self.target_x = 90
        self.target_y = 90
        
        # PID controller parameters for smoother motion
        self.kp = 0.3  # Proportional gain
        self.max_step = 5  # Maximum step size per frame
        
        # Thread-safe queue for commands
        self.command_queue = queue.Queue(maxsize=10)
        
        # Face detection parameters
        self.min_face_size = (60, 60)  # Minimum face size to detect
        self.detection_interval = 2  # Process every N frames
        self.frame_count = 0
        
        # Motion smoothing
        self.smoothing_factor = 0.7  # Exponential smoothing factor (0-1)
        self.prev_x = None
        self.prev_y = None
        
        # Status flags
        self.face_detected = False
        self.is_running = False

    def connect_to_arduino(self):
        """Connect to Arduino and initialize servos"""
        try:
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            # Send initial position
            self.send_command(self.current_x, self.current_y)
            print(f"[OK] Connected to Arduino on {self.com_port}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to Arduino: {e}")
            return False

    def send_command(self, x, y):
        """Add servo command to the queue"""
        try:
            self.command_queue.put((x, y), block=False)
        except queue.Full:
            pass  # Skip if queue is full (prevents lag)

    def command_sender_thread(self):
        """Thread that sends commands to Arduino at a controlled rate"""
        while self.is_running:
            try:
                if not self.command_queue.empty():
                    x, y = self.command_queue.get(block=False)
                    
                    # Only send if connected
                    if self.ser and self.ser.is_open:
                        command = f"{int(x)},{int(y)}\n"
                        self.ser.write(command.encode())
                        self.current_x, self.current_y = x, y
                    
                    self.command_queue.task_done()
                    
                # Limit command rate to prevent overwhelming Arduino
                time.sleep(0.05)  # 20 commands per second max
            except Exception as e:
                print(f"Command error: {e}")
                time.sleep(0.1)

    def calculate_servo_position(self, face_center_x, face_center_y, frame_width, frame_height):
        """Calculate servo positions with PID control for smooth movement"""
        # Map face position to servo angles (0-180)
        target_x = int(180 - (face_center_x / frame_width) * 180)  # Reversed for natural tracking
        target_y = int((1 - (face_center_y / frame_height)) * 180)
        
        # Apply constraints
        target_x = max(0, min(180, target_x))
        target_y = max(20, min(160, target_y))  # Limit vertical range
        
        # Smooth values with previous measurements if available
        if self.prev_x is not None and self.prev_y is not None:
            target_x = self.prev_x * self.smoothing_factor + target_x * (1 - self.smoothing_factor)
            target_y = self.prev_y * self.smoothing_factor + target_y * (1 - self.smoothing_factor)
        
        self.prev_x, self.prev_y = target_x, target_y
        
        # Calculate error
        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        
        # Apply proportional control with max step limit
        move_x = self.current_x + max(-self.max_step, min(self.max_step, error_x * self.kp))
        move_y = self.current_y + max(-self.max_step, min(self.max_step, error_y * self.kp))
        
        return int(move_x), int(move_y)

    def start(self):
        """Start face tracking"""
        # Load face detector
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        if face_cascade.empty():
            print("[ERROR] Haar cascade file not found!")
            return

        # Connect to Arduino
        if not self.connect_to_arduino():
            print("[WARNING] Running without servo control")

        # Start command sender thread
        self.is_running = True
        command_thread = Thread(target=self.command_sender_thread, daemon=True)
        command_thread.start()

        # Open camera
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[ERROR] Could not open webcam")
            self.cleanup()
            return
            
        # Set camera resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("[OK] Face tracking started. Press 'q' to quit.")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[ERROR] Failed to grab frame")
                    break

                frame = cv2.flip(frame, 1)  # Mirror image
                frame_height, frame_width = frame.shape[:2]
                frame_center_x = frame_width // 2
                frame_center_y = frame_height // 2

                # Process every N frames for better performance
                self.frame_count += 1
                if self.frame_count % self.detection_interval == 0:
                    # Convert to grayscale for face detection
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
                    # Detect faces with optimized parameters
                    faces = face_cascade.detectMultiScale(
                        gray, 
                        scaleFactor=1.1, 
                        minNeighbors=5, 
                        minSize=self.min_face_size,
                        flags=cv2.CASCADE_SCALE_IMAGE
                    )
                    
                    # Update face tracking status
                    self.face_detected = len(faces) > 0
                    
                    if self.face_detected:
                        # Get largest face (closest to camera)
                        largest_face = max(faces, key=lambda face: face[2] * face[3])
                        x, y, w, h = largest_face
                        
                        # Calculate face center
                        face_center_x = x + w // 2
                        face_center_y = y + h // 2
                        
                        # Calculate servo positions
                        move_x, move_y = self.calculate_servo_position(
                            face_center_x, face_center_y, frame_width, frame_height
                        )
                        
                        # Send command to Arduino if movement is significant
                        if abs(move_x - self.current_x) > 1 or abs(move_y - self.current_y) > 1:
                            self.send_command(move_x, move_y)
                        
                        # Draw rectangle and center point on face
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)
                        
                        # Display face coordinates
                        cv2.putText(frame, f"Face: ({face_center_x}, {face_center_y})", 
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Draw center cross
                cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), (255, 0, 0), 1)
                cv2.line(frame, (0, frame_center_y), (frame_width, frame_center_y), (255, 0, 0), 1)
                
                # Display servo angles
                cv2.putText(frame, f"Servo X: {int(self.current_x)}, Y: {int(self.current_y)}", 
                            (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Show tracking status
                status_text = "Tracking: Active" if self.face_detected else "Tracking: Searching"
                status_color = (0, 255, 0) if self.face_detected else (0, 0, 255)
                cv2.putText(frame, status_text, (frame_width - 200, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

                # Display the frame
                cv2.imshow('Face Tracking System', frame)

                # Check for exit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except Exception as e:
            print(f"Error in face tracking: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up resources"""
        self.is_running = False
        
        # Center servos before exit
        if self.ser and self.ser.is_open:
            self.ser.write("90,90\n".encode())
            time.sleep(0.5)
            self.ser.close()
            
        cv2.destroyAllWindows()
        print("Face tracking stopped")


# Arduino code (for reference) - upload this to your Arduino first
"""
#include <Servo.h>

Servo servoX;  // Horizontal (pan)
Servo servoY;  // Vertical (tilt)

int posX = 90;  // Default position
int posY = 90;  // Default position
String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);
  servoX.attach(9);  // Attach horizontal servo to pin 9
  servoY.attach(10); // Attach vertical servo to pin 10
  
  // Initialize servos to center position
  servoX.write(posX);
  servoY.write(posY);
  
  inputString.reserve(20);
}

void loop() {
  // Process serial commands when complete
  if (stringComplete) {
    // Parse X,Y values from string
    int commaIndex = inputString.indexOf(',');
    if (commaIndex > 0) {
      int newX = inputString.substring(0, commaIndex).toInt();
      int newY = inputString.substring(commaIndex + 1).toInt();
      
      // Validate ranges
      if (newX >= 0 && newX <= 180 && newY >= 0 && newY <= 180) {
        // Apply with rate limiting to prevent jitter
        posX = newX;
        posY = newY;
        
        // Move servos
        servoX.write(posX);
        servoY.write(posY);
      }
    }
    
    // Clear the string for next command
    inputString = "";
    stringComplete = false;
  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Add character to input string
    if (inChar != '\n') {
      inputString += inChar;
    }
    // Set flag when newline received
    else {
      stringComplete = true;
    }
  }
}
"""

if __name__ == "__main__":
    tracker = FaceTracker()
    tracker.start()