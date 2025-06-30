import cv2
import serial
import time
import numpy as np
from threading import Thread
import queue

class OrangeBallTracker:
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
        
        # Ball detection parameters
        self.min_ball_radius = 10  # Minimum radius to detect
        self.detection_interval = 1  # Process every N frames
        self.frame_count = 0
        
        # HSV range for orange color
        # Default values - will be refined with trackbars
        self.lower_orange = np.array([5, 100, 100])
        self.upper_orange = np.array([15, 255, 255])
        
        # Motion smoothing
        self.smoothing_factor = 0.7  # Exponential smoothing factor (0-1)
        self.prev_x = None
        self.prev_y = None
        
        # Status flags
        self.ball_detected = False
        self.is_running = False
        
        # Tracking window for consistent tracking
        self.track_window = None

    def connect_to_arduino(self):
        """Connect to Arduino and initialize servos"""
        try:
            # List available COM ports
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            print("[INFO] Available COM ports:")
            for p in ports:
                print(f"  - {p}")
            
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            
            # Test servos with center position
            print("[INFO] Testing servos with initial position...")
            test_cmd = f"90,90\n".encode()
            self.ser.write(test_cmd)
            time.sleep(0.5)
            
            # Send initial position
            self.send_command(self.current_x, self.current_y)
            print(f"[OK] Connected to Arduino on {self.com_port}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to Arduino: {e}")
            print("[INFO] Try specifying a different COM port when creating the tracker")
            print("Example: tracker = OrangeBallTracker(com_port='COM4')")
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
                        print(f"[DEBUG] Writing to serial: '{command.strip()}'")
                        bytes_written = self.ser.write(command.encode())
                        print(f"[DEBUG] Bytes written: {bytes_written}")
                        
                        # Read any response from Arduino for debugging
                        time.sleep(0.1)
                        if self.ser.in_waiting:
                            response = self.ser.readline().decode('ascii', errors='ignore').strip()
                            print(f"[DEBUG] Arduino response: '{response}'")
                        
                        self.current_x, self.current_y = x, y
                    else:
                        print("[ERROR] Cannot send command - serial not connected")
                    
                    self.command_queue.task_done()
                    
                # Limit command rate to prevent overwhelming Arduino
                time.sleep(0.05)  # 20 commands per second max
            except Exception as e:
                print(f"[ERROR] Command error: {e}")
                time.sleep(0.1)

    def calculate_servo_position(self, ball_center_x, ball_center_y, frame_width, frame_height):
        """Calculate servo positions with PID control for smooth movement"""
        # Map ball position to servo angles (0-180)
        target_x = int(180 - (ball_center_x / frame_width) * 180)  # Reversed for natural tracking
        target_y = int((1 - (ball_center_y / frame_height)) * 180)
        
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

    def create_trackbars(self):
        """Create trackbars for HSV range adjustment"""
        cv2.namedWindow('HSV Trackbars')
        
        # Create trackbars for HSV thresholds
        cv2.createTrackbar('H Min', 'HSV Trackbars', self.lower_orange[0], 179, lambda x: None)
        cv2.createTrackbar('S Min', 'HSV Trackbars', self.lower_orange[1], 255, lambda x: None)
        cv2.createTrackbar('V Min', 'HSV Trackbars', self.lower_orange[2], 255, lambda x: None)
        cv2.createTrackbar('H Max', 'HSV Trackbars', self.upper_orange[0], 179, lambda x: None)
        cv2.createTrackbar('S Max', 'HSV Trackbars', self.upper_orange[1], 255, lambda x: None)
        cv2.createTrackbar('V Max', 'HSV Trackbars', self.upper_orange[2], 255, lambda x: None)
        cv2.createTrackbar('Min Radius', 'HSV Trackbars', self.min_ball_radius, 100, lambda x: None)

    def get_trackbar_values(self):
        """Get current trackbar values"""
        h_min = cv2.getTrackbarPos('H Min', 'HSV Trackbars')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Trackbars')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Trackbars')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Trackbars')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Trackbars')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Trackbars')
        
        self.lower_orange = np.array([h_min, s_min, v_min])
        self.upper_orange = np.array([h_max, s_max, v_max])
        self.min_ball_radius = cv2.getTrackbarPos('Min Radius', 'HSV Trackbars')

    def detect_orange_ball(self, frame):
        """Detect orange ball using HSV color filtering"""
        # Get latest trackbar values
        self.get_trackbar_values()
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for orange color
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Show the mask for debugging
        cv2.imshow('Orange Mask', mask)
        
        # Check if any contours found
        if len(contours) > 0:
            # Find the largest contour (assuming it's the ball)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Calculate minimum enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            
            # Only proceed if the radius is above minimum size
            if radius > self.min_ball_radius:
                # Mark ball as detected
                self.ball_detected = True
                
                # Draw circle around the ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                
                # Display ball data
                cv2.putText(frame, f"Ball: ({int(x)}, {int(y)}), r: {int(radius)}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                return True, int(x), int(y), int(radius)
            
        # No valid ball found
        self.ball_detected = False
        return False, 0, 0, 0

    def start(self):
        """Start ball tracking"""
        # Create trackbars for HSV adjustment
        self.create_trackbars()
        
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

        print("[OK] Orange ball tracking started. Press 'q' to quit.")
        print("Use the HSV Trackbars window to adjust detection parameters.")

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
                    # Detect orange ball
                    found, ball_x, ball_y, ball_radius = self.detect_orange_ball(frame)
                    
                    if found:
                        # Calculate servo positions
                        move_x, move_y = self.calculate_servo_position(
                            ball_x, ball_y, frame_width, frame_height
                        )
                        
                        # Send command to Arduino with debug output
                        if abs(move_x - self.current_x) > 1 or abs(move_y - self.current_y) > 1:
                            print(f"[DEBUG] Sending servo command: X={move_x}, Y={move_y}")
                            self.send_command(move_x, move_y)
                        else:
                            print(f"[DEBUG] Movement too small: current=({self.current_x},{self.current_y}), target=({move_x},{move_y})")
                
                # Draw center cross
                cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), (255, 0, 0), 1)
                cv2.line(frame, (0, frame_center_y), (frame_width, frame_center_y), (255, 0, 0), 1)
                
                # Display servo angles
                cv2.putText(frame, f"Servo X: {int(self.current_x)}, Y: {int(self.current_y)}", 
                            (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Show tracking status
                status_text = "Tracking: Active" if self.ball_detected else "Tracking: Searching"
                status_color = (0, 255, 0) if self.ball_detected else (0, 0, 255)
                cv2.putText(frame, status_text, (frame_width - 200, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

                # Display the HSV values
                cv2.putText(frame, f"HSV Low: [{self.lower_orange[0]}, {self.lower_orange[1]}, {self.lower_orange[2]}]", 
                           (10, frame_height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"HSV High: [{self.upper_orange[0]}, {self.upper_orange[1]}, {self.upper_orange[2]}]", 
                           (10, frame_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Display the frame
                cv2.imshow('Orange Ball Tracking', frame)

                # Check for exit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except Exception as e:
            print(f"Error in ball tracking: {e}")
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
        print("Ball tracking stopped")


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
  
  // Wait for serial connection to establish
  delay(1000);
  
  // Debug message
  Serial.println("Ball Tracker Arduino Starting Up");
  
  // Attach servos
  servoX.attach(9);  // Attach horizontal servo to pin 9
  servoY.attach(10); // Attach vertical servo to pin 10
  
  // Initialize servos to center position
  servoX.write(posX);
  servoY.write(posY);
  
  // Confirm servos initialized
  Serial.println("Servos initialized to center position");
  
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
        // Debug output
        Serial.print("Moving to: ");
        Serial.print(newX);
        Serial.print(",");
        Serial.println(newY);
        
        // Apply with rate limiting to prevent jitter
        posX = newX;
        posY = newY;
        
        // Move servos
        servoX.write(posX);
        servoY.write(posY);
      }
      else {
        Serial.println("Invalid servo angles");
      }
    }
    else {
      Serial.println("Invalid command format");
    }
    
    // Clear the string for next command
    inputString = "";
    stringComplete = false;
  }
  
  // Periodically check if still alive
  static unsigned long lastPing = 0;
  if (millis() - lastPing > 2000) {
    Serial.println("Arduino alive");
    lastPing = millis();
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
    # Ask user for COM port
    import serial.tools.list_ports
    
    # List available ports
    print("\n=== Available COM Ports ===")
    ports = list(serial.tools.list_ports.comports())
    for i, p in enumerate(ports):
        print(f"{i+1}. {p.device} - {p.description}")
    
    if ports:
        try:
            selection = input("\nEnter COM port number (or press Enter for default COM3): ")
            if selection.strip():
                port_index = int(selection) - 1
                if 0 <= port_index < len(ports):
                    com_port = ports[port_index].device
                else:
                    print("Invalid selection, using default COM3")
                    com_port = "COM3"
            else:
                com_port = "COM3"
        except ValueError:
            print("Invalid input, using default COM3")
            com_port = "COM3"
    else:
        print("No COM ports detected! Check if Arduino is connected.")
        com_port = "COM3"
    
    print(f"\nUsing COM port: {com_port}")
    print("Press any key to start tracking, or Ctrl+C to exit...")
    input()
    
    # Create and start tracker
    tracker = OrangeBallTracker(com_port=com_port)
    tracker.start()