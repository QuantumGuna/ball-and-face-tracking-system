# 🎯 Ball and Face Tracking System using OpenCV and Arduino

A real-time object tracking system using a webcam and Arduino-controlled servos. This project includes two modes:
- 🟠 Orange Ball Tracking (`ball_track.py`)
- 👤 Human Face Tracking (`face_track.py`)

The servos automatically follow the detected object using computer vision and serial communication.

## 📌 Features

- Real-time video processing with OpenCV
- Object tracking based on HSV color (for orange ball)
- Face detection using Haar Cascade
- PID-style smooth servo motion
- HSV trackbar tuning for dynamic adjustment
- Servo control via serial communication with Arduino

## 🧰 Requirements

Install the required Python packages using:

```bash
pip install -r requirements.txt
```
## requirements.txt includes:
- opencv-python
- pyserial
- numpy

  
## 🔧 Hardware Required
- Arduino Uno (or compatible)
- 2x Servo Motors (e.g., SG90 or MG995)
- USB cable to connect Arduino
- Webcam
- Orange ball (for tracking)


## ⚙️ How It Works
- The Python script captures frames from the webcam.
- It detects an orange ball or a face, depending on the mode.
- It calculates the object's position relative to the frame center.
- Then it sends new servo angles via serial to Arduino to move the camera toward the object.


## ▶️ How to Run
🔶 To Run Orange Ball Tracking:
```bash
python ball_track.py
```
This will open two windows:
- One with the tracking view
- One with HSV sliders to adjust color detection in real time
Make sure the ball is clearly visible in the frame.

## 👤 To Run Face Tracking:
```bash
python face_track.py
```
This will detect and follow human faces using Haar Cascade.

## 📁 Project Structure
```bash
├── ball_track.py       # Orange ball tracking with servo control
├── face_track.py       # Face tracking with servo control
├── requirements.txt    # Python dependencies
├── README.md           # Project documentation
└── test/               # (Optional) Test scripts
    ├── face_test.py    # Face detection only
    ├── tracking.py     # Minimal face-to-servo mapping test
```

## 🛠️ Troubleshooting
- If the COM port is incorrect, change com_port='COM3' in the Python files.
- If servos jitter or don’t move, check power supply and connections.
- Use the HSV sliders in ball_track.py to fine-tune detection based on lighting.

## 👨‍💻 Author
Gunasekhar Bathula
Final Year B.Tech Student
Passionate about Embedded Systems and AI

## 📝 License
This project is licensed under the MIT License.
