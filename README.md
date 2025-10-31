<div align="center">
  <img src="docs/logo_grey.png" alt="SPOTD Logo" width="120"/>
  
  # spotd - Automatic body tracking system
    ⚠️ A problem was encountered with serial communication, fix coming "soon" !!
  
  **An automatic tracking system that uses computer vision to detect body position and control a pan-tilt system via PID control.**
  
  ![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
  ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)
  ![MediaPipe](https://img.shields.io/badge/MediaPipe-latest-orange.svg)
  ![License](https://img.shields.io/badge/license-MIT-blue.svg)
  
  *Developed for the UF22 Automatic Systems course*
  
</div>

---
  > 📘 Complete documentation is now available: `spotd/docs/spotd_docu.pdf` (only read if you are a nerd and know italian)

## Features

- **Real-time** body pose tracking with MediaPipe
- **Modern GUI** based on CustomTkinter
- **PID control system** optimized for pan-tilt servomotors
- **Simulation and hardware modes** for testing and deployment
- **Real-time visualization** of tracking coordinates
- **Auto-detection** of serial ports

## Quick Start

### Prerequisites

  > ⚠️ MediaPipe requires python3.12.x or earlier to run, python3.13.x will NOT work!
- **USB camera** or any other webcam with live feed
- **Arduino** (optional for hardware control)
### Quick Installation

```bash
# Clone the repo
git clone https://github.com/hert1zm/spotd.git
cd spotd

# Install dependencies - if pip doesn't work, use python -m pip install ...
pip install opencv-python mediapipe customtkinter Pillow numpy scipy

# Start the application
python spotd.py
```
## Usage

### Simulation Mode

Spotd automatically starts in simulation mode - stand in front of the webcam and the system will detect your position and display the servo commands in the terminal.

### Hardware Mode
1. Connect an Arduino (or any MCU that supports serial communication and can control 2 servos)
2. Toggle "Hardware Mode" in the GUI
3. Select the correct serial port and click "Connect"
4. The system will start sending angles information to the servos 

## Arduino configuration
This is an example configuration using an Arduino as servo controller.

The system communicates via serial following this format:
```
PAN:<angle>,TILT:<angle>\n
```

**Sketch:**
```cpp
#include <Servo.h>
Servo panServo, tiltServo;

void setup() {
  Serial.begin(9600);
  panServo.attach(9);
  tiltServo.attach(10);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int panIdx = cmd.indexOf("PAN:");
    int tiltIdx = cmd.indexOf("TILT:");
    
    if (panIdx != -1 && tiltIdx != -1) {
      int panAngle = cmd.substring(panIdx + 4, tiltIdx - 1).toInt();
      int tiltAngle = cmd.substring(tiltIdx + 5).toInt();
      
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
      Serial.println("OK");
    }
  }
}
```
---

<div align="center">
  
**© 2025 hert1zm , Pensaci 2 Volte**

</div>
