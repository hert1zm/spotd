<div align="center">
  <img src="docs/logo_grey.png" alt="SPOTD Logo" width="120"/>
  
  # spotd - Sistema di Puntatore Automatizzato
  
  **Un sistema di tracking automatico che utilizza visione artificiale per rilevare la posizione del corpo e controllare un sistema pan-tilt tramite controllo PID.**
  
  ![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
  ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)
  ![MediaPipe](https://img.shields.io/badge/MediaPipe-latest-orange.svg)
  ![License](https://img.shields.io/badge/license-MIT-blue.svg)
  
  *Sviluppato per il corso UF22 Sistemi Automatici*
  
</div>

---
  > 📘 La documentazione completa è ora disponibile: `spotd/docs/spotd_docu.pdf`

## Caratteristiche Principali

- **Tracking in tempo reale** delle pose corporee con MediaPipe
- **Interfaccia grafica moderna** sviluppata con CustomTkinter
- **Sistema di controllo PID** ottimizzato per servomotori pan-tilt
- **Modalità simulazione e hardware** per testing e deployment
- **Visualizzazione real-time** delle coordinate di tracking
- **Auto-rilevamento** porte seriali Arduino

## Quick Start

### Installazione Rapida

```bash
# Clona la repo
git clone https://github.com/hert1zm/spotd.git
cd spotd

# Installa le dipendenze - se pip non funziona usa python -m pip install ...
pip install opencv-python mediapipe customtkinter Pillow numpy scipy

# Avvia l'applicazione
python spotd.py
```

### Prerequisiti

- **Python 3.8+**
- **telecamera** USB o qualsiasi altra webcam con live feed
- **Arduino** (opzionale per controllo hardware)

## Utilizzo

### Modalità Simulazione
L'applicazione parte automaticamente in modalità simulazione - posizionati davanti alla webcam e il sistema rileverà la tua posizione mostrando i comandi servo nel terminale.

### Modalità Hardware
1. Connetti Arduino/microcontrollore
2. Attiva "Modalità HW" nell'interfaccia
3. Seleziona porta seriale e clicca "Connetti"
4. Il sistema invierà comandi reali ai servomotori

## Configurazione Arduino

Il sistema comunica via seriale nel formato:
```
PAN:<angolo>,TILT:<angolo>\n
```

**Sketch Arduino di esempio:**
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
