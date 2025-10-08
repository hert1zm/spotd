"""
==================================================================================
SPOTD - Sistema di Puntatore Automatizzato - UF22 Sistemi automatici

2025 (c) Alessandro Miori - https://github.com/hert1zm
Pensaci 2 Volte.

Descrizione:
Sistema di tracking automatico per che utilizza computer vision per rilevare la 
posizione dell'attore e controlla un sistema pan-tilt tramite PID.
==================================================================================
"""

import os
import cv2
import mediapipe as mp
import customtkinter as ctk
import tkinter as tk
from PIL import Image, ImageTk, ImageDraw, ImageFont
import numpy as np
from scipy.integrate import odeint
import time
import warnings

# Sopprime i warning di CustomTkinter
warnings.filterwarnings("ignore", category=UserWarning, module="customtkinter")

def print_startup_banner():
    """Stampa il banner di avvio dell'applicazione"""
    time.sleep(1) # Aspetta gli spam di warning di MediaPipe
    os.system('cls' if os.name == 'nt' else 'clear')
    banner = """
    ███████╗██████╗  ██████╗ ████████╗██████╗ 
    ██╔════╝██╔══██╗██╔═══██╗╚══██╔══╝██╔══██╗
    ███████╗██████╔╝██║   ██║   ██║   ██║  ██║
    ╚════██║██╔═══╝ ██║   ██║   ██║   ██║  ██║
    ███████║██║     ╚██████╔╝   ██║   ██████╔╝
    ╚══════╝╚═╝      ╚═════╝    ╚═╝   ╚═════╝  
    by Miori Alessandro | https://github.com/hert1zm
    
    ═══════════════════════════════════════════
    """
    print(banner)

def get_serial_ports():
    """Rileva solo le porte seriali con dispositivi disponibili"""
    available_ports = []
    
    try:
        import serial.tools.list_ports
        
        # Scansiona tutte le porte seriali
        ports = serial.tools.list_ports.comports() 
        
        for port in ports:
            # Filtra solo porte con dispositivi 
            if port.device and (port.description != "n/a" and port.description != port.device):
                # Controlla se e' un probabile Arduino/microcontrollore
                description_lower = port.description.lower()
                manufacturer_lower = (port.manufacturer or "").lower()
                
                # Parole chiave che indicano dispositivi seriali utili
                useful_keywords = [
                    'arduino', 'usb', 'serial', 'ch340', 'ch341', 'ftdi', 
                    'cp210', 'cp2102', 'usb-serial', 'silicon labs'
                ]
                
                # Esclude dispositivi inutili
                exclude_keywords = [
                    'bluetooth', 'modem', 'infrared', 'irda'
                ]
                
                is_useful = any(keyword in description_lower or keyword in manufacturer_lower 
                               for keyword in useful_keywords)
                is_excluded = any(keyword in description_lower 
                                 for keyword in exclude_keywords)
                
                if is_useful and not is_excluded:
                    # Formato: "COM3 (Arduino Uno)"
                    display_name = f"{port.device} ({port.description[:30]})"
                    available_ports.append(display_name)
        
        # Se non trova niente, fallback a test rapido di connessione
        if not available_ports:
            #print("\r    SERIAL INFO: Scansione porte seriali...") 
            test_ports = []
            
            # Lista porte da testare
            if os.name == 'nt':  # os.name nt indica Windows
                test_ports = [f'COM{i}' for i in range(1, 21)]
            else:  # Se no dovrei essere su Linux/Mac
                test_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', 
                             '/dev/ttyACM1', '/dev/cu.usbmodem*', '/dev/cu.usbserial*']
            
            for port_name in test_ports:
                try:
                    import serial
                    # Test rapido connessione
                    test_serial = serial.Serial(port_name, 9600, timeout=0.5)
                    test_serial.close()
                    available_ports.append(f"{port_name} (Dispositivo rilevato)")
                    #print(f"\r    SERIAL INFO: Trovato dispositivo su {port_name}")
                except:
                    pass  # Porta non disponibile o occupata
                    
    except ImportError:
        #print("\r    SERIAL INFO: pySerial non installato - usando lista predefinita")
        # Fallback se pySerial non e' installato
        if os.name == 'nt':
            available_ports = ['COM3 (Predefinito)', 'COM4 (Predefinito)'] # Sto schifo va sistemato
        else:
            available_ports = ['/dev/ttyUSB0 (Predefinito)', '/dev/ttyACM0 (Predefinito)']
    
    # Se ancora niente, scrive che non sono disponibili
    if not available_ports:
        available_ports = ["Nessun dispositivo rilevato"]
    
    return available_ports


# ================================
# MODELLI MATEMATICI E CONTROLLO
# ================================

class PanTiltSystem:
    """
    Modello dinamico del sistema pan-tilt.
    Rappresenta il comportamento fisico del sistema meccanico come equazione differenziale
    del secondo ordine: J*θ'' + b*θ' = u
    """
    def __init__(self, J=0.01, b=0.05):
        """
        Inizializza i parametri del sistema fisico
        Args:
            J (float): Momento d'inerzia [kg⋅m²]
            b (float): Coefficiente di attrito viscoso [N⋅m⋅s]
        """
        self.J, self.b = J, b
        self.theta, self.theta_dot = 0.0, 0.0 # Posizione e velocita angolare

    def _dyn(self, state, t, u):
        """
        Definisce le equazioni dinamiche del sistema
        """
        theta, theta_dot = state
        theta_ddot = (u - self.b * theta_dot) / self.J
        return [theta_dot, theta_ddot]

    def update(self, u, dt=0.01):
        """Aggiorna lo stato del sistema integrando le equazioni dinamiche"""
        t = np.linspace(0, dt, 2)
        theta, theta_dot = odeint(self._dyn, [self.theta, self.theta_dot], t, args=(u,))[-1]
        self.theta, self.theta_dot = theta, theta_dot
        return theta

class PIDController:
    """
    Controllore PID digitale per il controllo di posizione angolare
    Implementa l'algoritmo: u = Kp*e + Ki*∫e*dt + Kd*de/dt
    """
    def __init__(self, Kp, Ki, Kd, dt, out_lim=(-10,10)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt, self.out_lim = dt, out_lim
        self.int_err, self.prev_err = 0.0, 0.0 # Errore integrato e precedente

    def compute(self, err):
        """Calcola il segnale di controllo PID"""
        # Integrativo
        self.int_err += err * self.dt
        # Derivativo
        deriv = (err - self.prev_err) / self.dt
        self.prev_err = err
        # PID completo
        out = self.Kp*err + self.Ki*self.int_err + self.Kd*deriv
        return float(np.clip(out, *self.out_lim))

class ServoInterface:
    """
    Interfaccia per il controllo dei servomotori pan-tilt
    Converte gli angoli calcolati in comandi per servomotori
    """
    def __init__(self):
        self.hw = False # Flag per connessione hardware reale
        self.first_print = True # Flag per prima stampa
        self.serial_port = None # Flag per la porta seriale utilizzata
        self.connection_status = "Disconnesso"

    def connect_hardware(self, port_name):
        """Gestisce connessione/disconnessione hardware"""
        if not hasattr(self, 'pointer'):
            return
            
        if self.pointer.servo.hw:
            # Disconnetti
            self.pointer.servo.disconnect_hardware()
            self.connect_btn.configure(text="Connetti")
            self.hw_status_label.configure(text="Disconnesso", text_color="#ff6b6b")
        else:
            # Connetti
            selected_port = self.port_var.get()
            if selected_port and selected_port != "Seleziona porta" and "Nessun dispositivo" not in selected_port:
                # Estrai solo il nome della porta (tipo COM3)
                port_name = selected_port.split(' ')[0]
                
                if self.pointer.servo.connect_hardware(port_name):
                    self.connect_btn.configure(text="Disconnetti")
                    self.hw_status_label.configure(text=f"Connesso su {port_name}", text_color="#51cf66")
                else:
                    self.hw_status_label.configure(text="Errore connessione", text_color="#ff6b6b")
            else:
                self.hw_status_label.configure(text="Seleziona una porta valida", text_color="#ff6b6b")


    def disconnect_hardware(self):
        """Disconnette dall'hardware"""
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        self.hw = False
        self.connection_status = "Disconnesso"
        #print("\r    SERIAL INFO: Hardware disconnesso")

    def _clip(self, a):
        """Limita gli angoli nell'intervallo 0-180° per servomotori standard"""
        return int(np.clip(a, 0, 180))

    def set_angles(self, p, t):
        """
        Imposta gli angoli di pan e tilt
        Args:
            p (float): Angolo di pan [gradi]
            t (float): Angolo di tilt [gradi]
        """
        p, t = self._clip(p), self._clip(t)
        
        if self.hw and self.serial_port:
            # MODALITA HARDWARE REALE
            try:
                # Invia comandi al mcu tramite seriale
                command = f"PAN:{p},TILT:{t}\n"
                self.serial_port.write(command.encode())
                
                # Legge eventuale risposta da Arduino
                if self.serial_port.in_waiting > 0:
                    response = self.serial_port.readline().decode().strip()
                    if response and len(response) > 0:
                        #print(f"\r    SERIAL INFO: Arduino: {response}")
                        pass
                        
            except Exception as e:
                #print(f"\r    SERIAL INFO: Errore comunicazione Arduino: {e}")
                self.connection_status = f"Errore com: {str(e)}"
        else:
            # MODALITA SIMULAZIONE
            if self.first_print:
                print() # Riga vuota all'inizio
                self.first_print = False
            print(f"\r    SERVO ▶ Pan: {p:3d}° | Tilt: {t:3d}° | Status: TRACKING ", end="", flush=True)

class Pointer:
    """Sistema completo di puntamento"""
    def __init__(self):
        """Inizializza il sistema di puntamento con parametri ottimizzati"""
        # Sistemi dinamici (parametri diversi per pan e tilt)
        self.pan = PanTiltSystem(0.01, 0.05) # Pan: piu veloce
        self.tilt = PanTiltSystem(0.008, 0.04) # Tilt: leggermente piu basso
        
        # Controllori PID tarati per prestazioni ottimali
        self.pid_pan = PIDController(3.0, 1.2, 0.15, 0.02)
        self.pid_tilt = PIDController(2.5, 1.0, 0.1, 0.02)
        
        # Interfaccia hardware
        self.servo = ServoInterface()
        self.dt = 0.02 # Periodo di controllo 50Hz

    def step(self, pan_t, tilt_t):
        """Esegue un passo di controllo per entrambi gli assi"""
        # Calcolo degli errori e segnali di controllo
        u_pan = self.pid_pan.compute(pan_t - self.pan.theta)
        u_tilt = self.pid_tilt.compute(tilt_t - self.tilt.theta)
        
        # Aggiornamento dei modelli dinamici
        self.pan.update(u_pan, self.dt)
        self.tilt.update(u_tilt, self.dt)
        
        # Conversione a gradi e offset per servomotori (0-180°)
        pd = np.degrees(self.pan.theta) + 90
        td = np.degrees(self.tilt.theta) + 90
        
        # Invio comandi ai servomotori
        self.servo.set_angles(pd, td)
        return pd, td

# ===========================
# INTERFACCIA GRAFICA
# ===========================

class SpotdApp:
    """
    Applicazione principale con interfaccia grafica per il sistema
    """
    def __init__(self):
        """Inizializza l'applicazione e avvia il loop principale"""
        # Configurazione tema scuro
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        # Inizializzazione componenti
        self.setup_window()
        self.setup_camera()
        self.setup_ui()
        self.setup_tracking()
        
        # Stampo il banner di startup
        print_startup_banner()
        
        # Avvio ciclo di aggiornamento
        self.update()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def setup_window(self):
        """Configura la finestra principale dell'applicazione"""
        self.root = ctk.CTk()
        self.root.title("spotd")
        self.root.geometry("1400x850")
        self.root.configure(fg_color="#000000")

    def setup_camera(self):
        """Inizializza la telecamera e il sistema di pose detection MediaPipe"""
        mp_pose = mp.solutions.pose
        self.pose = mp_pose.Pose(
            static_image_mode=False, # Video in tempo reale
            model_complexity=1, # Bilancio qualita'/velocita'
            enable_segmentation=False, # Solo keypoint, no segmentazione
            min_detection_confidence=0.5, # Soglia rilevamento
            min_tracking_confidence=0.5 # Soglia tracking
        )
        
        # Apertura telecamera (webcam principale) - Per modificare quale webcam utilizzare in caso di multiple, modificare l'indice di VideoCapture()
        self.cap = cv2.VideoCapture(0)
        self.fw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.fh = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def setup_ui(self):
        """Costruisce l'interfaccia utente con layout a tre pannelli"""
        # Container principale
        main = ctk.CTkFrame(self.root, fg_color="#000000")
        main.pack(fill="both", expand=True)

        # === SIDEBAR SINISTRA ===
        sidebar = ctk.CTkFrame(main, width=160, fg_color="#111111")  # Piu' larga per controlli hardware
        sidebar.pack(side="left", fill="y")
        sidebar.pack_propagate(False)

        # Logo
        SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        logo_path = os.path.join(SCRIPT_DIR, "docs/logo_white.png")
        
        try:
            # Caricamento e ridimensionamento logo
            orig = Image.open(logo_path)
            h = 80  # Logo piu' piccolo per far spazio
            w = int(orig.width * (h / orig.height))
            resized = orig.resize((w,h), Image.LANCZOS)
            self.logo_img = ctk.CTkImage(light_image=resized, dark_image=resized, size=(w,h))
            ctk.CTkLabel(sidebar, image=self.logo_img, text="").pack(pady=15)
        except:
            # Fallback testuale se logo non disponibile :(
            ctk.CTkLabel(sidebar, text="SPOTD", font=ctk.CTkFont("Inter", 20, "bold")).pack(pady=15)

        # === CONTROLLI HARDWARE ===
        hw_frame = ctk.CTkFrame(sidebar, fg_color="#222222")
        hw_frame.pack(pady=10, padx=10, fill="x")
        
        ctk.CTkLabel(hw_frame, text="Hardware Control", 
                    font=ctk.CTkFont("Inter", 12, "bold")).pack(pady=5)
        
        # Switch modalita' hardware
        self.hw_switch = ctk.CTkSwitch(hw_frame, text="Modalita' HW", 
                                      command=self.toggle_hardware_mode)
        self.hw_switch.pack(pady=5)
        
        # Menu porte seriali
        self.port_var = ctk.StringVar(value="Seleziona porta")
        self.port_menu = ctk.CTkOptionMenu(hw_frame, variable=self.port_var, 
                                          values=get_serial_ports())
        self.port_menu.pack(pady=5, padx=5, fill="x")
        
        # Pulsante connessione
        self.connect_btn = ctk.CTkButton(hw_frame, text="Connetti", 
                                        command=self.connect_hardware, state="disabled")
        self.connect_btn.pack(pady=5, padx=5, fill="x")
        
        # Status hardware
        self.hw_status_label = ctk.CTkLabel(hw_frame, text="Disconnesso", 
                                           font=ctk.CTkFont("Inter", 10), text_color="#ff6b6b")
        self.hw_status_label.pack(pady=2)

        # === CONTROLLI STANDARD ===
        # Pulsanti di controllo
        self.btn_capture = ctk.CTkButton(sidebar, text="Capture", command=self.capture_frame)
        self.btn_capture.pack(pady=(10,5), padx=10, fill="x")
        
        self.btn_record = ctk.CTkButton(sidebar, text="Record", command=self.toggle_record)
        self.btn_record.pack(pady=5, padx=10, fill="x")
        
        # Indicatore stato registrazione
        self.rec_indicator = ctk.CTkLabel(sidebar, text="● REC OFF", text_color="#ff6b6b")
        self.rec_indicator.pack(pady=(5,20))

        # === AREA CONTENUTO CENTRALE ===
        content = ctk.CTkFrame(main, fg_color="transparent")
        content.pack(side="right", fill="both", expand=True, padx=20, pady=20)

        # === PANNELLO SINISTRO (Info & Tracking) ===
        left = ctk.CTkFrame(content, width=300, fg_color="#111111")
        left.pack(side="left", fill="y", padx=(0,20))
        left.pack_propagate(False)

        # Labels informativi
        self.status_label = ctk.CTkLabel(left, text="● Initializing...",
                                        font=ctk.CTkFont("Inter",12), text_color="#ff6b6b")
        self.status_label.pack(pady=(20,10))
        
        self.position_label = ctk.CTkLabel(left, text="X: N/A, Y: N/A",
                                          font=ctk.CTkFont("Inter",12), text_color="#cccccc")
        self.position_label.pack(pady=5)
        
        self.servo_label = ctk.CTkLabel(left, text="Pan: N/A | Tilt: N/A",
                                       font=ctk.CTkFont("Inter",12), text_color="#cccccc")
        self.servo_label.pack(pady=5)

        # Canvas di tracking con griglia 8x8
        self.canvas = tk.Canvas(left, width=250, height=250, bg="#000000", highlightthickness=0)
        self.canvas.pack(pady=20)
        
        # Creazione griglia di riferimento
        step = 250 // 8
        for i in range(0,250,step):
            self.canvas.create_line(i,0,i,250,fill="#333333")
            self.canvas.create_line(0,i,250,i,fill="#333333")
        
        # Punto di tracking (pallino rosso)
        self.dot_rad = 5
        self.dot_id = self.canvas.create_oval(125-self.dot_rad,125-self.dot_rad,
                                             125+self.dot_rad,125+self.dot_rad,
                                             fill="#ff4757",outline="#ffffff",width=2)

        # === PANNELLO DESTRO (feed live) ===
        right = ctk.CTkFrame(content, fg_color="#111111")
        right.pack(side="right", fill="both", expand=True)
        
        ctk.CTkLabel(right, text="Live Feed",
                    font=ctk.CTkFont("Inter",20,"bold"), text_color="#ffffff").pack(pady=(20,10))
        
        # Frame per il video
        vf = ctk.CTkFrame(right, fg_color="#000000")
        vf.pack(fill="both", expand=True, padx=20, pady=(0,20))
        
        self.video_label = ctk.CTkLabel(vf, text="")
        self.video_label.pack(expand=True)

    def toggle_hardware_mode(self):
        """Attiva/disattiva modalita' hardware"""
        if self.hw_switch.get():
            # Modalita' hardware attivata
            self.connect_btn.configure(state="normal")
            self.port_menu.configure(state="normal")
        else:
            # Modalita' simulazione
            self.connect_btn.configure(state="disabled")
            self.port_menu.configure(state="disabled")
            if hasattr(self, 'pointer'):
                self.pointer.servo.disconnect_hardware()
                self.hw_status_label.configure(text="Disconnesso", text_color="#ff6b6b")

    def connect_hardware(self):
        """Gestisce connessione/disconnessione hardware"""
        if not hasattr(self, 'pointer'):
            return
            
        if self.pointer.servo.hw:
            # Disconnetti
            self.pointer.servo.disconnect_hardware()
            self.connect_btn.configure(text="Connetti")
            self.hw_status_label.configure(text="Disconnesso", text_color="#ff6b6b")
        else:
            # Connetti
            selected_port = self.port_var.get()
            if selected_port != "Seleziona porta":
                if self.pointer.servo.connect_hardware(selected_port):
                    self.connect_btn.configure(text="Disconnetti")
                    self.hw_status_label.configure(text=f"Connesso su {selected_port}", text_color="#51cf66")
                else:
                    self.hw_status_label.configure(text="Errore connessione", text_color="#ff6b6b")

    def setup_tracking(self):
        """Inizializza i parametri per il sistema di tracking stabilizzato"""
        # Sistema di controllo
        self.pointer = Pointer()
        
        # Parametri posizione e trail
        self.sx = self.sy = 125 # Posizione iniziale centro canvas
        self.trail = [] # Lista punti trail
        self.trail_len = 25 # Lunghezza massima trail
        
        # Parametri di stabilizzazione
        self.alpha = 0.95 # Fattore smoothing (aumentato per stabilita')
        self.last_time = time.time()
        
        # Campo visivo telecamera (per ora messo a 70 e 55 per la webcam)
        self.fov_h, self.fov_v = 70, 55 # Gradi orizzontale e verticale
        
        # Flag registrazione
        self.recording = False
        
        # Landmark centrali del corpo per tracking stabile
        self.core_landmarks = [
            11, 12, # spalle (left_shoulder, right_shoulder)
            23, 24, # anche (left_hip, right_hip)
        ]
        
        # Parametri controllo stabilita'
        self.movement_threshold = 15 # Pixel di movimento minimo
        self.stable_counter = 0 # Contatore frames stabili
        self.stability_frames = 5 # Frames per considerare stabile

    def capture_frame(self):
        """Cattura e salva un frame della telecamera"""
        ret, frame = self.cap.read()
        if ret:
            SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
            save_path = os.path.join(SCRIPT_DIR, "capture.png") #Salva l'immagine nello stesso path dello script
            cv2.imwrite(save_path, frame)
            self.status_label.configure(text="● Captured", text_color="#51cf66")

    def toggle_record(self):
        """Attiva/disattiva la modalita' registrazione (placeholder perche' non e' implemwntata)"""
        self.recording = not self.recording
        color = "#ff6b6b" if not self.recording else "#51cf66"
        text = "● REC OFF" if not self.recording else "● REC ON"
        self.rec_indicator.configure(text=text, text_color=color)

    def pix2angle(self, cx, cy, w, h):
        """Converte coordinate pixel in angoli per il sistema pan-tilt"""
        # Normalizzazione coordinate (-0.5 a +0.5)
        dx = (cx - w/2) / w
        dy = (cy - h/2) / h
        # Conversione in angoli basata su FOV telecamera
        return np.deg2rad(dx*self.fov_h), np.deg2rad(dy*self.fov_v)

    def plot_skeleton(self, frame, lms, vis_th=0.3):
        """Disegna lo scheletro pose sulla frame video"""
        ih, iw = frame.shape[:2]
        
        # Disegna connessioni scheletro
        for a,b in mp.solutions.pose.POSE_CONNECTIONS:
            if lms[a].visibility>=vis_th and lms[b].visibility>=vis_th:
                x1,y1 = int(lms[a].x*iw), int(lms[a].y*ih)
                x2,y2 = int(lms[b].x*iw), int(lms[b].y*ih)
                cv2.line(frame,(x1,y1),(x2,y2),(0,255,100),2)
        
        # Disegna keypoint
        for lm in lms:
            if lm.visibility>=vis_th:
                x,y=int(lm.x*iw),int(lm.y*ih)
                cv2.circle(frame,(x,y),3,(255,100,100),-1)

    def update(self):
        """Loop principale di aggiornamento dell'applicazione"""
        # === GESTIONE STATO TELECAMERA ===
        if not self.cap.isOpened():
            # Telecamera disconnessa - mostra placeholder
            self.status_label.configure(text="● Webcam Disconnessa", text_color="#ff6b6b")
            
            # Crea immagine placeholder
            ph,pw=400,300
            img=Image.new("RGB",(ph,pw),(0,0,0))
            d=ImageDraw.Draw(img)
            fnt=ImageFont.load_default()
            d.text((10,pw//2-10),"Nessun live feed",(200,200,200),font=fnt)
            imgtk=ImageTk.PhotoImage(img)
            self.video_label.configure(image=imgtk)
            self.video_label.image=imgtk
            
            # Tentativo riconnessione
            try: self.cap.open(0)
            except: pass
            
            self.root.after(1000,self.update)
            return

        # === ACQUISIZIONE FRAME ===
        ret,frame=self.cap.read()
        if not ret:
            self.cap.release()
            self.root.after(10,self.update)
            return

        # Flip orizzontale
        frame=cv2.flip(frame,1)
        rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        
        # === PROCESSING POSE DETECTION ===
        res=self.pose.process(rgb)

        # Inizializzazione variabili stato
        st="● No Body" # Status text
        sc="#ff6b6b" # Status color
        pdeg=tdeg=None
        
        if res.pose_landmarks:
            # Pose rilevata
            st="● Tracking"
            sc="#51cf66"
            lms = res.pose_landmarks.landmark
            
            # === ALGORITMO TRACKING STABILIZZATO ===
            # Estrazione landmark centrali del corpo
            core_points = []
            for idx in self.core_landmarks:
                if idx < len(lms) and lms[idx].visibility >= 0.5:
                    core_points.append((lms[idx].x, lms[idx].y))
            
            if core_points:
                # Calcolo centro di massa dei landmark centrali
                xs = [p[0] for p in core_points]
                ys = [p[1] for p in core_points]
                mx = sum(xs) / len(xs)
                my = sum(ys) / len(ys)
                
                # Conversione coordinate
                cx, cy = int(mx * frame.shape[1]), int(my * frame.shape[0])
                nx, ny = int(cx / frame.shape[1] * 250), int(cy / frame.shape[0] * 250)
                
                # === CONTROLLO STABILITA' ===
                movement_distance = np.sqrt((nx - self.sx)**2 + (ny - self.sy)**2)
                
                if movement_distance > self.movement_threshold:
                    # Movimento significativo - aggiorna posizione
                    self.sx = int(self.alpha * self.sx + (1 - self.alpha) * nx)
                    self.sy = int(self.alpha * self.sy + (1 - self.alpha) * ny)
                    self.stable_counter = 0
                else:
                    # Movimento minimo - incrementa stabilita'
                    self.stable_counter += 1
                    if self.stable_counter < self.stability_frames:
                        # Piccoli aggiustamenti
                        self.sx = int(0.98 * self.sx + 0.02 * nx)
                        self.sy = int(0.98 * self.sy + 0.02 * ny)
                    # Dopo stability_frames il punto si ferma completamente
                
                # === AGGIORNAMENTO VISUALIZZAZIONE ===
                # Posizione pallino sul canvas
                self.canvas.coords(self.dot_id, self.sx-self.dot_rad, self.sy-self.dot_rad,
                                 self.sx+self.dot_rad, self.sy+self.dot_rad)
                self.position_label.configure(text=f"X: {self.sx}, Y: {self.sy}")
                
                # === CONTROLLO PID ===
                # Eseguito solo durante movimento per evitare oscillazioni
                now = time.time()
                if now - self.last_time >= self.pointer.dt and self.stable_counter < self.stability_frames:
                    pdeg, tdeg = self.pointer.step(*self.pix2angle(cx, cy, frame.shape[1], frame.shape[0]))
                    self.last_time = now
                
                # === GESTIONE TRAIL ===
                self.trail.append((self.sx, self.sy))
                if len(self.trail) > self.trail_len:
                    self.trail.pop(0)
            else:
                # Nessun landmark valido
                self.position_label.configure(text="X: ---, Y: ---")
            
            # === DISEGNO TRAIL SUL CANVAS ===
            self.canvas.delete("trail")
            for i in range(1,len(self.trail)):
                x1,y1=self.trail[i-1]
                x2,y2=self.trail[i]
                # Gradiente colore trail (piu' recente = piu' luminoso)
                a=i/len(self.trail)
                col=f"#{int(255*a):02x}{int(71*a):02x}{int(87*a):02x}"
                self.canvas.create_line(x1,y1,x2,y2,fill=col,width=2,tags="trail")
            
            # Disegna scheletro sul video
            self.plot_skeleton(frame,lms)
        else:
            # Nessuna pose rilevata
            self.position_label.configure(text="X: N/A, Y: N/A")
            self.canvas.delete("trail")
            self.trail.clear()

        # === AGGIORNAMENTO INTERFACCIA ===
        # Status tracking
        self.status_label.configure(text=st,text_color=sc)
        
        # Angoli servomotori
        if pdeg is not None:
            self.servo_label.configure(text=f"Pan: {pdeg:.0f}° | Tilt: {tdeg:.0f}°")
        else:
            self.servo_label.configure(text="Pan: N/A | Tilt: N/A")

        # === DISPLAY VIDEO ===
        # Ridimensionamento per display
        dh=800
        ar=frame.shape[1]/frame.shape[0]
        dw=int(dh*ar)
        frm=cv2.resize(frame,(dw,dh))
        rgb2=cv2.cvtColor(frm,cv2.COLOR_BGR2RGB)
        imgtk=ImageTk.PhotoImage(Image.fromarray(rgb2))
        self.video_label.configure(image=imgtk)
        self.video_label.image=imgtk

        # Programmazione prossimo aggiornamento (100Hz)
        self.root.after(10,self.update)

    def on_close(self):
        """Pulizia risorse alla chiusura dell'applicazione"""
        if hasattr(self, 'pointer') and self.pointer.servo.serial_port:
            self.pointer.servo.disconnect_hardware()
        self.cap.release()
        self.root.destroy()

if __name__ == "__main__": # Avvio applicazione 
    SpotdApp()

