import tkinter as tk
from tkinter import ttk
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports
import socket
import select
import os
import threading
import time
import math

matplotlib.use("TkAgg")

# ==============================================================================
# CONFIGURACIÓN ESTÉTICA (DARK HUD / CYBER INSTRUMENTATION)
# ==============================================================================
COLOR_BG = "#060B14"
COLOR_PANEL = "#0C1524"
COLOR_CARD = "#08101E"
COLOR_BORDER = "#142238"
COLOR_CYAN = "#00F0FF"
COLOR_MAGENTA = "#FF007F"
COLOR_AMBER = "#FFB703"
COLOR_GREEN = "#00FF66"
COLOR_TEXT_DIM = "#5D7394"
COLOR_TEXT_WHITE = "#E6F0FA"
COLOR_GRID = "#0E1C30"
FONT_MONO = "Consolas"

# n = muestras por vector. La consigna pide n entre 120 y 720, con 360 inicial.
# Es una variable GLOBAL que se actualiza con el control "n" (ver cambiar_n): todo el
# código que genera y analiza vectores la lee en el momento de usarla.
N_MIN, N_MAX, N_INICIAL = 120, 720, 360
PUNTOS = N_INICIAL
R_CARGA_INICIAL = 100.0   # ohm (consigna: carga resistiva de 100 ohm)

TCP_PUERTO = 5000                       # debe coincidir con TCP_PUERTO del firmware
ARCHIVO_IP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "esp32_ip.txt")


class ConexionTCP:
    """Conexión por WiFi al ESP32 con la MISMA interfaz que serial.Serial.

    Así el resto de la app (transmitir_al_esp32, hilo_lectura_serie) funciona igual
    sin saber si habla por cable o por WiFi. Solo usa: is_open, in_waiting,
    readline(), write(), flush(), close().
    """

    def __init__(self, host, puerto=TCP_PUERTO, timeout_conexion=3.0):
        self.sock = socket.create_connection((host, puerto), timeout=timeout_conexion)
        # Timeout largo solo para ESCRIBIR: el ESP32 puede estar calculando unos segundos
        # y mientras tanto no vacía su buffer de recepción; sendall espera sin fallar.
        self.sock.settimeout(15.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""
        self.is_open = True

    def _llenar(self, espera=0.02):
        # select evita bloquear: solo se lee si ya hay datos esperando.
        try:
            listo, _, _ = select.select([self.sock], [], [], espera)
            if listo:
                datos = self.sock.recv(4096)
                if not datos:                 # el ESP32 cerró la conexión
                    self.is_open = False
                else:
                    self.buf += datos
        except OSError:
            self.is_open = False

    @property
    def in_waiting(self):
        self._llenar(0)
        return len(self.buf)

    def readline(self):
        limite = time.time() + 0.5
        while b"\n" not in self.buf and self.is_open and time.time() < limite:
            self._llenar(0.05)
        if b"\n" in self.buf:
            linea, self.buf = self.buf.split(b"\n", 1)
            return linea + b"\n"
        return b""

    def reset_input_buffer(self):
        self.buf = b""

    def write(self, datos):
        self.sock.sendall(datos)

    def flush(self):
        pass

    def close(self):
        self.is_open = False
        try:
            self.sock.close()
        except OSError:
            pass


class OsciloscopioApp:
    def __init__(self, root):
        self.root = root
        self.root.title("FOURIER POWER ANALYZER // ESP32 INTERFACE")
        self.root.geometry("1420x760")
        self.root.configure(bg=COLOR_BG)

        # Variables de control y concurrencia
        self.running_thread = True
        self.serial_port = None
        self._lectura_id = 0            # identifica al hilo de lectura vigente
        self.disparo_armado = False     # True mientras el ESP32 está disparando
        self.bloqueo_tx_echo = False

        # Catálogo de señales
        self.senales_disponibles = [
            "1. Seno (Senoidal Pura)",
            "2. Cuadrada (Simetrica)",
            "3. Cuasi Senoidal (120°)",
            "4. RMMOC (Media Onda Controlado)",
            "5. RMOCC (Onda Completa Controlado)",
            "6. RTMO (Trifasico Media Onda)",
            "7. ADC (V e I medidos por el ESP32)"
        ]

        # Última captura del ADC (la manda el ESP32 en la línea ADC_DATA):
        # tensión normalizada (-1..1) y corriente en amperes, n muestras cada una.
        self.adc_v = []
        self.adc_i = []
        
        self.tipo_var = tk.StringVar(value=self.senales_disponibles[0])
        self.armonicos_var = tk.IntVar(value=5)
        self.vmax_var = tk.DoubleVar(value=311.13)
        self.freq_var = tk.StringVar(value="50Hz")
        self.alfa_var = tk.DoubleVar(value=45.0)

        # Vectores de dominio temporal y espectral
        self.deg_x = [360.0 * (k / PUNTOS) for k in range(PUNTOS)]
        self.v_orig = [0.0] * PUNTOS
        self.i_orig = [0.0] * PUNTOS
        self.v_recon = [0.0] * PUNTOS

        self.telemetry_rows = {}
        self.metricas_teoricas = {p: 0.0 for p in (
            "Vmax", "Vmin", "Vavg", "Vrms",
            "Imax", "Imin", "Iavg", "Irms",
            "S", "P", "Q", "D", "FP", "THDv", "THDi"
        )}

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.crear_layout()
        self.actualizar_estado_widgets()
        self.generar_vectores_locales()
        self.actualizar_graficos()
        self.auto_conectar_serial()

    # ==========================================================================
    # CONSTRUCCIÓN DE LA INTERFAZ
    # ==========================================================================
    def crear_layout(self):
        # 1. BARRA SUPERIOR DE PARÁMETROS
        header = tk.Frame(self.root, bg=COLOR_PANEL, highlightthickness=1, highlightbackground=COLOR_BORDER)
        header.pack(side=tk.TOP, fill=tk.X, padx=15, pady=(12, 6))

        ctrl_frame = tk.Frame(header, bg=COLOR_PANEL)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.X, padx=10, pady=8)

        # Selector de Señal
        tk.Label(ctrl_frame, text="SIGNAL SOURCE ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        
        combo_style = ttk.Style()
        combo_style.theme_use("clam")
        combo_style.configure("Custom.TCombobox", fieldbackground="#08101E",
                                background=COLOR_CARD, foreground=COLOR_TEXT_WHITE,
                                selectbackground="#142845", selectforeground=COLOR_CYAN)
        
        self.combo_senal = ttk.Combobox(ctrl_frame, textvariable=self.tipo_var,
                                        values=self.senales_disponibles, width=33,
                                        style="Custom.TCombobox", state="readonly")
        self.combo_senal.pack(side=tk.LEFT, padx=(0, 12))
        self.combo_senal.bind("<<ComboboxSelected>>", self.on_cambio_parametro)

        # Control de Armónicos N
        tk.Label(ctrl_frame, text="N ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        
        self.spin_arm = tk.Spinbox(ctrl_frame, from_=1, to=300, textvariable=self.armonicos_var,
                                   width=4, font=(FONT_MONO, 9), bg="#08101E",
                                   fg=COLOR_CYAN, insertbackground=COLOR_CYAN,
                                   highlightthickness=1, highlightbackground=COLOR_BORDER,
                                   command=self.on_cambio_parametro)
        self.spin_arm.pack(side=tk.LEFT, padx=(0, 12))
        self.spin_arm.bind("<Return>", self.on_cambio_parametro)
        self.spin_arm.bind("<KeyRelease>", self.on_cambio_parametro)

        # Control de Tensión Vmax
        tk.Label(ctrl_frame, text="Vmax [V] ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        
        self.entry_vmax = tk.Entry(ctrl_frame, textvariable=self.vmax_var, width=7,
                                   font=(FONT_MONO, 9), bg="#08101E", fg=COLOR_TEXT_WHITE,
                                   insertbackground=COLOR_CYAN, highlightthickness=1,
                                   highlightbackground=COLOR_BORDER, justify="center")
        self.entry_vmax.pack(side=tk.LEFT, padx=(0, 12))
        self.entry_vmax.bind("<Return>", self.on_cambio_parametro)
        self.entry_vmax.bind("<KeyRelease>", self.on_cambio_parametro)

        # Control de Frecuencia
        tk.Label(ctrl_frame, text="FREQ ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        
        self.entry_freq = tk.Entry(ctrl_frame, textvariable=self.freq_var, width=6,
                                   font=(FONT_MONO, 9), bg="#08101E", fg=COLOR_TEXT_WHITE,
                                   insertbackground=COLOR_CYAN, highlightthickness=1,
                                   highlightbackground=COLOR_BORDER, justify="center")
        self.entry_freq.pack(side=tk.LEFT, padx=(0, 12))
        self.entry_freq.bind("<Return>", self.on_cambio_parametro)
        self.entry_freq.bind("<KeyRelease>", self.on_cambio_parametro)

        # Control del Ángulo Alpha (α)
        self.lbl_alfa = tk.Label(ctrl_frame, text="ALPHA α [°] ▶", font=(FONT_MONO, 9, "bold"),
                                 fg=COLOR_TEXT_DIM, bg=COLOR_PANEL)
        self.lbl_alfa.pack(side=tk.LEFT, padx=(0, 4))
        
        self.spin_alfa = tk.Spinbox(ctrl_frame, from_=0.0, to=180.0, increment=5.0,
                                    textvariable=self.alfa_var, width=5,
                                    font=(FONT_MONO, 9), bg="#08101E", fg=COLOR_AMBER,
                                    insertbackground=COLOR_AMBER, highlightthickness=1,
                                    highlightbackground=COLOR_BORDER,
                                    command=self.on_cambio_parametro)
        self.spin_alfa.pack(side=tk.LEFT, padx=(0, 12))
        self.spin_alfa.bind("<Return>", self.on_cambio_parametro)
        self.spin_alfa.bind("<KeyRelease>", self.on_cambio_parametro)
        self.spin_alfa.bind("<ButtonRelease-1>", self.on_cambio_parametro)

        # Botón TX
        btn_send = tk.Button(ctrl_frame, text="[ TX SYNC ]", font=(FONT_MONO, 8, "bold"),
                             bg="#0E223D", fg=COLOR_CYAN, activebackground="#163866",
                             activeforeground="#FFFFFF", bd=0, padx=10, pady=2,
                             command=self.transmitir_al_esp32)
        btn_send.pack(side=tk.LEFT, padx=(0, 10))

        # Indicador de Enlace Serial
        self.lbl_status = tk.Label(header, text="● SEARCHING ESP32...", font=(FONT_MONO, 9, "bold"),
                                   fg=COLOR_AMBER, bg=COLOR_PANEL)
        self.lbl_status.pack(side=tk.RIGHT, padx=12)

        # Conexión por WiFi: IP del ESP32 (la imprime en el monitor serie: WIFI_READY,<IP>)
        btn_wifi = tk.Button(header, text="[ WIFI ]", font=(FONT_MONO, 8, "bold"),
                             bg="#0E223D", fg=COLOR_CYAN, activebackground="#163866",
                             activeforeground="#FFFFFF", bd=0, padx=10, pady=2,
                             command=self.conectar_wifi)
        btn_wifi.pack(side=tk.RIGHT, padx=(0, 4))
        self.ip_var = tk.StringVar(value=self.leer_ip_guardada())
        self.entry_ip = tk.Entry(header, textvariable=self.ip_var, width=15,
                                 font=(FONT_MONO, 9), bg="#08101E", fg=COLOR_TEXT_WHITE,
                                 insertbackground=COLOR_CYAN, highlightthickness=1,
                                 highlightbackground=COLOR_BORDER, justify="center")
        self.entry_ip.pack(side=tk.RIGHT, padx=(0, 4))
        self.entry_ip.bind("<Return>", lambda e: self.conectar_wifi())
        tk.Label(header, text="ESP32 IP ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.RIGHT, padx=(0, 4))

        # 1b. BARRA DE DISPARO (cruce por cero + rectificadores controlados)
        # El ángulo alfa es el mismo de la barra de arriba (solo RMMOC y RMOCC).
        barra_disp = tk.Frame(self.root, bg=COLOR_PANEL, highlightthickness=1,
                              highlightbackground=COLOR_BORDER)
        barra_disp.pack(side=tk.TOP, fill=tk.X, padx=15, pady=(0, 6))

        tk.Label(barra_disp, text="DISPARO ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(10, 4), pady=6)
        self.btn_disparo = tk.Button(barra_disp, text="[ ARMAR ]", font=(FONT_MONO, 8, "bold"),
                                     bg="#0E223D", fg=COLOR_AMBER, activebackground="#163866",
                                     activeforeground="#FFFFFF", bd=0, padx=10, pady=2,
                                     command=self.alternar_disparo)
        self.btn_disparo.pack(side=tk.LEFT, padx=(0, 12))

        # Captura manual del ADC (solo tiene efecto con la señal "7. ADC").
        self.btn_adc = tk.Button(barra_disp, text="[ CAPTURAR ADC ]", font=(FONT_MONO, 8, "bold"),
                                 bg="#0E223D", fg=COLOR_CYAN, activebackground="#163866",
                                 activeforeground="#FFFFFF", bd=0, padx=10, pady=2,
                                 command=self.capturar_adc)
        self.btn_adc.pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(barra_disp, text="ANCHO / T.MUERTO [µs] ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        self.ancho_var = tk.StringVar(value="50")
        tk.Entry(barra_disp, textvariable=self.ancho_var, width=6, font=(FONT_MONO, 9),
                 bg="#08101E", fg=COLOR_TEXT_WHITE, insertbackground=COLOR_CYAN,
                 highlightthickness=1, highlightbackground=COLOR_BORDER,
                 justify="center").pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(barra_disp, text="COMP [µs] ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        self.comp_var = tk.StringVar(value="100")
        tk.Entry(barra_disp, textvariable=self.comp_var, width=6, font=(FONT_MONO, 9),
                 bg="#08101E", fg=COLOR_TEXT_WHITE, insertbackground=COLOR_CYAN,
                 highlightthickness=1, highlightbackground=COLOR_BORDER,
                 justify="center").pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(barra_disp, text="n ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        self.n_var = tk.IntVar(value=N_INICIAL)
        self.spin_n = tk.Spinbox(barra_disp, from_=N_MIN, to=N_MAX, increment=10,
                                 textvariable=self.n_var, width=5, font=(FONT_MONO, 9),
                                 bg="#08101E", fg=COLOR_CYAN, insertbackground=COLOR_CYAN,
                                 highlightthickness=1, highlightbackground=COLOR_BORDER,
                                 command=self.on_cambio_n)
        self.spin_n.pack(side=tk.LEFT, padx=(0, 12))
        self.spin_n.bind("<Return>", self.on_cambio_n)
        self.spin_n.bind("<FocusOut>", self.on_cambio_n)

        tk.Label(barra_disp, text="R [Ω] ▶", font=(FONT_MONO, 9, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_PANEL).pack(side=tk.LEFT, padx=(0, 4))
        self.r_var = tk.StringVar(value=f"{R_CARGA_INICIAL:g}")
        ent_r = tk.Entry(barra_disp, textvariable=self.r_var, width=6, font=(FONT_MONO, 9),
                         bg="#08101E", fg=COLOR_TEXT_WHITE, insertbackground=COLOR_CYAN,
                         highlightthickness=1, highlightbackground=COLOR_BORDER,
                         justify="center")
        ent_r.pack(side=tk.LEFT, padx=(0, 12))
        ent_r.bind("<Return>", self.on_cambio_parametro)
        ent_r.bind("<KeyRelease>", self.on_cambio_parametro)

        self.lbl_zcd = tk.Label(barra_disp, text="ZCD: -- (sin datos)", font=(FONT_MONO, 9, "bold"),
                                fg=COLOR_TEXT_DIM, bg=COLOR_PANEL)
        self.lbl_zcd.pack(side=tk.RIGHT, padx=12)

        # 2. CONTENEDOR PRINCIPAL
        main_frame = tk.Frame(self.root, bg=COLOR_BG)
        main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=15, pady=8)

        left_frame = tk.Frame(main_frame, bg=COLOR_BG)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_frame = tk.Frame(main_frame, bg=COLOR_BG, width=380)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        right_frame.pack_propagate(False)

        self.crear_panel_graficos(left_frame)
        self.crear_panel_telemetria_tabla(right_frame)

    def crear_panel_graficos(self, parent):
        self.fig, (self.ax_ch_a, self.ax_ch_b) = plt.subplots(2, 1, figsize=(8.0, 5.5), facecolor=COLOR_BG)
        self.fig.subplots_adjust(left=0.08, right=0.92, top=0.93, bottom=0.09, hspace=0.35)
        self.ax_ch_a_i = self.ax_ch_a.twinx()

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def crear_panel_telemetria_tabla(self, parent):
        tk.Label(parent, text="[ TELEMETRY // TEÓRICO vs ESP32 ]", font=(FONT_MONO, 11, "bold"),
                 fg=COLOR_CYAN, bg=COLOR_BG).pack(anchor="w", pady=(0, 10))

        tabla_card = tk.Frame(parent, bg=COLOR_PANEL, highlightthickness=1, highlightbackground=COLOR_BORDER)
        tabla_card.pack(fill=tk.X, pady=4)

        headers = ["PARAM", "TEÓRICO", "ESP32 (N)", "ERROR %"]
        col_widths = [7, 11, 11, 8]
        
        header_row = tk.Frame(tabla_card, bg="#0E1A2E")
        header_row.pack(fill=tk.X, padx=2, pady=2)

        for i, (h, w) in enumerate(zip(headers, col_widths)):
            align = "w" if i == 0 else "e"
            lbl = tk.Label(header_row, text=h, font=(FONT_MONO, 8, "bold"),
                           fg=COLOR_CYAN, bg="#0E1A2E", width=w, anchor=align)
            lbl.pack(side=tk.LEFT, padx=3, pady=6)

        sep = tk.Frame(tabla_card, bg=COLOR_BORDER, height=1)
        sep.pack(fill=tk.X, padx=4)

        parametros = [
            "Vmax", "Vmin", "Vavg", "Vrms",
            "Imax", "Imin", "Iavg", "Irms",
            "S", "P", "Q", "D", "FP", "THDv", "THDi"
        ]
        for p in parametros:
            row = tk.Frame(tabla_card, bg=COLOR_PANEL)
            row.pack(fill=tk.X, padx=4, pady=6)

            lbl_nom = tk.Label(row, text=f"{p:<5}", font=(FONT_MONO, 9, "bold"),
                               fg=COLOR_TEXT_WHITE, bg=COLOR_PANEL, width=col_widths[0], anchor="w")
            lbl_nom.pack(side=tk.LEFT, padx=3)

            lbl_teo = tk.Label(row, text="0.00 V", font=(FONT_MONO, 9),
                               fg=COLOR_TEXT_DIM, bg=COLOR_PANEL, width=col_widths[1], anchor="e")
            lbl_teo.pack(side=tk.LEFT, padx=3)

            lbl_esp = tk.Label(row, text="0.00 V", font=(FONT_MONO, 9, "bold"),
                               fg=COLOR_AMBER, bg=COLOR_PANEL, width=col_widths[2], anchor="e")
            lbl_esp.pack(side=tk.LEFT, padx=3)

            lbl_err = tk.Label(row, text="0.00%", font=(FONT_MONO, 9, "bold"),
                               fg=COLOR_GREEN, bg=COLOR_PANEL, width=col_widths[3], anchor="e")
            lbl_err.pack(side=tk.LEFT, padx=3)

            self.telemetry_rows[p] = {
                "teo": lbl_teo,
                "esp": lbl_esp,
                "err": lbl_err
            }

        footer = tk.Frame(parent, bg=COLOR_BG)
        footer.pack(side=tk.BOTTOM, fill=tk.X, pady=8)
        tk.Label(footer, text="ENGINE: FOURIER DFT n = 120-720 PTS\nMCU: ESP32 DSP TELEMETRY",
                 font=(FONT_MONO, 8), fg=COLOR_TEXT_DIM, bg=COLOR_BG, justify=tk.LEFT).pack(anchor="w")

    # ==========================================================================
    # CÁLCULOS ANALÍTICOS Y SÍNTESIS DE ONDAS
    # ==========================================================================
    def calcular_teorico_analitico(self, tipo, vp, alfa_deg=0.0):
        alfa_rad = math.radians(alfa_deg)
        
        if tipo == "SENO":
            v_max = vp
            v_min = -vp
            v_avg = 0.0
            v_rms = vp / math.sqrt(2.0)

        elif tipo == "CUADRADA":
            v_max = vp
            v_min = -vp
            v_avg = 0.0
            v_rms = vp

        elif tipo == "CUASI_SENO":
            v_max = vp
            v_min = -vp
            v_avg = 0.0
            v_rms = vp * math.sqrt(2.0 / 3.0)

        elif "RMMOC" in tipo:
            v_max = vp
            v_min = 0.0
            v_avg = (vp / (2.0 * math.pi)) * (1.0 + math.cos(alfa_rad))  #[cite: 5, 14]
            termino_rad = (math.pi - alfa_rad + (math.sin(2.0 * alfa_rad) / 2.0)) / (4.0 * math.pi)
            v_rms = vp * math.sqrt(max(0.0, termino_rad))  #[cite: 5, 14]

        elif "RMOCC" in tipo:
            v_max = vp
            v_min = 0.0
            v_avg = (vp / math.pi) * (1.0 + math.cos(alfa_rad))  #[cite: 5, 6]
            termino_rad = (math.pi - alfa_rad + (math.sin(2.0 * alfa_rad) / 2.0)) / (2.0 * math.pi)
            v_rms = vp * math.sqrt(max(0.0, termino_rad))  #[cite: 5, 6]

        elif tipo == "RTMO":
            v_max = vp
            v_min = vp * 0.5  #[cite: 6]
            v_avg = vp * ((3.0 * math.sqrt(3.0)) / (2.0 * math.pi))  #[cite: 6]
            v_rms = vp * math.sqrt(0.5 + ((3.0 * math.sqrt(3.0)) / (8.0 * math.pi)))  #[cite: 6]

        else:
            v_max = v_min = v_avg = v_rms = 0.0

        return {"Vmax": v_max, "Vmin": v_min, "Vavg": v_avg, "Vrms": v_rms}

    def actualizar_estado_widgets(self):
        tipo = self.tipo_var.get()
        if "RMMOC" in tipo or "RMOCC" in tipo:
            self.lbl_alfa.config(fg=COLOR_AMBER)
            self.spin_alfa.config(state="normal")
        else:
            self.lbl_alfa.config(fg=COLOR_TEXT_DIM)
            self.spin_alfa.config(state="disabled")

    def generar_vectores_locales(self):
        tipo_str = self.tipo_var.get()
        
        try:
            vp = float(self.entry_vmax.get())
        except (ValueError, AttributeError):
            vp = 311.13

        try:
            N = int(self.spin_arm.get())
        except (ValueError, AttributeError):
            N = 5

        try:
            alfa_deg = float(self.spin_alfa.get())
            alfa_deg = max(0.0, min(alfa_deg, 180.0))
        except (ValueError, AttributeError):
            alfa_deg = 0.0

        alfa_rad = math.radians(alfa_deg)
        v_raw = []
        i_raw = []

        if "1. Seno" in tipo_str:
            self.nombre_corto = "SENO"
            for k in range(PUNTOS):
                wt = 2.0 * math.pi * (k / PUNTOS)
                v_raw.append(math.sin(wt))
                i_raw.append(0.65 * math.sin(wt - math.radians(35.0)))

        elif "2. Cuadrada" in tipo_str:
            self.nombre_corto = "CUADRADA"
            for k in range(PUNTOS):
                v = 1.0 if (k / PUNTOS) < 0.5 else -1.0
                i = (k / PUNTOS * 2 - 0.5) if (k / PUNTOS) < 0.5 else (1.5 - k / PUNTOS * 2)
                v_raw.append(v)
                i_raw.append(i * 0.5)

        elif "3. Cuasi Senoidal" in tipo_str:
            self.nombre_corto = "CUASI_SENO"
            for k in range(PUNTOS):
                deg = (360.0 * k) / PUNTOS
                v = 1.0 if (30.0 <= deg < 150.0) else (-1.0 if 210.0 <= deg < 330.0 else 0.0)
                v_raw.append(v)
                i_raw.append(v * 0.7)

        elif "4. RMMOC" in tipo_str:
            self.nombre_corto = f"RMMOC a{int(alfa_deg)}"
            for k in range(PUNTOS):
                wt = 2.0 * math.pi * (k / PUNTOS)
                # Conducción exclusiva en semiciclo positivo entre alfa y pi
                if alfa_rad <= wt < math.pi:
                    v = math.sin(wt)
                    i = max(0.0, 0.65 * math.sin(wt - math.radians(20.0)))
                else:
                    v = 0.0
                    i = 0.0
                v_raw.append(v)
                i_raw.append(i)

        elif "5. RMOCC" in tipo_str:
            self.nombre_corto = f"RMOCC a{int(alfa_deg)}"
            for k in range(PUNTOS):
                wt = 2.0 * math.pi * (k / PUNTOS)
                # Semiciclo positivo: alfa <= wt < pi
                if 0.0 <= wt < math.pi:
                    v = math.sin(wt) if wt >= alfa_rad else 0.0
                # Semiciclo negativo: (pi + alfa) <= wt < 2*pi
                else:
                    v = abs(math.sin(wt)) if wt >= (math.pi + alfa_rad) else 0.0
                v_raw.append(v)
                i_raw.append(v * 0.5)

        elif "6. RTMO" in tipo_str:
            self.nombre_corto = "RTMO"
            for k in range(PUNTOS):
                wt = 2.0 * math.pi * (k / PUNTOS)
                v_r = math.sin(wt)
                v_s = math.sin(wt - (2.0 * math.pi / 3.0))
                v_t = math.sin(wt - (4.0 * math.pi / 3.0))
                v = max(v_r, v_s, v_t)  #[cite: 6]
                v_raw.append(v)
                i_raw.append(v * 0.6)

        elif "7. ADC" in tipo_str:
            # Señal REAL: son las muestras que midió el ESP32 con su ADC (última captura).
            # Si todavía no hay una captura de n muestras, se muestra una línea en cero.
            self.nombre_corto = "ADC"
            if len(self.adc_v) == PUNTOS and len(self.adc_i) == PUNTOS:
                v_raw = list(self.adc_v)
                escala_i = vp / self.leer_resistencia()    # [A] a fondo de escala
                i_raw = [x / escala_i for x in self.adc_i]
            else:
                v_raw = [0.0] * PUNTOS
                i_raw = [0.0] * PUNTOS

        if self.nombre_corto != "ADC":
            # La consigna fija una carga puramente resistiva (R = 100 ohm): i(t) = v(t)/R,
            # con la misma forma que v. Se descartan las formas de corriente de demostración.
            i_raw = list(v_raw)

        self.v_orig = [val * vp for val in v_raw]
        # Carga resistiva: i = v / R  (v = val * vp  =>  i = val * vp / R)
        self.i_orig = [val * (vp / self.leer_resistencia()) for val in i_raw]

        # Actualización de métricas analíticas ideales
        self.metricas_teoricas = self.calcular_teorico_analitico(self.nombre_corto, vp, alfa_deg)
        if self.nombre_corto == "ADC":
            # Una señal medida no tiene fórmula teórica: la columna "TEÓRICO" pasa a ser el
            # cálculo de referencia de la PC con las MISMAS muestras que midió el ESP32.
            self.metricas_teoricas.update({
                "Vmax": max(self.v_orig), "Vmin": min(self.v_orig),
                "Vavg": sum(self.v_orig) / PUNTOS,
                "Vrms": math.sqrt(sum(x * x for x in self.v_orig) / PUNTOS),
            })
        i_max = max(self.i_orig)
        i_min = min(self.i_orig)
        i_avg = sum(self.i_orig) / PUNTOS
        i_rms = math.sqrt(sum(valor * valor for valor in self.i_orig) / PUNTOS)
        v_rms = math.sqrt(sum(valor * valor for valor in self.v_orig) / PUNTOS)
        potencia_s = v_rms * i_rms
        potencia_p = sum(v * i for v, i in zip(self.v_orig, self.i_orig)) / PUNTOS
        potencia_q = 0.0
        # THD (Hart ec. 6.8): sqrt(sum_{n>=2}(an^2+bn^2)) / sqrt(a1^2+b1^2). La relacion
        # entre valores eficaces de armonico = relacion entre amplitudes (el 1/2 se cancela).
        suma_v_arm = 0.0   # sum_{n>=2} (an^2+bn^2) de la tension
        suma_i_arm = 0.0   # idem corriente
        fund_v = 0.0       # a1^2+b1^2 tension
        fund_i = 0.0       # a1^2+b1^2 corriente
        for n in range(1, min(N, PUNTOS // 2) + 1):
            angulos = [2.0 * math.pi * n * k / PUNTOS for k in range(PUNTOS)]
            coef_v_cos = (2.0 / PUNTOS) * sum(v * math.cos(angulo) for v, angulo in zip(self.v_orig, angulos))
            coef_v_sin = (2.0 / PUNTOS) * sum(v * math.sin(angulo) for v, angulo in zip(self.v_orig, angulos))
            coef_i_cos = (2.0 / PUNTOS) * sum(i * math.cos(angulo) for i, angulo in zip(self.i_orig, angulos))
            coef_i_sin = (2.0 / PUNTOS) * sum(i * math.sin(angulo) for i, angulo in zip(self.i_orig, angulos))
            potencia_q += (coef_v_sin * coef_i_cos - coef_v_cos * coef_i_sin) / 2.0
            if n == 1:
                fund_v = coef_v_cos ** 2 + coef_v_sin ** 2
                fund_i = coef_i_cos ** 2 + coef_i_sin ** 2
            else:
                suma_v_arm += coef_v_cos ** 2 + coef_v_sin ** 2
                suma_i_arm += coef_i_cos ** 2 + coef_i_sin ** 2
        # Si la fundamental es ~0 el THD no esta definido -> se informa 0
        thd_v = 100.0 * math.sqrt(suma_v_arm / fund_v) if fund_v > 1e-12 else 0.0
        thd_i = 100.0 * math.sqrt(suma_i_arm / fund_i) if fund_i > 1e-12 else 0.0
        potencia_d = math.sqrt(max(0.0, potencia_s ** 2 - potencia_p ** 2 - potencia_q ** 2))
        self.metricas_teoricas.update({
            "Imax": i_max, "Imin": i_min, "Iavg": i_avg, "Irms": i_rms,
            "S": potencia_s, "P": potencia_p, "Q": potencia_q, "D": potencia_d,
            "FP": potencia_p / potencia_s if potencia_s > 0.0 else 0.0,
            "THDv": thd_v, "THDi": thd_i,
        })
        unidades = {
            "Vmax": "V", "Vmin": "V", "Vavg": "V", "Vrms": "V",
            "Imax": "A", "Imin": "A", "Iavg": "A", "Irms": "A",
            "S": "VA", "P": "W", "Q": "var", "D": "VA", "FP": "",
            "THDv": "%", "THDi": "%"
        }
        for p, vals in self.telemetry_rows.items():
            vals["teo"].config(text=f"{self.metricas_teoricas[p]:>7.2f} {unidades[p]}")

        # Síntesis local de Fourier para previsualización inmediata en Canal B
        a0 = sum(v_raw) / PUNTOS
        an = []
        bn = []
        for n in range(1, N + 1):
            sum_cos = sum(v_raw[k] * math.cos(2.0 * math.pi * n * k / PUNTOS) for k in range(PUNTOS))
            sum_sin = sum(v_raw[k] * math.sin(2.0 * math.pi * n * k / PUNTOS) for k in range(PUNTOS))
            an.append((2.0 / PUNTOS) * sum_cos)
            bn.append((2.0 / PUNTOS) * sum_sin)

        self.v_recon = []
        for k in range(PUNTOS):
            val = a0
            for n in range(1, N + 1):
                ang = 2.0 * math.pi * n * k / PUNTOS
                val += an[n - 1] * math.cos(ang) + bn[n - 1] * math.sin(ang)
            self.v_recon.append(val * vp)

    # ==========================================================================
    # ACTUALIZACIÓN DE GRÁFICOS
    # ==========================================================================
    def actualizar_graficos(self):
        if not self.root.winfo_exists():
            return

        try:
            N = int(self.spin_arm.get())
        except (ValueError, AttributeError):
            N = 5

        self.ax_ch_a.clear()
        self.ax_ch_a_i.clear()
        self.ax_ch_b.clear()

        # CANAL A (Dominio Temporal)
        self.ax_ch_a.set_facecolor(COLOR_BG)
        self.ax_ch_a.plot(self.deg_x, self.v_orig, color=COLOR_CYAN, linewidth=2.0, label="v[k] Voltage")
        self.ax_ch_a_i.plot(self.deg_x, self.i_orig, color=COLOR_MAGENTA, linewidth=1.8, linestyle="--", label="i[k] Current")

        self.ax_ch_a.set_title(f"CHANNEL A // WAVEFORM: {self.nombre_corto} (TIME DOMAIN)",
                               fontname=FONT_MONO, fontsize=9, color=COLOR_TEXT_WHITE, loc="left", pad=8)
        self.ax_ch_a.set_ylabel("VOLTAGE [V]", fontname=FONT_MONO, fontsize=8, color=COLOR_CYAN)
        self.ax_ch_a_i.set_ylabel("CURRENT [A]", fontname=FONT_MONO, fontsize=8, color=COLOR_MAGENTA)

        # CANAL B (Correlación Fourier)
        self.ax_ch_b.set_facecolor(COLOR_BG)
        self.ax_ch_b.plot(self.deg_x, self.v_orig, color="#213550", linewidth=1.2, label="Señal original")
        self.ax_ch_b.plot(self.deg_x, self.v_recon, color=COLOR_CYAN, linewidth=1.8, label=f"Reconstruida // X={N}")

        self.ax_ch_b.set_title(f"CHANNEL B // FOURIER CORRELATION // {N} HARMONICS",
                               fontname=FONT_MONO, fontsize=9, color=COLOR_TEXT_WHITE, loc="left", pad=8)
        self.ax_ch_b.set_xlabel("SAMPLE INDEX k [ELECTRICAL DEGREES °]", fontname=FONT_MONO, fontsize=8, color=COLOR_TEXT_DIM)
        self.ax_ch_b.set_ylabel("VOLTAGE [V]", fontname=FONT_MONO, fontsize=8, color=COLOR_CYAN)

        for ax in [self.ax_ch_a, self.ax_ch_b]:
            ax.set_xlim(0, 360)
            ax.set_xticks(range(0, 361, 30))
            ax.grid(True, linestyle=":", linewidth=0.6, color=COLOR_GRID)
            ax.tick_params(colors=COLOR_TEXT_DIM, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color(COLOR_BORDER)
            
            leg = ax.legend(facecolor=COLOR_CARD, edgecolor=COLOR_BORDER, fontsize=7, loc="upper right")
            for text in leg.get_texts():
                text.set_color(COLOR_TEXT_WHITE)
                text.set_fontname(FONT_MONO)

        self.ax_ch_a_i.tick_params(colors=COLOR_MAGENTA, labelsize=7)
        for spine in self.ax_ch_a_i.spines.values():
            spine.set_color(COLOR_BORDER)

        try:
            self.canvas.draw_idle()
        except Exception:
            pass

    # ==========================================================================
    # COMUNICACIÓN SERIE Y TELEMETRÍA
    # ==========================================================================
    def on_cambio_parametro(self, event=None):
        if self.bloqueo_tx_echo:
            return
        
        # Cancela envíos pendientes mientras se sigue escribiendo
        if hasattr(self, '_debounce_timer') and self._debounce_timer is not None:
            self.root.after_cancel(self._debounce_timer)
            
        # Espera 250 ms tras la última tecla/clic antes de calcular y transmitir
        self._debounce_timer = self.root.after(250, self._procesar_cambio_parametro)

    def _procesar_cambio_parametro(self):
        self._debounce_timer = None
        self.actualizar_estado_widgets()
        self.generar_vectores_locales()
        self.actualizar_graficos()
        self.transmitir_al_esp32()
        self.revisar_disparo_tras_cambio()

    def auto_conectar_serial(self):
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        for p in puertos:
            try:
                ser = serial.Serial(p, 115200, timeout=0.5)
                time.sleep(1.2)
                ser.reset_input_buffer()
                self.serial_port = ser
                self.lbl_status.config(text=f"● ESP32 ONLINE // {p}", fg=COLOR_CYAN)
                self.iniciar_lectura()
                self.transmitir_al_esp32()
                return
            except Exception:
                continue
        # Sin cable: si hay una IP guardada, se intenta por WiFi.
        if self.ip_var.get().strip() and self.conectar_wifi(silencioso=True):
            return
        self.lbl_status.config(text="● LOCAL MODE // NO PORT", fg="#E76F51")

    def leer_ip_guardada(self):
        try:
            with open(ARCHIVO_IP, "r", encoding="utf-8") as f:
                return f.read().strip()
        except OSError:
            return ""

    def iniciar_lectura(self):
        """Lanza un hilo de lectura nuevo; el anterior (si había) termina solo."""
        self._lectura_id += 1
        threading.Thread(target=self.hilo_lectura_serie, args=(self._lectura_id,),
                         daemon=True).start()

    def conectar_wifi(self, silencioso=False):
        ip = self.ip_var.get().strip()
        if not ip:
            self.lbl_status.config(text="● WIFI: FALTA IP", fg="#E76F51")
            return False
        self.lbl_status.config(text=f"● CONECTANDO WIFI // {ip}...", fg=COLOR_AMBER)
        self.root.update_idletasks()
        try:
            con = ConexionTCP(ip)
        except OSError as e:
            if not silencioso:
                print(f"[WIFI ERR] {e}")
            self.lbl_status.config(text=f"● WIFI SIN RESPUESTA // {ip}", fg="#E76F51")
            return False
        # Se cierra la conexión anterior (USB o WiFi) y se reemplaza por la nueva.
        anterior = self.serial_port
        self.serial_port = con
        if anterior is not None:
            try:
                anterior.close()
            except Exception:
                pass
        try:
            with open(ARCHIVO_IP, "w", encoding="utf-8") as f:
                f.write(ip)
        except OSError:
            pass
        self.lbl_status.config(text=f"● ESP32 ONLINE // WIFI {ip}", fg=COLOR_CYAN)
        self.iniciar_lectura()
        self.transmitir_al_esp32()
        return True

    # ----------------------------------------------------------------------
    # DISPARO (comandos CTRL al ESP32)
    # ----------------------------------------------------------------------
    def leer_resistencia(self):
        """R de la carga [ohm]. Si el campo es inválido o <= 0 se usa el valor inicial."""
        try:
            r = float(self.r_var.get())
            return r if r > 0 else R_CARGA_INICIAL
        except (ValueError, AttributeError):
            return R_CARGA_INICIAL

    def on_cambio_n(self, event=None):
        """Cambió n (muestras por vector, 120..720): reconstruye todo con el nuevo n."""
        global PUNTOS
        try:
            n = int(float(self.spin_n.get()))
        except (ValueError, AttributeError):
            n = N_INICIAL
        n = max(N_MIN, min(N_MAX, n))
        self.n_var.set(n)
        if n == PUNTOS:
            return
        PUNTOS = n
        self.deg_x = [360.0 * (k / PUNTOS) for k in range(PUNTOS)]
        self.v_orig = [0.0] * PUNTOS
        self.i_orig = [0.0] * PUNTOS
        self.v_recon = [0.0] * PUNTOS
        # Con n muestras por período solo hay n/2 armónicos válidos (Nyquist).
        n_arm_max = min(300, PUNTOS // 2)
        self.spin_arm.config(to=n_arm_max)
        if self.armonicos_var.get() > n_arm_max:
            self.armonicos_var.set(n_arm_max)
        self.on_cambio_parametro()

    def enviar_texto(self, texto):
        """Manda una línea de texto al ESP32 (USB o WiFi). Devuelve True si se envió."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(texto.encode('utf-8'))
                self.serial_port.flush()
                return True
            except Exception as e:
                print(f"[TX ERR] {e}")
        return False

    def modo_disparo_actual(self):
        """RMMOC / RMOCC (rectificador) o INVSQ / INVQS (inversor) según la señal elegida; None si no aplica."""
        tipo = self.tipo_var.get()
        if "RMMOC" in tipo:
            return "RMMOC"
        if "RMOCC" in tipo:
            return "RMOCC"
        if "Cuadrada" in tipo:
            return "INVSQ"          # inversor: onda cuadrada (puente H)
        if "Cuasi" in tipo:
            return "INVQS"          # inversor: onda cuasicuadrada (puente H)
        return None

    def enviar_comando_disparo(self):
        modo = self.modo_disparo_actual()
        try:
            alfa = float(self.alfa_var.get())
            ancho = int(float(self.ancho_var.get()))
            comp = int(float(self.comp_var.get()))
        except ValueError:
            self.lbl_zcd.config(text="DISPARO: VALORES INVÁLIDOS", fg="#E76F51")
            return False
        return self.enviar_texto(f"CTRL,{modo},{alfa:.2f},{ancho},{comp}\n")

    def alternar_disparo(self):
        if self.disparo_armado:
            self.enviar_texto("CTRL,OFF\n")
            self.disparo_armado = False
            self.btn_disparo.config(text="[ ARMAR ]", fg=COLOR_AMBER)
            return
        if self.modo_disparo_actual() is None:
            self.lbl_zcd.config(text="ELEGÍ RMMOC, RMOCC, CUADRADA O CUASI", fg="#E76F51")
            return
        if not (self.serial_port and self.serial_port.is_open):
            self.lbl_zcd.config(text="SIN ENLACE CON EL ESP32", fg="#E76F51")
            return
        if self.enviar_comando_disparo():
            self.disparo_armado = True
            self.btn_disparo.config(text="[ APAGAR ]", fg=COLOR_MAGENTA)

    def revisar_disparo_tras_cambio(self):
        """Si el disparo está armado y cambió la señal o alfa: reenvía o apaga."""
        if not self.disparo_armado:
            return
        if self.modo_disparo_actual() is None:
            self.enviar_texto("CTRL,OFF\n")
            self.disparo_armado = False
            self.btn_disparo.config(text="[ ARMAR ]", fg=COLOR_AMBER)
        else:
            self.enviar_comando_disparo()

    def actualizar_zcd_ui(self, partes):
        # ZCD_ESTADO,presente,periodo_us,frec_hz,modo,alfa,ancho,comp
        try:
            presente = partes[1] == "1"
            periodo = int(partes[2])
            frec = float(partes[3])
        except (IndexError, ValueError):
            return
        if presente:
            self.lbl_zcd.config(text=f"ZCD: OK  {frec:.2f} Hz  (T = {periodo} µs)", fg=COLOR_GREEN)
        else:
            self.lbl_zcd.config(text="ZCD: SIN CRUCE POR CERO", fg="#E76F51")

    def transmitir_al_esp32(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                vp = float(self.entry_vmax.get())
            except (ValueError, AttributeError):
                vp = 311.13
            armonicos = max(1, min(300, PUNTOS // 2, int(self.armonicos_var.get())))
            if "7. ADC" in self.tipo_var.get():
                # No se mandan vectores: se le pide al ESP32 que mida (ADC) y devuelva
                # las muestras (ADC_DATA) y las métricas (DATA_METRICAS).
                self.enviar_texto(f"ADC,{vp:.2f},{self.leer_resistencia():g},{armonicos},{PUNTOS}\n")
                return
            freq = self.freq_var.get()
            muestras_norm = [f"{v / vp:.4f}" for v in self.v_orig]
            muestras_corriente = [f"{i:.4f}" for i in self.i_orig]
            trama = (
                f"{self.nombre_corto},{vp:.2f},{freq},{armonicos},{PUNTOS},"
                f"{','.join(muestras_norm)},{','.join(muestras_corriente)}\n"
            )
            try:
                self.serial_port.write(trama.encode('utf-8'))
                self.serial_port.flush()
            except Exception as e:
                print(f"[TX ERR] {e}")

    def hilo_lectura_serie(self, mi_id=None):
        if mi_id is None:
            mi_id = self._lectura_id
        while self.running_thread and mi_id == self._lectura_id:
            try:
                if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting:
                    linea = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if linea.startswith("DATA_METRICAS"):
                        tokens = linea.split(",")
                        if len(tokens) == 17:  # DATA_METRICAS + N + 13 metricas + THDv + THDi
                            n_act = int(tokens[1])
                            metricas = [float(valor) for valor in tokens[2:]]

                            if self.running_thread and self.root.winfo_exists():
                                self.root.after(0, self.actualizar_telemetria_ui, n_act, metricas)
                    elif linea.startswith("ADC_DATA"):
                        # ADC_DATA,n,v[0..n-1],i[0..n-1]
                        tokens = linea.split(",")
                        n_rx = int(tokens[1])
                        if len(tokens) == 2 + 2 * n_rx and self.running_thread and self.root.winfo_exists():
                            datos = [float(x) for x in tokens[2:]]
                            self.root.after(0, self.aplicar_captura_adc, datos[:n_rx], datos[n_rx:])
                    elif linea.startswith("ADC_SIN_ZCD"):
                        if self.running_thread and self.root.winfo_exists():
                            self.root.after(0, lambda: self.lbl_zcd.config(
                                text="ADC: SIN ZCD (conectá el generador a GPIO4)", fg="#E76F51"))
                    elif linea.startswith("ADC_INFO"):
                        print(linea)   # n, período y cuántas muestras llegaron tarde
                    elif linea.startswith("ZCD_ESTADO"):
                        if self.running_thread and self.root.winfo_exists():
                            self.root.after(0, self.actualizar_zcd_ui, linea.split(","))
                time.sleep(0.02)
            except Exception:
                break

    def capturar_adc(self):
        """Botón CAPTURAR ADC: pide una captura nueva (hace falta ZCD en el ESP32)."""
        if "7. ADC" not in self.tipo_var.get():
            self.lbl_zcd.config(text="ELEGÍ LA SEÑAL 7. ADC", fg="#E76F51")
            return
        if not (self.serial_port and self.serial_port.is_open):
            self.lbl_zcd.config(text="SIN ENLACE CON EL ESP32", fg="#E76F51")
            return
        self.transmitir_al_esp32()

    def aplicar_captura_adc(self, v_norm, i_amp):
        """Llegó una captura del ADC: se guarda y se redibuja (hilo principal de Tk)."""
        self.adc_v = v_norm
        self.adc_i = i_amp
        if "7. ADC" not in self.tipo_var.get() or len(v_norm) != PUNTOS:
            return
        self.generar_vectores_locales()
        self.actualizar_graficos()

    def actualizar_telemetria_ui(self, n_act, metricas):
        try:
            if not self.root.winfo_exists() or not self.running_thread:
                return

            try:
                vp = float(self.entry_vmax.get())
            except (ValueError, AttributeError):
                vp = 311.13

            nombres_metricas = [
                "Vmax", "Vmin", "Vavg", "Vrms",
                "Imax", "Imin", "Iavg", "Irms",
                "S", "P", "Q", "D", "FP", "THDv", "THDi"
            ]
            esp_dict = dict(zip(nombres_metricas, metricas))

            unidades = {
                "Vmax": "V", "Vmin": "V", "Vavg": "V", "Vrms": "V",
                "Imax": "A", "Imin": "A", "Iavg": "A", "Irms": "A",
                "S": "VA", "P": "W", "Q": "var", "D": "VA", "FP": "",
            "THDv": "%", "THDi": "%"
            }
            for p in nombres_metricas:
                teo_val = self.metricas_teoricas[p]
                esp_val = esp_dict[p]

                unidad = unidades[p]
                self.telemetry_rows[p]["esp"].config(text=f"{esp_val:>7.2f} {unidad}")

                # Normalización a fondo de escala (Vp) si el valor teórico es ~0 V
                if abs(teo_val) > 0.05:
                    err_val = abs((esp_val - teo_val) / teo_val) * 100.0
                elif p in ["FP"]:
                    err_val = abs(esp_val - teo_val) * 100.0
                elif p in ["THDv", "THDi"]:
                    err_val = abs(esp_val - teo_val)  # ya estan en %: error en puntos porcentuales
                else:
                    err_val = (abs(esp_val - teo_val) / vp) * 100.0

                txt_err = f"{err_val:>5.2f}%"
                color_err = COLOR_GREEN if err_val < 1.0 else (COLOR_AMBER if err_val < 5.0 else COLOR_MAGENTA)
                self.telemetry_rows[p]["err"].config(text=txt_err, fg=color_err)
            
            # Sincronización si el encoder físico modificó N en el ESP32
            if self.armonicos_var.get() != n_act:
                self.bloqueo_tx_echo = True
                self.armonicos_var.set(n_act)
                self.generar_vectores_locales()
                self.actualizar_graficos()
                self.bloqueo_tx_echo = False

        except (tk.TclError, RuntimeError):
            pass

    def on_closing(self):
        if self.disparo_armado:
            self.enviar_texto("CTRL,OFF\n")   # nunca dejar el disparo activo al salir
        self.running_thread = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = OsciloscopioApp(root)
    root.mainloop()