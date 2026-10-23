#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>

#define MAX_PUNTOS 720         // máximo de muestras por vector (consigna: n entre 120 y 720)
#define MIN_PUNTOS 120         // mínimo de muestras por vector
#define N_INICIAL 360          // n inicial de la consigna
#define MAX_ARMONICOS 300
#define TIMEOUT_AUTO_MS 450
#define TCP_PUERTO 5000        // puerto TCP donde la app Python se conecta por WiFi

struct FourierAnalyzer {
    char tipo[16];
    char freq[12];
    float vmax;
    int armonicos_sel;
    int armonicos_act;
    int n_muestras;      // n actual: muestras por vector (120..720). Lo fija configurar_n()
    int n_arm_max;       // máximo de armónicos válidos = min(MAX_ARMONICOS, n/2) (Nyquist)

    // Muestras crudas de entrada (1 período)
    float vector_in[MAX_PUNTOS];
    float corriente_in[MAX_PUNTOS];

    // Coeficientes de Fourier
    float a0;
    float an[MAX_ARMONICOS + 1];
    float bn[MAX_ARMONICOS + 1];
    float a0_i;
    float an_i[MAX_ARMONICOS + 1];
    float bn_i[MAX_ARMONICOS + 1];

    // Métricas eléctricas calculadas
    float v_max_calc;
    float v_min_calc;
    float v_avg_calc;
    float v_rms_calc;
    float i_max_calc;
    float i_min_calc;
    float i_avg_calc;
    float i_rms_calc;
    float potencia_s;
    float potencia_p;
    float potencia_q;
    float potencia_d;
    float factor_potencia;
    float thd_v;   // THD de tensión [%], armónicos 2..N respecto de la fundamental
    float thd_i;   // THD de corriente [%]

    bool datos_listos;
};

// ---------------------------------------------------------------------------
// Estado compartido entre módulos (cada variable se DEFINE en un solo .cpp)
// ---------------------------------------------------------------------------
extern FourierAnalyzer fa;                    // definido en fourier.cpp
extern volatile bool actualizar_pantalla;     // definido en display.cpp
extern volatile bool cambio_pendiente;        // definido en display.cpp

// ---------------------------------------------------------------------------
// fourier.cpp: análisis de Fourier y métricas
// ---------------------------------------------------------------------------
// Tablas de cos y sin de 2*pi*j/n, j = 0..n-1, con n = fa.n_muestras (se recalculan en configurar_n()).
// Como los ángulos del análisis son siempre 2*pi*h*k/n, cos(h*theta_k) = tabla_cos[(h*k) % n]:
// se evita llamar millones de veces a cosf/sinf (por software en el ESP32, no tiene FPU trigonométrica).
extern float tabla_cos[MAX_PUNTOS];
extern float tabla_sin[MAX_PUNTOS];
void configurar_n(int n);   // fija n (120..720), recalcula tablas y el máximo de armónicos

void analizar_fourier();
void calcular_metricas_y_reportar(int n_arm);

// ---------------------------------------------------------------------------
// display.cpp: OLED + encoder
// ---------------------------------------------------------------------------
void init_hardware();
void display_suspender_encoder();   // detach de la ISR (durante OTA)
void display_reanudar_encoder();    // vuelve a conectar la ISR
void verificar_auto_reconstruccion();
void render_pantalla();

// ---------------------------------------------------------------------------
// comms.cpp: protocolo con la app Python (USB serie y WiFi TCP)
// ---------------------------------------------------------------------------
void recibir_datos_serial();
void atender_tcp();                      // recepción por WiFi (TCP), llamar desde loop()
void enviar_linea(const char* linea);   // responde por USB y por WiFi (si hay cliente)

// ---------------------------------------------------------------------------
// ota.cpp: WiFi + OTA
// ---------------------------------------------------------------------------
void init_wifi_ota();   // inicia WiFi y la tarea de OTA (core 0), no bloquea
bool ota_en_curso();    // true mientras se está actualizando el firmware

// ---------------------------------------------------------------------------
// zcd.cpp: detector de cruce por cero
// ---------------------------------------------------------------------------
void init_zcd();
void zcd_suspender();            // detach de la ISR (durante OTA)
void zcd_reanudar();
bool zcd_presente();             // hay red/señal con período válido y reciente
uint32_t zcd_periodo_us();       // último período medido [us]
float zcd_frecuencia_hz();
uint32_t zcd_ultimo_cruce_us();  // instante (micros) del último cruce aceptado

// ---------------------------------------------------------------------------
// adc.cpp: captura de V (GPIO34) e I (GPIO35) sincronizada con el ZCD
// ---------------------------------------------------------------------------
void init_adc();
void adc_pedir(float r_ohm);     // pide una captura (comms.cpp, al recibir "ADC,...")
void adc_actualizar();           // desde loop(): espera el cruce, muestrea y reporta

// ---------------------------------------------------------------------------
// control.cpp: disparo de rectificadores (gates de SCR) sincronizado con el ZCD
// ---------------------------------------------------------------------------
void init_control();
void control_modo_seguro();               // apaga todas las salidas de potencia
void control_on_cruce();                  // la llama la ISR del ZCD en cada cruce positivo
void control_actualizar();                // desde loop(): avisos y ZCD_ESTADO cada 1 s
void control_procesar_comando(char* linea);   // "CTRL,..." (desde comms.cpp)

#endif
