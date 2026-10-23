#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>

#define MAX_PUNTOS 1000
#define MAX_ARMONICOS 300
#define TIMEOUT_AUTO_MS 450

struct FourierAnalyzer {
    char tipo[16];
    char freq[12];
    float vmax;
    int armonicos_sel;
    int armonicos_act;

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
// Tablas de cos y sin de 2*pi*j/MAX_PUNTOS, j = 0..MAX_PUNTOS-1 (llenar UNA vez con init_tablas_trig()).
// Como los ángulos del análisis son siempre 2*pi*n*k/MAX_PUNTOS, cos(n*theta_k) = tabla_cos[(n*k) % MAX_PUNTOS]:
// se evita llamar millones de veces a cosf/sinf (por software en el ESP32, no tiene FPU trigonométrica).
extern float tabla_cos[MAX_PUNTOS];
extern float tabla_sin[MAX_PUNTOS];
void init_tablas_trig();

void analizar_fourier();
void calcular_metricas_y_reportar(int n_arm);

// ---------------------------------------------------------------------------
// display.cpp: OLED + encoder
// ---------------------------------------------------------------------------
void init_hardware();
void verificar_auto_reconstruccion();
void render_pantalla();

// ---------------------------------------------------------------------------
// comms.cpp: protocolo serie con la app Python
// ---------------------------------------------------------------------------
void recibir_datos_serial();

// ---------------------------------------------------------------------------
// ota.cpp: WiFi + OTA
// ---------------------------------------------------------------------------
void init_wifi_ota();
void atender_wifi_ota();

#endif
