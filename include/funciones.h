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
    
    bool datos_listos;
};

void init_hardware();
void init_wifi_ota();
void atender_wifi_ota();
void recibir_datos_serial();
void verificar_auto_reconstruccion();
void render_pantalla();

#endif