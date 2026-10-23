// 2. Síntesis y cálculo de métricas eléctricas
/* Segun los armonicos:
V_avg: Es directamente la componente continua a_0 * V_max
V_rms (Teorema de Parseval / Ley de Ortogonalidad): 
    Suma cuadrática de los valores eficaces individuales de cada armónico sintetizado:
        $$V_{rms}(N) = V_{max} * \sqrt{a_0^2 + \sum_{n=1}^{N} \frac{a_n^2 + b_n^2}{2}}$$
    Segun sus formulas y funciones:
Señal                            | Vavg Teórico                 | Vrms Teórico                                              | Vmax Teórico | Vmin Teórico
---------------------------------+------------------------------+-----------------------------------------------------------+--------------+-------------
Senoidal Pura                    | 0 V                          | Vp / sqrt(2) ≈ 0.7071 Vp                                  | +Vp          | -Vp
Cuadrada Simétrica               | 0 V                          | Vp = 1.0000 Vp                                            | +Vp          | -Vp
Cuasi Senoidal (120°)            | 0 V                          | Vp * sqrt(2/3) ≈ 0.8165 Vp                                | +Vp          | -Vp
RMMO (Media Onda)                | (Vp / pi) ≈ 0.3183 Vp        | Vp / 2 = 0.5000 Vp                                        | +Vp          | 0 V
RMOOC (Onda Completa, alpha=0)   | (2 * Vp / pi) ≈ 0.6366 Vp    | Vp / sqrt(2) ≈ 0.7071 Vp                                  | +Vp          | 0 V
RMOCC (Onda Completa Controlado) | (Vp / pi) * (1 + cos(alpha)) | (Vp / sqrt(2)) * sqrt((pi - alpha + sin(2*alpha)/2) / pi) | +Vp          | 0 V
*/
// fourier.cpp — Análisis de Fourier discreto y cálculo de métricas eléctricas.
// (Movido sin cambios de lógica desde funciones.cpp)
#include "funciones.h"
#include <math.h>

FourierAnalyzer fa = {};

float tabla_cos[MAX_PUNTOS];
float tabla_sin[MAX_PUNTOS];

// Llena las tablas. Se calcula en double para que cada entrada quede redondeada
// correctamente a float (más preciso que acumular ángulos en float como antes).
void init_tablas_trig() {
    for (int j = 0; j < MAX_PUNTOS; j++) {
        const double ang = 2.0 * M_PI * (double)j / (double)MAX_PUNTOS;
        tabla_cos[j] = (float)cos(ang);
        tabla_sin[j] = (float)sin(ang);
    }
}

// 1. Correlación Ortogonal de Fourier Discreta
static void calcular_coeficientes(const float* vector, float& a0, float* an, float* bn) {
    const int M = MAX_PUNTOS;
    const float factor_dos_m = 2.0f / (float)M;

    float suma_dc = 0.0f;
    for (int k = 0; k < M; k++) {
        suma_dc += vector[k];
    }
    a0 = suma_dc / (float)M;

    for (int n = 1; n <= MAX_ARMONICOS; n++) {
        float suma_cos = 0.0f;
        float suma_sin = 0.0f;
        // idx = (n*k) mod M, calculado de forma incremental: en cada paso se suma n
        // y, si se pasó de M, se resta M (vale porque n <= MAX_ARMONICOS < M). Evita la división.
        int idx = 0;
        for (int k = 0; k < M; k++) {
            suma_cos += vector[k] * tabla_cos[idx];
            suma_sin += vector[k] * tabla_sin[idx];
            idx += n;
            if (idx >= M) idx -= M;
        }

        an[n] = factor_dos_m * suma_cos;
        bn[n] = factor_dos_m * suma_sin;
    }
}

void analizar_fourier() {
    calcular_coeficientes(fa.vector_in, fa.a0, fa.an, fa.bn);
    calcular_coeficientes(fa.corriente_in, fa.a0_i, fa.an_i, fa.bn_i);
}

// 2. Síntesis y cálculo de métricas
void calcular_metricas_y_reportar(int n_arm) {
    const int M = MAX_PUNTOS;
    float v_max = -9999.0f;
    float v_min = 9999.0f;
    float i_max = -9999.0f;
    float i_min = 9999.0f;
    float suma_potencia_activa = 0.0f;
    float suma_potencia_reactiva = 0.0f;

    for (int k = 0; k < M; k++) {
        float val_norm = fa.a0;
        float val_corriente = fa.a0_i;

        // Mismo truco que en calcular_coeficientes: idx = (n*k) mod M, incremental.
        // El coseno y el seno se leen UNA vez y se usan para tensión y corriente.
        int idx = 0;
        for (int n = 1; n <= n_arm; n++) {
            idx += k;
            if (idx >= M) idx -= M;      // k < M, así que una resta alcanza
            const float c = tabla_cos[idx];
            const float sn = tabla_sin[idx];
            val_norm += (fa.an[n] * c) + (fa.bn[n] * sn);
            val_corriente += (fa.an_i[n] * c) + (fa.bn_i[n] * sn);
        }

        float v_inst = val_norm * fa.vmax;
        float i_inst = val_corriente;
        if (v_inst > v_max) v_max = v_inst;
        if (v_inst < v_min) v_min = v_inst;
        if (i_inst > i_max) i_max = i_inst;
        if (i_inst < i_min) i_min = i_inst;
        suma_potencia_activa += v_inst * i_inst;
    }

    fa.v_avg_calc = fa.a0 * fa.vmax;

    float suma_potencia_ac = 0.0f;
    for (int n = 1; n <= n_arm; n++) {
        suma_potencia_ac += (fa.an[n] * fa.an[n] + fa.bn[n] * fa.bn[n]) / 2.0f;
    }
    fa.v_rms_calc = fa.vmax * sqrtf((fa.a0 * fa.a0) + suma_potencia_ac);
    fa.v_max_calc = v_max;
    fa.v_min_calc = v_min;
    fa.i_max_calc = i_max;
    fa.i_min_calc = i_min;

    float suma_corriente_ac = 0.0f;
    for (int n = 1; n <= n_arm; n++) {
        suma_corriente_ac += (fa.an_i[n] * fa.an_i[n] + fa.bn_i[n] * fa.bn_i[n]) / 2.0f;
        suma_potencia_reactiva +=
            ((fa.vmax * fa.bn[n]) * fa.an_i[n] - (fa.vmax * fa.an[n]) * fa.bn_i[n]) / 2.0f;
    }

    fa.i_avg_calc = fa.a0_i;
    fa.i_rms_calc = sqrtf((fa.a0_i * fa.a0_i) + suma_corriente_ac);
    fa.potencia_s = fa.v_rms_calc * fa.i_rms_calc;
    fa.potencia_p = suma_potencia_activa / (float)M;
    fa.potencia_q = suma_potencia_reactiva;
    float potencia_d_cuadrado = (fa.potencia_s * fa.potencia_s) -
                                (fa.potencia_p * fa.potencia_p) -
                                (fa.potencia_q * fa.potencia_q);
    fa.potencia_d = sqrtf(max(0.0f, potencia_d_cuadrado));
    fa.factor_potencia = (fa.potencia_s > 0.0f) ? fa.potencia_p / fa.potencia_s : 0.0f;

    // ------------------------------------------------------------------
    // THD (Hart, ec. 6.8):  THD = sqrt( sum_{n>=2} (an^2+bn^2)/2 ) / sqrt( (a1^2+b1^2)/2 )
    // Es la relación entre el valor eficaz de los armónicos (n>=2) y el de la
    // fundamental. El factor 1/2 (amplitud -> eficaz) aparece arriba y abajo y se
    // cancela, por eso se trabaja directamente con las amplitudes.
    // La continua (a0) NO entra: no es un armónico (con la def. de Hart).
    // Se usan los mismos N armónicos (n_arm) que el resto de las métricas, así
    // el THD "ve" lo mismo que el osciloscopio y la tabla de error de la app.
    // La escala vmax se cancela en el cociente, por eso se usan an/bn normalizados.
    // ------------------------------------------------------------------
    float arm_v = 0.0f, arm_i = 0.0f;   // sum_{n>=2} (an^2 + bn^2)
    for (int n = 2; n <= n_arm; n++) {
        arm_v += fa.an[n] * fa.an[n] + fa.bn[n] * fa.bn[n];
        arm_i += fa.an_i[n] * fa.an_i[n] + fa.bn_i[n] * fa.bn_i[n];
    }
    const float fund_v = fa.an[1] * fa.an[1] + fa.bn[1] * fa.bn[1];
    const float fund_i = fa.an_i[1] * fa.an_i[1] + fa.bn_i[1] * fa.bn_i[1];
    // Si la fundamental es ~0 el THD no está definido: se informa 0 (evita dividir por 0)
    fa.thd_v = (fund_v > 1e-12f) ? 100.0f * sqrtf(arm_v / fund_v) : 0.0f;
    fa.thd_i = (fund_i > 1e-12f) ? 100.0f * sqrtf(arm_i / fund_i) : 0.0f;

    Serial.printf("DATA_METRICAS,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                  n_arm, fa.v_max_calc, fa.v_min_calc, fa.v_avg_calc, fa.v_rms_calc,
                  fa.i_max_calc, fa.i_min_calc, fa.i_avg_calc, fa.i_rms_calc,
                  fa.potencia_s, fa.potencia_p, fa.potencia_q, fa.potencia_d,
                  fa.factor_potencia, fa.thd_v, fa.thd_i);
}
