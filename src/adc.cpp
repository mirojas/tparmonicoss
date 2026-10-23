// adc.cpp — Captura de tensión y corriente reales por el ADC, sincronizada con el ZCD.
//
// QUÉ HACE
//   Cuando la app pide una captura ("ADC,vmax,R,Narm,n"), espera el PRÓXIMO cruce por
//   cero positivo detectado por zcd.cpp y, a partir de ese instante, toma n muestras de
//   tensión (GPIO34) y n de corriente (GPIO35) repartidas en UN período:
//
//        instante de la muestra k  =  t_cruce + k * T / n        (T = período medido por el ZCD)
//
//   Por qué así: la consigna pide n muestras por período y que la medición arranque
//   en el cruce por cero. Como T se mide en cada ciclo, la captura sigue a la señal
//   aunque la frecuencia no sea exactamente 50 Hz. Además, el índice k queda
//   asociado al ángulo 360*k/n grados, que es lo que necesita el análisis de Fourier.
//
// ESCALA (hay que decidirla porque el ADC solo mide 0 a 3,3 V)
//   Se asume que la señal llega acondicionada a 0..3,3 V con el CERO en 1,65 V
//   (mitad de escala). Entonces, con raw = 0..4095:
//        v_norm = 2*raw/4095 - 1                  (-1 .. +1)
//        v      = v_norm * Vmax                   (la app lo hace con el campo Vmax)
//        i      = (2*raw_i/4095 - 1) * Vmax / R   (carga resistiva: el canal de corriente
//                                                  a fondo de escala equivale a Vmax/R)
//   Con el generador de onda cuadrada de la prueba inicial (0 V / 3,3 V) se obtiene una
//   cuadrada simétrica de +-Vmax, que es justo lo que pide el paso 1 de la consigna.
//
// LIMITACIONES (honestas)
//   - El ADC del ESP32 no es lineal en los extremos (satura cerca de 3,1 V con 11 dB):
//     una cuadrada de 3,3 V se lee como ~4095 y 0 V como ~0, pero una senoide muy grande
//     se aplana. Para medir bien conviene acondicionar la señal a ~0,2..3,0 V.
//   - Cada muestra necesita dos conversiones (V e I). Si n es muy grande no alcanza el
//     tiempo (a 50 Hz y n = 720 hay solo 27 us por muestra): las muestras que llegan tarde
//     se cuentan y se informan en ADC_INFO. Con n <= 360 sobra tiempo.
//   - Se usa solo ADC1 (GPIO32..39) porque ADC2 no funciona con el WiFi encendido.
//   - La captura bloquea loop() durante ~1 período (20 ms): es aceptable y evita ISR con ADC.
#include "funciones.h"
#include <driver/adc.h>
#include <stdio.h>

#define ADC_CANAL_V   ADC1_CHANNEL_6    // GPIO34 (solo entrada)
#define ADC_CANAL_I   ADC1_CHANNEL_7    // GPIO35 (solo entrada)
#define ADC_FONDO     4095.0f           // 12 bits

static volatile bool pedido = false;    // hay una captura pendiente
static float r_carga = 100.0f;          // [ohm], viene en el pedido
static bool modo_libre = false;         // true = captura sin ZCD (prueba de cableado)

// Buffer de salida de la línea ADC_DATA: hasta 2*720 valores de 8 caracteres + cabecera.
static char buf_salida[2 * MAX_PUNTOS * 9 + 48];

void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CANAL_V, ADC_ATTEN_DB_11);   // rango ~0..3,1 V
    adc1_config_channel_atten(ADC_CANAL_I, ADC_ATTEN_DB_11);
}

// La llama comms.cpp al recibir "ADC,...": fa.vmax, n y armónicos ya están cargados.
void adc_pedir(float r_ohm) {
    r_carga = (r_ohm > 0.0f) ? r_ohm : 100.0f;
    // Sin cruce por cero no hay referencia de tiempo: se captura igual, pero "libre" (un período
    // nominal de 20 ms desde ahora, NO sincronizada) y se avisa. Sirve para probar el cableado:
    // una entrada al aire o tocada con el dedo capta ruido y zumbido de 50 Hz y se ve en la app.
    modo_libre = !zcd_presente();
    pedido = true;
}

// Se llama desde loop().
void adc_actualizar() {
    if (!pedido) return;
    pedido = false;

    const int n = fa.n_muestras;
    uint32_t T = zcd_periodo_us();
    uint32_t t0;
    if (modo_libre || T == 0) {
        // Captura libre: período nominal de 50 Hz, arranca ya.
        T = 20000;
        t0 = micros();
        enviar_linea("ADC_LIBRE\n");
    } else {
        // 1) Esperar el PRÓXIMO cruce (se compara el instante del último cruce contra el actual).
        const uint32_t cruce_previo = zcd_ultimo_cruce_us();
        const uint32_t t_inicio = micros();
        while (zcd_ultimo_cruce_us() == cruce_previo) {
            if ((uint32_t)(micros() - t_inicio) > 60000UL) {   // más de 60 ms sin cruce
                enviar_linea("ADC_SIN_ZCD\n");
                return;
            }
        }
        t0 = zcd_ultimo_cruce_us();
    }

    // 2) n muestras repartidas en un período, a partir del cruce.
    const uint32_t paso_us = T / (uint32_t)n;   // tiempo disponible por muestra
    int tarde = 0;
    const float escala_i = fa.vmax / r_carga;   // [A] a fondo de escala del canal de corriente
    for (int k = 0; k < n; k++) {
        const uint32_t objetivo = t0 + (T * (uint32_t)k) / (uint32_t)n;
        while ((int32_t)(micros() - objetivo) < 0) { }          // espera activa hasta el instante k
        if ((uint32_t)(micros() - objetivo) > paso_us) tarde++; // ya pasó más de una muestra

        const int raw_v = adc1_get_raw(ADC_CANAL_V);
        const int raw_i = adc1_get_raw(ADC_CANAL_I);
        fa.vector_in[k]    = 2.0f * (float)raw_v / ADC_FONDO - 1.0f;           // normalizada (-1..1)
        fa.corriente_in[k] = (2.0f * (float)raw_i / ADC_FONDO - 1.0f) * escala_i;   // [A]
    }

    // 3) Mismo análisis que para las señales sintéticas.
    strncpy(fa.tipo, "ADC", sizeof(fa.tipo) - 1);
    fa.datos_listos = true;
    analizar_fourier();

    // 4) Se devuelven las muestras a la app (para graficarlas) y luego las métricas.
    int len = snprintf(buf_salida, sizeof(buf_salida), "ADC_DATA,%d", n);
    for (int k = 0; k < n; k++) {
        len += snprintf(buf_salida + len, sizeof(buf_salida) - len, ",%.4f", fa.vector_in[k]);
    }
    for (int k = 0; k < n; k++) {
        len += snprintf(buf_salida + len, sizeof(buf_salida) - len, ",%.4f", fa.corriente_in[k]);
    }
    snprintf(buf_salida + len, sizeof(buf_salida) - len, "\n");
    enviar_linea(buf_salida);

    calcular_metricas_y_reportar(fa.armonicos_act);

    char info[80];
    snprintf(info, sizeof(info), "ADC_INFO,n=%d,T_us=%lu,paso_us=%lu,tarde=%d\n",
             n, (unsigned long)T, (unsigned long)paso_us, tarde);
    enviar_linea(info);

    cambio_pendiente = false;
    actualizar_pantalla = true;
}
