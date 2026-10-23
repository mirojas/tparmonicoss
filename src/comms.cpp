// comms.cpp — Protocolo serie con la app Python (recepción de vectores).
// (Movido sin cambios de lógica desde funciones.cpp)
#include "funciones.h"
#include <string.h>

static char rx_buffer[24000];

// 4. Recepción UART sin bloqueo de buffer
void recibir_datos_serial() {
    if (Serial.available() > 0) {
        size_t bytes_leidos = Serial.readBytesUntil('\n', rx_buffer, sizeof(rx_buffer) - 1);
        rx_buffer[bytes_leidos] = '\0';

        if (bytes_leidos > 10) {
            char* token = strtok(rx_buffer, ",");
            if (token != NULL) strncpy(fa.tipo, token, sizeof(fa.tipo) - 1);

            token = strtok(NULL, ",");
            if (token != NULL) fa.vmax = atof(token);

            token = strtok(NULL, ",");
            if (token != NULL) strncpy(fa.freq, token, sizeof(fa.freq) - 1);

            token = strtok(NULL, ",");
            if (token != NULL) {
                fa.armonicos_act = constrain(atoi(token), 1, MAX_ARMONICOS);
                fa.armonicos_sel = fa.armonicos_act;
            }

            int idx = 0;
            token = strtok(NULL, ",");
            while (token != NULL && idx < (2 * MAX_PUNTOS)) {
                if (idx < MAX_PUNTOS) {
                    fa.vector_in[idx] = atof(token);
                } else {
                    fa.corriente_in[idx - MAX_PUNTOS] = atof(token);
                }
                idx++;
                token = strtok(NULL, ",");
            }

            if (idx == (2 * MAX_PUNTOS)) {
                fa.datos_listos = true;
                
                // Conserva N armónicos actuales
                int n_mantener = (fa.armonicos_act >= 1) ? fa.armonicos_act : 1;
                fa.armonicos_sel = n_mantener;
                fa.armonicos_act = n_mantener;
                
                // Medición de tiempos para ver cuánto tarda cada etapa (ms).
                // La app Python ignora esta línea (solo procesa DATA_METRICAS).
                const unsigned long t0 = millis();
                analizar_fourier();
                const unsigned long t1 = millis();
                calcular_metricas_y_reportar(n_mantener);
                const unsigned long t2 = millis();
                Serial.printf("INFO_TIEMPO_MS,N=%d,fourier=%lu,metricas=%lu\n",
                              n_mantener, t1 - t0, t2 - t1);
                
                cambio_pendiente = false;
                actualizar_pantalla = true;
            }
        }
    }
}
