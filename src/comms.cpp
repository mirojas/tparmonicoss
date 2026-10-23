// comms.cpp — Comunicación con la app Python: por cable USB (serie) y por WiFi (TCP).
//
// Las dos vías usan EXACTAMENTE el mismo protocolo de texto (una línea por mensaje):
//   PC -> ESP32:  tipo,vmax,freq,Narm,n,v[0..n-1],i[0..n-1]\n
//   ESP32 -> PC:  DATA_METRICAS,N,...\n   (ver fourier.cpp)
// Cualquier línea que entra por serie o por TCP pasa por procesar_trama(); y todo lo
// que el ESP32 responde sale por enviar_linea(), que escribe en Serial y, si hay
// una app conectada por WiFi, también en el socket.
//
// POR QUÉ TCP: es un canal punto a punto confiable (no se pierden ni se desordenan
// bytes) y bastante más rápido que 115200 baud: las ~20 kB de un vector tardan unos
// pocos ms por WiFi contra 1-2 s por serie.
#include "funciones.h"
#include <WiFi.h>
#include <string.h>

static char rx_buffer[24000];

static WiFiServer servidor(TCP_PUERTO);
static WiFiClient cliente;
static bool servidor_iniciado = false;
static bool cliente_activo = false;   // había una app conectada en la última pasada

// Escribe una línea de respuesta por todos los canales activos.
void enviar_linea(const char* linea) {
    Serial.print(linea);
    if (cliente && cliente.connected()) {
        cliente.print(linea);
    }
}

// Procesa una trama ya leída en rx_buffer (bytes_leidos caracteres, sin el '\n').
static void procesar_trama(size_t bytes_leidos) {
    rx_buffer[bytes_leidos] = '\0';

    // Comandos de control de disparo ("CTRL,..."): son cortos, por eso se
    // reconocen ANTES del filtro de longitud de las tramas de vectores.
    if (strncmp(rx_buffer, "CTRL,", 5) == 0) {
        control_procesar_comando(rx_buffer);
        return;
    }

    if (bytes_leidos > 10) {

        char* token = strtok(rx_buffer, ",");
        if (token != NULL) strncpy(fa.tipo, token, sizeof(fa.tipo) - 1);

        token = strtok(NULL, ",");
        if (token != NULL) fa.vmax = atof(token);

        token = strtok(NULL, ",");
        if (token != NULL) strncpy(fa.freq, token, sizeof(fa.freq) - 1);

        // Cantidad de armónicos pedida (se limita DESPUÉS de conocer n, ver más abajo).
        token = strtok(NULL, ",");
        const int armonicos_pedidos = (token != NULL) ? atoi(token) : 1;

        // n: muestras por vector (consigna: entre 120 y 720). Viene en la trama, así el
        // ESP32 sabe cuántos valores esperar. Si está fuera de rango se descarta la trama.
        token = strtok(NULL, ",");
        const int n_recibido = (token != NULL) ? atoi(token) : 0;
        if (n_recibido < MIN_PUNTOS || n_recibido > MAX_PUNTOS) {
            enviar_linea("ERROR_N,fuera_de_120_720\n");
            return;
        }
        configurar_n(n_recibido);                 // recalcula tablas y n_arm_max si cambió
        fa.armonicos_act = constrain(armonicos_pedidos, 1, fa.n_arm_max);
        fa.armonicos_sel = fa.armonicos_act;

        // Primero n valores de tensión y después n de corriente.
        const int n = fa.n_muestras;
        int idx = 0;
        token = strtok(NULL, ",");
        while (token != NULL && idx < (2 * n)) {
            if (idx < n) {
                fa.vector_in[idx] = atof(token);
            } else {
                fa.corriente_in[idx - n] = atof(token);
            }
            idx++;
            token = strtok(NULL, ",");
        }

        if (idx == (2 * n)) {
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

// Recepción UART sin bloqueo de buffer (cable USB)
void recibir_datos_serial() {
    if (Serial.available() > 0) {
        size_t bytes_leidos = Serial.readBytesUntil('\n', rx_buffer, sizeof(rx_buffer) - 1);
        procesar_trama(bytes_leidos);
    }
}

// Recepción por WiFi (TCP). Se llama desde loop(), en el mismo núcleo que el resto
// del procesamiento, así no hay dos hilos tocando el socket ni la estructura fa.
void atender_tcp() {
    if (WiFi.status() != WL_CONNECTED) {
        // Sin WiFi la app tampoco puede estar conectada: si había disparo armado, se apaga.
        if (cliente_activo) {
            control_modo_seguro();
            cliente_activo = false;
            Serial.println("TCP_CLIENTE_DESCONECTADO");
        }
        return;
    }

    // El servidor se abre una sola vez, cuando ya hay WiFi (necesita tener IP).
    if (!servidor_iniciado) {
        servidor.begin();
        servidor.setNoDelay(true);
        servidor_iniciado = true;
        Serial.printf("TCP_READY,%u\n", (unsigned)TCP_PUERTO);
    }

    // Si la app se (re)conecta, se descarta el cliente anterior y se usa el nuevo.
    if (servidor.hasClient()) {
        WiFiClient nuevo = servidor.available();
        if (nuevo) {
            cliente.stop();
            cliente = nuevo;
            cliente.setNoDelay(true);
            cliente.setTimeout(500);   // ms que espera readBytesUntil si la línea llega partida
            control_modo_seguro();     // app nueva: se arranca con el disparo apagado
            Serial.println("TCP_CLIENTE_CONECTADO");
        }
    }

    // Fail-safe: si la app por WiFi se cae mientras había disparo armado, se apaga todo.
    const bool ahora = cliente && cliente.connected();
    if (cliente_activo && !ahora) {
        control_modo_seguro();
        Serial.println("TCP_CLIENTE_DESCONECTADO");
    }
    cliente_activo = ahora;

    if (cliente && cliente.connected() && cliente.available() > 0) {
        size_t bytes_leidos = cliente.readBytesUntil('\n', rx_buffer, sizeof(rx_buffer) - 1);
        procesar_trama(bytes_leidos);
    }
}
