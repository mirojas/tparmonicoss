// ota.cpp — Conexión WiFi y actualización de firmware por aire (ArduinoOTA).
//
// POR QUÉ UNA TAREA EN EL CORE 0
//   ArduinoOTA.handle() tiene que ejecutarse seguido (cada pocas decenas de ms) para
//   atender al PC que sube el firmware. Si estuviera en loop(), un cálculo largo
//   (Fourier, dibujo del OLED) lo dejaría sin atender y el upload fallaría.
//   El ESP32 tiene 2 núcleos: el loop() de Arduino corre en el core 1 y el stack
//   WiFi en el core 0. Poniendo OTA en una tarea del core 0, se atiende en paralelo
//   y no depende de lo que esté calculando loop().
//   (Misma idea que SCR_OTA_14.ino, con las mejoras de abajo.)
//
// MEJORAS RESPECTO DEL .ino DE REFERENCIA
//   - NO hay ESP.restart() si falla el WiFi: el equipo sigue andando por USB y la
//     tarea reintenta la conexión sola (setAutoReconnect + chequeo periódico).
//   - Mientras se actualiza (ota_en_curso) loop() se pausa y se pasa a "modo seguro":
//     se suspende el encoder y se apagan las salidas de potencia.
//   - Credenciales fuera del código (include/secrets.h, no se sube a git).
#include "funciones.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Falta include/secrets.h: copiá include/secrets.h.example como include/secrets.h y completá tus credenciales."
#endif

// Lo escribe la tarea del core 0 (callbacks de OTA) y lo lee loop() en el core 1.
static volatile bool g_ota_en_curso = false;
static bool g_ota_iniciado = false;          // solo lo toca la tarea de OTA
static bool g_wifi_avisado = false;          // para imprimir la IP una sola vez por conexión

bool ota_en_curso() {
    return g_ota_en_curso;
}

// Configura ArduinoOTA. Se llama UNA vez, cuando el WiFi ya está conectado.
static void iniciar_ota() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        // 1. Avisar a loop() que se frene (deja de leer serie, calcular y dibujar).
        g_ota_en_curso = true;
        // 2. Modo seguro: salidas de potencia apagadas y encoder sin interrupciones.
        //    Mientras se escribe la flash no queremos ISR ni disparos de gate.
        control_modo_seguro();
        display_suspender_encoder();
        Serial.println("OTA_START");
    });
    ArduinoOTA.onEnd([]() {
        // Al terminar, ArduinoOTA reinicia el ESP32 solo.
        Serial.println("OTA_END");
    });
    ArduinoOTA.onProgress([](unsigned int actual, unsigned int total) {
        static int ultimo_pct = -10;
        int pct = (int)((100UL * actual) / total);
        if (pct >= ultimo_pct + 10) {      // un mensaje cada 10 %
            ultimo_pct = pct;
            Serial.printf("OTA_PROGRESO,%d\n", pct);
        }
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA_ERROR,%u\n", static_cast<unsigned>(error));
        // Si falló, el equipo sigue funcionando: se reactiva todo.
        display_reanudar_encoder();
        g_ota_en_curso = false;
    });
    ArduinoOTA.begin();   // también publica el nombre por mDNS
}

// Tarea que corre en el core 0: mantiene el WiFi/OTA sin molestar al loop().
static void tarea_ota(void*) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!g_ota_iniciado) {
                iniciar_ota();
                g_ota_iniciado = true;
            }
            if (!g_wifi_avisado) {
                Serial.print("WIFI_READY,");
                Serial.println(WiFi.localIP());
                g_wifi_avisado = true;
            }
            ArduinoOTA.handle();
        } else {
            // Sin conexión: la reconexión la hace el stack WiFi (setAutoReconnect).
            g_wifi_avisado = false;
        }
        // Cede tiempo al sistema: sin esto el watchdog reinicia el ESP32.
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// Arranca WiFi y la tarea. NO bloquea: setup() sigue enseguida y el equipo
// funciona por USB aunque no haya red.
void init_wifi_ota() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    xTaskCreatePinnedToCore(
        tarea_ota,     // función de la tarea
        "Tarea_OTA",   // nombre (para depurar)
        8192,          // stack en bytes
        nullptr,       // parámetro
        1,             // prioridad (baja/normal)
        nullptr,       // handle (no lo necesitamos)
        0              // core 0 (donde vive el stack WiFi)
    );
}
