// ota.cpp — Conexión WiFi y actualización OTA (ArduinoOTA).
// (Movido sin cambios de lógica desde funciones.cpp)
// TODO Paso 4: pasar credenciales a secrets.h y mover handle() a tarea en core 0.
#include "funciones.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

static const char* WIFI_SSID = "EDP-TESTING";
static const char* WIFI_PASSWORD = "Potencia26";
static const char* OTA_HOSTNAME = "esp32-potencia";
static const char* OTA_PASSWORD = "edptest";

void init_wifi_ota() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const unsigned long inicio = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - inicio < 15000UL) {
        delay(250);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WIFI_ERROR");
        return;
    }

    if (!MDNS.begin(OTA_HOSTNAME)) {
        Serial.println("MDNS_ERROR");
    }

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        Serial.println("OTA_START");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA_END");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA_ERROR,%u\n", static_cast<unsigned>(error));
    });
    ArduinoOTA.begin();

    Serial.print("WIFI_READY,");
    Serial.println(WiFi.localIP());
}

void atender_wifi_ota() {
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }
}
