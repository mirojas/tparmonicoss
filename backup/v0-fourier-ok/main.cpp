#include <Arduino.h>
#include "funciones.h"

void setup() {
    Serial.setRxBufferSize(24000);
    Serial.begin(115200);
    init_hardware();
    init_wifi_ota();
}

void loop() {
    atender_wifi_ota();
    recibir_datos_serial();
    verificar_auto_reconstruccion();
    render_pantalla();
}