#include <Arduino.h>
#include "funciones.h"

void setup() {
    Serial.setRxBufferSize(24000);
    Serial.begin(115200);
    configurar_n(N_INICIAL);   // n = 360 (consigna); prepara tablas antes de cualquier cálculo
    init_hardware();
    init_control();       // timer y gates (en bajo) ANTES de habilitar la interrupción del ZCD
    init_zcd();
    init_wifi_ota();
}

void loop() {
    // Durante un OTA, loop() se frena: la tarea del core 0 es la que trabaja.
    if (ota_en_curso()) {
        delay(10);
        return;
    }
    recibir_datos_serial();
    atender_tcp();
    control_actualizar();
    verificar_auto_reconstruccion();
    render_pantalla();
}