// control.cpp — Salidas de potencia (disparo de SCR/gate) y secuencias de control.
// Por ahora solo el modo seguro; el control por ZCD llega en el Paso 5.
#include "funciones.h"

// Deja todas las salidas de potencia apagadas. Se llama antes de un OTA (y
// en cualquier situación de falla). Cuando exista el pin de disparo (Paso 5)
// se hará acá: pinMode(PIN_DISPARO, OUTPUT); digitalWrite(PIN_DISPARO, LOW);
void control_modo_seguro() {
}
