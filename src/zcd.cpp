// zcd.cpp — Detector de cruce por cero (ZCD) por interrupción.
//
// QUÉ HACE
//   Mide cada cuánto llega un flanco al pin PIN_ZCD y, de ahí, el período y la
//   frecuencia de la red. Cada flanco válido le avisa a control.cpp
//   (control_on_cruce) para que arranque la cuenta del ángulo de disparo.
//
// QUÉ SEÑAL ESPERA (TP, "Conexión física")
//   1) Prueba inicial: un generador de onda CUADRADA de 50 Hz directo al pin.
//      Un flanco por ciclo = el cruce por cero positivo de la senoide asociada.
//   2) Circuito final con optoacoplador: lo importante es que el pin reciba UN
//      flanco por ciclo en el cruce por cero positivo. Si tu circuito da un pulso
//      por SEMICICLO (cada 10 ms), el filtro de período de abajo lo rechaza y habrá
//      que ajustar (avisame y lo adaptamos).
//   El flanco a usar se elige con ZCD_FLANCO (RISING o FALLING). El .ino de
//   referencia usaba FALLING; con el generador cuadrado, RISING es el cruce positivo.
//
// POR QUÉ SE FILTRA
//   - Rebote/ruido: se ignora cualquier flanco a menos de ZCD_REBOTE_US del anterior.
//   - Período absurdo: solo se aceptan períodos de 15 a 25 ms (40 a 66 Hz).
//   - "Red presente" recién después de ZCD_CRUCES_VALIDOS períodos válidos seguidos
//     y mientras el último flanco sea reciente (ZCD_TIMEOUT_US).
#include "funciones.h"

#define PIN_ZCD              4          // GPIO del ZCD (el mismo del .ino de referencia)
#ifndef ZCD_FLANCO
#define ZCD_FLANCO           RISING     // flanco que marca el cruce por cero positivo
#endif
#define ZCD_REBOTE_US        2000UL     // ignora flancos más cercanos que esto
#define ZCD_PERIODO_MIN_US   15000UL    // 66,7 Hz
#define ZCD_PERIODO_MAX_US   25000UL    // 40 Hz
#define ZCD_CRUCES_VALIDOS   2          // períodos válidos seguidos para dar la red por presente
#define ZCD_TIMEOUT_US       60000UL    // sin flancos por más de esto = red ausente

static volatile uint32_t ultimo_cruce_us = 0;   // instante del último flanco aceptado
static volatile uint32_t periodo_us = 0;        // último período válido medido
static volatile uint8_t  cruces_validos = 0;    // períodos válidos consecutivos

// ISR del flanco. Va en IRAM (RAM interna) para poder correr aunque la flash esté ocupada.
// No usa float: las ISR del ESP32 no deben usar la FPU.
static void IRAM_ATTR isr_zcd() {
    const uint32_t t = (uint32_t)micros();
    const uint32_t dt = t - ultimo_cruce_us;
    if (dt < ZCD_REBOTE_US) return;             // rebote: se descarta
    ultimo_cruce_us = t;

    if (dt >= ZCD_PERIODO_MIN_US && dt <= ZCD_PERIODO_MAX_US) {
        periodo_us = dt;
        if (cruces_validos < 255) cruces_validos++;
        if (cruces_validos >= ZCD_CRUCES_VALIDOS) {
            control_on_cruce();                 // arranca la secuencia de disparo
        }
    } else {
        cruces_validos = 0;                     // período fuera de rango: se pierde el enganche
    }
}

void init_zcd() {
    pinMode(PIN_ZCD, INPUT);   // la señal viene con niveles lógicos (generador u optoacoplador con pull-up externo)
    attachInterrupt(digitalPinToInterrupt(PIN_ZCD), isr_zcd, ZCD_FLANCO);
}

// Durante un OTA no queremos interrupciones (se escribe la flash).
void zcd_suspender() { detachInterrupt(digitalPinToInterrupt(PIN_ZCD)); }
void zcd_reanudar()  { attachInterrupt(digitalPinToInterrupt(PIN_ZCD), isr_zcd, ZCD_FLANCO); }

bool zcd_presente() {
    return (cruces_validos >= ZCD_CRUCES_VALIDOS) &&
           ((uint32_t)micros() - ultimo_cruce_us < ZCD_TIMEOUT_US);
}

// Se llama desde la ISR de control: va en IRAM.
uint32_t IRAM_ATTR zcd_periodo_us() { return periodo_us; }

float zcd_frecuencia_hz() {
    const uint32_t p = periodo_us;
    return (p > 0) ? 1.0e6f / (float)p : 0.0f;
}
