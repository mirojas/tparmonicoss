// control.cpp — Disparo de rectificadores controlados (RMMOC y RMOCC) sincronizado
// con el cruce por cero.
//
// IDEA
//   Cada cruce por cero positivo (zcd.cpp) llama a control_on_cruce(). Ahí se arma
//   una secuencia corta de eventos "subir/bajar pin de gate" medidos desde el cruce:
//
//     RMMOC (media onda controlado):  gate1 sube a alfa, baja a alfa+ancho.
//     RMOCC (onda completa controlado): lo mismo en gate1 y, medio período después,
//                                        otro pulso igual en gate2 (el semiciclo negativo).
//
//   El ángulo de disparo alfa [grados] se convierte en tiempo con el período medido:
//       retardo = periodo * alfa / 360   -   compensación
//   (es lo que hace el .ino de referencia, pero con un timer de hardware).
//
// POR QUÉ UN TIMER DE HARDWARE Y NO LOOP()
//   El .ino de referencia disparaba desde loop() comparando micros(). Acá loop()
//   puede quedar ocupado muchos milisegundos (cálculo de Fourier, OLED, red), lo que
//   movería el disparo. El timer de hardware genera los flancos con precisión de µs
//   sin depender de loop().
//
// COMPENSACIÓN (TP, "Conexión física" punto 3)
//   comp_us se RESTA al retardo para absorber el retraso del optoacoplador y del
//   circuito de disparo. Hay que ajustarlo midiendo con osciloscopio.
//
// SEGURIDAD
//   - Al arrancar, los gates están en BAJO y el control está APAGADO (modo OFF).
//   - Solo se dispara tras un comando explícito de la app (CTRL,...).
//   - Sin cruce por cero no se dispara nada (y se informa por serie/WiFi).
//   - Si la app por WiFi se desconecta, se apaga todo (ver comms.cpp).
//   - Antes de armar cada secuencia se bajan los gates, por si un pulso quedó a medias.
//
// COMANDOS (texto, una línea):   CTRL,OFF
//                                CTRL,RMMOC,alfa_deg,ancho_us,comp_us
//                                CTRL,RMOCC,alfa_deg,ancho_us,comp_us
//   Respuestas:  CTRL_OK,... / CTRL_ERROR,...   y cada 1 s:  ZCD_ESTADO,...
//
// NOTA: usa la API de timers de arduino-esp32 2.x (timerBegin(num, divisor, ...)),
// que es la que trae platform espressif32 6.x.
#include "funciones.h"
#include <soc/gpio_struct.h>   // GPIO.out_w1ts / out_w1tc: poner/bajar pines desde una ISR
#include <math.h>
#include <string.h>

#define PIN_GATE1   16        // gate del SCR 1 (el mismo del .ino de referencia)
#define PIN_GATE2   17        // gate del SCR 2 (semiciclo negativo, RMOCC)
#define MASCARA_G1  (1UL << PIN_GATE1)
#define MASCARA_G2  (1UL << PIN_GATE2)

#define ALFA_MIN_CDEG    500      // 5,00°   (mismo rango del .ino: evita solaparse con el pulso del ZCD)
#define ALFA_MAX_CDEG  17500      // 175,00°
#define ANCHO_MIN_US      10
#define ANCHO_MAX_US    2000
#define COMP_MAX_US     2000

enum ModoControl : uint8_t { CTRL_OFF = 0, CTRL_RMMOC = 1, CTRL_RMOCC = 2 };

// Parámetros (los escribe el comando; los lee la ISR). El ángulo se guarda en
// centésimas de grado ENTERAS para no usar float dentro de la ISR.
static volatile uint8_t  modo = CTRL_OFF;
static volatile uint32_t alfa_cdeg = 9000;     // 90,00°
static volatile uint32_t ancho_us = 50;
static volatile uint32_t comp_us = 100;

// Secuencia de eventos de un ciclo: cada uno espera dt_us DESDE EL EVENTO ANTERIOR
// (el primero, desde el cruce) y después sube o baja un gate.
struct Evento { uint32_t dt_us; uint32_t mascara; bool nivel; };
static volatile Evento eventos[4];
static volatile uint8_t n_eventos = 0;
static volatile uint8_t idx_evento = 0;

static hw_timer_t* timer = nullptr;

// Escribe un evento. Se asigna campo por campo porque el arreglo es volatile.
static inline void poner_evento(uint8_t i, uint32_t dt, uint32_t mascara, bool nivel) {
    eventos[i].dt_us = dt;
    eventos[i].mascara = mascara;
    eventos[i].nivel = nivel;
}

// ISR del timer: ejecuta el evento actual y deja armado el siguiente.
static void IRAM_ATTR isr_timer() {
    if (idx_evento >= n_eventos) return;
    const uint32_t mascara = eventos[idx_evento].mascara;
    if (eventos[idx_evento].nivel) GPIO.out_w1ts = mascara;   // gate a ALTO
    else                           GPIO.out_w1tc = mascara;   // gate a BAJO
    idx_evento++;
    if (idx_evento < n_eventos) {
        timerWrite(timer, 0);                                  // la cuenta se reinicia en cada evento
        timerAlarmWrite(timer, eventos[idx_evento].dt_us, false);
        timerAlarmEnable(timer);
    }
}

void init_control() {
    pinMode(PIN_GATE1, OUTPUT);
    pinMode(PIN_GATE2, OUTPUT);
    digitalWrite(PIN_GATE1, LOW);
    digitalWrite(PIN_GATE2, LOW);
    // Timer 0, divisor 80 => 80 MHz / 80 = 1 MHz => 1 cuenta = 1 µs.
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &isr_timer, true);
}

// Apaga todo: sin disparos y gates en bajo. Se usa antes de un OTA y ante cualquier falla.
void control_modo_seguro() {
    modo = CTRL_OFF;
    if (timer) timerAlarmDisable(timer);
    GPIO.out_w1tc = MASCARA_G1 | MASCARA_G2;
    n_eventos = 0;
    idx_evento = 0;
}

// Llamada por la ISR del ZCD en cada cruce positivo válido.
void IRAM_ATTR control_on_cruce() {
    if (modo == CTRL_OFF) return;

    // Un pulso del ciclo anterior podría haber quedado a medias: se corta.
    timerAlarmDisable(timer);
    GPIO.out_w1tc = MASCARA_G1 | MASCARA_G2;

    const uint32_t T = zcd_periodo_us();
    // retardo = T * alfa / 360 - compensación (alfa en centésimas de grado => /36000)
    int32_t retardo = (int32_t)(((uint64_t)T * alfa_cdeg) / 36000UL) - (int32_t)comp_us;
    if (retardo < 10) retardo = 10;

    uint32_t w = ancho_us;
    if (w > T / 8) w = T / 8;      // el pulso nunca debe ocupar una fracción grande del ciclo
    if (w < ANCHO_MIN_US) w = ANCHO_MIN_US;

    // Pulso 1: gate1 sube a 'retardo', baja 'w' después.
    poner_evento(0, (uint32_t)retardo, MASCARA_G1, true);
    poner_evento(1, w,                 MASCARA_G1, false);
    n_eventos = 2;

    if (modo == CTRL_RMOCC) {
        // Pulso 2, medio período después del pulso 1: gate2 sube/baja.
        // (desde el final del pulso 1 faltan T/2 - w para llegar a 'retardo + T/2')
        poner_evento(2, T / 2 - w, MASCARA_G2, true);
        poner_evento(3, w,         MASCARA_G2, false);
        n_eventos = 4;
    }

    idx_evento = 0;
    timerWrite(timer, 0);
    timerAlarmWrite(timer, eventos[0].dt_us, false);
    timerAlarmEnable(timer);
}

// Interpreta una línea de comando "CTRL,...". 'linea' se modifica (strtok).
void control_procesar_comando(char* linea) {
    strtok(linea, ",");                          // descarta "CTRL"
    const char* m = strtok(NULL, ",");
    if (m == NULL) { enviar_linea("CTRL_ERROR,formato\n"); return; }

    if (strcmp(m, "OFF") == 0) {
        control_modo_seguro();
        enviar_linea("CTRL_OK,OFF\n");
        return;
    }

    uint8_t nuevo;
    if      (strcmp(m, "RMMOC") == 0) nuevo = CTRL_RMMOC;
    else if (strcmp(m, "RMOCC") == 0) nuevo = CTRL_RMOCC;
    else { enviar_linea("CTRL_ERROR,modo\n"); return; }

    const char* t_alfa  = strtok(NULL, ",");
    const char* t_ancho = strtok(NULL, ",");
    const char* t_comp  = strtok(NULL, ",");
    if (t_alfa == NULL) { enviar_linea("CTRL_ERROR,falta_alfa\n"); return; }

    long a = lroundf(atof(t_alfa) * 100.0f);                 // grados -> centésimas
    long w = (t_ancho != NULL) ? atol(t_ancho) : 50;
    long c = (t_comp  != NULL) ? atol(t_comp)  : 100;
    a = constrain(a, (long)ALFA_MIN_CDEG, (long)ALFA_MAX_CDEG);
    w = constrain(w, (long)ANCHO_MIN_US, (long)ANCHO_MAX_US);
    c = constrain(c, 0L, (long)COMP_MAX_US);

    // Primero los parámetros y recién al final el modo: así la ISR nunca ve
    // el modo nuevo con parámetros viejos.
    alfa_cdeg = (uint32_t)a;
    ancho_us = (uint32_t)w;
    comp_us = (uint32_t)c;
    modo = nuevo;

    char resp[96];
    snprintf(resp, sizeof(resp), "CTRL_OK,%s,%.2f,%ld,%ld\n", m, a / 100.0f, w, c);
    enviar_linea(resp);
}

// Se llama desde loop(): informa si hay o no cruce por cero y manda el estado cada 1 s.
void control_actualizar() {
    static bool presente_antes = false;
    static unsigned long t_ultimo = 0;

    const bool presente = zcd_presente();
    if (presente != presente_antes) {
        presente_antes = presente;
        // TP: "si no se detecta cruce por cero: informar"
        enviar_linea(presente ? "ZCD_OK\n" : "ZCD_SIN_CRUCE\n");
    }

    if (millis() - t_ultimo >= 1000UL) {
        t_ultimo = millis();
        char linea[128];
        snprintf(linea, sizeof(linea), "ZCD_ESTADO,%d,%lu,%.2f,%u,%.2f,%lu,%lu\n",
                 presente ? 1 : 0, (unsigned long)zcd_periodo_us(), zcd_frecuencia_hz(),
                 (unsigned)modo, alfa_cdeg / 100.0f, (unsigned long)ancho_us,
                 (unsigned long)comp_us);
        enviar_linea(linea);
    }
}
