// control.cpp — Control sincronizado con el cruce por cero (ZCD):
//   * Rectificadores controlados RMMOC y RMOCC (disparo de SCR a un ángulo alfa).
//   * Inversor con puente H: onda CUADRADA y CUASICUADRADA, y 50 Hz libres si no hay red.
//
// IDEA COMÚN
//   Cada cruce por cero positivo (zcd.cpp) llama a control_on_cruce(). Ahí se arma
//   una secuencia corta de eventos "subir/bajar un pin" medidos desde el cruce, y un
//   timer de hardware los ejecuta con precisión de microsegundos.
//
// POR QUÉ UN TIMER DE HARDWARE Y NO LOOP()
//   El .ino de referencia disparaba desde loop() comparando micros(). Acá loop()
//   puede quedar ocupado muchos milisegundos (cálculo de Fourier, OLED, red), lo que
//   movería el disparo. El timer de hardware no depende de loop().
//
// ---------------------------------------------------------------------------
// RECTIFICADORES  (comando CTRL,RMMOC|RMOCC,alfa_deg,ancho_us,comp_us)
//   RMMOC (media onda):    gate1 sube a alfa y baja 'ancho' después.
//   RMOCC (onda completa): lo mismo en gate1 y, medio período después, en gate2.
//   El ángulo se convierte en tiempo con el período medido:
//       retardo = periodo * alfa / 360 - compensación
//   comp_us absorbe el retraso del optoacoplador y del disparo (TP, "Conexión física" 3).
//
// INVERSOR CON PUENTE H  (comando CTRL,INVSQ|INVQS,alfa(no se usa),tmuerto_us,comp_us)
//   El puente H tiene dos diagonales de llaves: A (positiva) y B (negativa).
//   Se usan dos pines: PIN_INV_A y PIN_INV_B (hay que excitar las llaves con drivers
//   y aislación; el ESP32 solo da la señal lógica).
//
//   CUADRADA ("inmediatamente" tras el cruce, TP):
//       A = 1 desde el cruce hasta T/2 - tmuerto
//       B = 1 desde T/2 hasta T - tmuerto
//     El TIEMPO MUERTO evita que A y B conduzcan a la vez (cortocircuito de la
//     alimentación, "shoot-through"): una diagonal se apaga 'tmuerto' ANTES de que
//     se encienda la otra.
//
//   CUASICUADRADA ("30° después del cruce", TP): pulsos de 120° con 60° de cero entre ellos
//       A = 1 de 30° a 150°      B = 1 de 210° a 330°
//     (coincide con la señal "Cuasi" de la app). Los 60° de cero ya separan A y B,
//     así que no hace falta tiempo muerto. 'comp_us' adelanta toda la secuencia
//     para compensar el retraso del ZCD.
//
//   SIN CRUCE POR CERO: el inversor genera 50 Hz en forma DESINCRONIZADA (período fijo
//   de 20 ms). Cuando vuelve la red, el primer cruce válido lo sincroniza (de golpe).
//   La resincronización suave (variar la frecuencia y poner en fase lentamente, "último
//   paso" del TP) NO está implementada.
//
// ---------------------------------------------------------------------------
// SEGURIDAD
//   - Al arrancar, todos los pines están en BAJO y el control está APAGADO (modo OFF).
//   - Solo se dispara tras un comando explícito de la app (CTRL,...).
//   - Rectificadores: sin cruce por cero no se dispara nada (y se informa).
//   - Si la app por WiFi se desconecta, se apaga todo (ver comms.cpp).
//   - Antes de armar cada secuencia se bajan los pines, por si algo quedó a medias.
//
// Respuestas:  CTRL_OK,... / CTRL_ERROR,...   y cada 1 s:  ZCD_ESTADO,...
//
// NOTA: usa la API de timers de arduino-esp32 2.x (timerBegin(num, divisor, ...)),
// que es la que trae platform espressif32 6.x.
#include "funciones.h"
#include <soc/gpio_struct.h>   // GPIO.out_w1ts / out_w1tc: poner/bajar pines desde una ISR
#include <math.h>
#include <string.h>

// Pines. Ya ocupados por el resto del proyecto: 4 (ZCD), 5 y 18 (encoder), 21 y 22 (OLED).
#define PIN_GATE1   16        // gate del SCR 1 (el mismo del .ino de referencia)
#define PIN_GATE2   17        // gate del SCR 2 (semiciclo negativo, RMOCC)
#define PIN_INV_A   25        // puente H, diagonal A (semiciclo positivo)
#define PIN_INV_B   26        // puente H, diagonal B (semiciclo negativo)
#define MASCARA_G1     (1UL << PIN_GATE1)
#define MASCARA_G2     (1UL << PIN_GATE2)
#define MASCARA_INV_A  (1UL << PIN_INV_A)
#define MASCARA_INV_B  (1UL << PIN_INV_B)
#define MASCARA_TODAS  (MASCARA_G1 | MASCARA_G2 | MASCARA_INV_A | MASCARA_INV_B)

#define ALFA_MIN_CDEG    500      // 5,00°   (mismo rango del .ino: evita solaparse con el pulso del ZCD)
#define ALFA_MAX_CDEG  17500      // 175,00°
#define ANCHO_MIN_US      2       // (en inversor este parámetro es el tiempo muerto)
#define ANCHO_MAX_US    2000
#define COMP_MAX_US     2000
#define PERIODO_LIBRE_US 20000    // 50 Hz cuando no hay red

enum ModoControl : uint8_t {
    CTRL_OFF = 0, CTRL_RMMOC = 1, CTRL_RMOCC = 2, CTRL_INV_CUAD = 3, CTRL_INV_CUASI = 4
};

// Parámetros (los escribe el comando; los lee la ISR). El ángulo se guarda en
// centésimas de grado ENTERAS para no usar float dentro de la ISR.
static volatile uint8_t  modo = CTRL_OFF;
static volatile uint32_t alfa_cdeg = 9000;     // 90,00°
static volatile uint32_t ancho_us = 50;        // rectificador: ancho del pulso; inversor: tiempo muerto
static volatile uint32_t comp_us = 100;
static volatile bool     libre = false;        // true: el inversor corre solo a 50 Hz (sin red)

// Secuencia de eventos de un ciclo: cada uno espera dt_us DESDE EL EVENTO ANTERIOR
// (el primero, desde el cruce) y después sube o baja los pines de 'mascara'.
struct Evento { uint32_t dt_us; uint32_t mascara; bool nivel; };
static volatile Evento eventos[5];
static volatile uint8_t n_eventos = 0;
static volatile uint8_t idx_evento = 0;

static hw_timer_t* timer = nullptr;

// Escribe un evento. Se asigna campo por campo porque el arreglo es volatile.
static inline void IRAM_ATTR poner_evento(uint8_t i, uint32_t dt, uint32_t mascara, bool nivel) {
    eventos[i].dt_us = dt;
    eventos[i].mascara = mascara;
    eventos[i].nivel = nivel;
}

static inline bool es_inversor(uint8_t m) {
    return m == CTRL_INV_CUAD || m == CTRL_INV_CUASI;
}

// ISR del timer: ejecuta el evento actual y deja armado el siguiente.
static void IRAM_ATTR isr_timer() {
    if (idx_evento >= n_eventos) return;
    const uint32_t mascara = eventos[idx_evento].mascara;
    if (eventos[idx_evento].nivel) GPIO.out_w1ts = mascara;   // pines a ALTO
    else                           GPIO.out_w1tc = mascara;   // pines a BAJO
    idx_evento++;
    // Modo libre: al terminar el ciclo se vuelve a empezar solo (el último evento
    // es una espera vacía que completa los 20 ms).
    if (idx_evento >= n_eventos && libre) idx_evento = 0;
    if (idx_evento < n_eventos) {
        timerWrite(timer, 0);                                  // la cuenta se reinicia en cada evento
        timerAlarmWrite(timer, eventos[idx_evento].dt_us, false);
        timerAlarmEnable(timer);
    }
}

void init_control() {
    const uint8_t pines[4] = { PIN_GATE1, PIN_GATE2, PIN_INV_A, PIN_INV_B };
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(pines[i], OUTPUT);
        digitalWrite(pines[i], LOW);
    }
    // Timer 0, divisor 80 => 80 MHz / 80 = 1 MHz => 1 cuenta = 1 µs.
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &isr_timer, true);
}

// Apaga todo: sin disparos y pines en bajo. Se usa antes de un OTA y ante cualquier falla.
void control_modo_seguro() {
    modo = CTRL_OFF;
    libre = false;
    if (timer) timerAlarmDisable(timer);
    GPIO.out_w1tc = MASCARA_TODAS;
    n_eventos = 0;
    idx_evento = 0;
}

// Arma la secuencia de eventos del modo actual para un período T [us].
// Va en IRAM porque la usa la ISR del ZCD. Sin float.
static void IRAM_ATTR cargar_secuencia(uint32_t T) {
    uint32_t acumulado = 0;   // suma de los dt cargados (para completar el ciclo en modo libre)
    uint8_t n = 0;

    if (modo == CTRL_RMMOC || modo == CTRL_RMOCC) {
        // retardo = T * alfa / 360 - compensación (alfa en centésimas de grado => /36000)
        int32_t retardo = (int32_t)(((uint64_t)T * alfa_cdeg) / 36000UL) - (int32_t)comp_us;
        if (retardo < 10) retardo = 10;
        uint32_t w = ancho_us;
        if (w > T / 8) w = T / 8;      // el pulso nunca debe ocupar una fracción grande del ciclo
        if (w < 10) w = 10;

        // Pulso 1: gate1 sube a 'retardo', baja 'w' después.
        poner_evento(n++, (uint32_t)retardo, MASCARA_G1, true);
        poner_evento(n++, w,                 MASCARA_G1, false);
        if (modo == CTRL_RMOCC) {
            // Pulso 2, medio período después del pulso 1: gate2 sube/baja.
            // (desde el final del pulso 1 faltan T/2 - w para llegar a 'retardo + T/2')
            poner_evento(n++, T / 2 - w, MASCARA_G2, true);
            poner_evento(n++, w,         MASCARA_G2, false);
        }
    } else if (modo == CTRL_INV_CUAD) {
        uint32_t dm = ancho_us;                    // tiempo muerto
        if (dm > T / 20) dm = T / 20;
        if (dm < ANCHO_MIN_US) dm = ANCHO_MIN_US;
        // A: de 0 a T/2 - dm.   B: de T/2 a T - dm.
        poner_evento(n++, 2,               MASCARA_INV_A, true);    // "inmediatamente" (2 us)
        poner_evento(n++, T / 2 - dm - 2,  MASCARA_INV_A, false);
        poner_evento(n++, dm,              MASCARA_INV_B, true);
        poner_evento(n++, T / 2 - dm,      MASCARA_INV_B, false);
    } else if (modo == CTRL_INV_CUASI) {
        // A: de 30° a 150°.  B: de 210° a 330°.  (30° = T/12). Con red, se adelanta comp_us.
        int32_t inicio = (int32_t)(T / 12) - (libre ? 0 : (int32_t)comp_us);
        if (inicio < 2) inicio = 2;
        poner_evento(n++, (uint32_t)inicio, MASCARA_INV_A, true);
        poner_evento(n++, T / 3,            MASCARA_INV_A, false);   // 30° -> 150°
        poner_evento(n++, T / 6,            MASCARA_INV_B, true);    // 150° -> 210°
        poner_evento(n++, T / 3,            MASCARA_INV_B, false);   // 210° -> 330°
    }

    for (uint8_t i = 0; i < n; i++) acumulado += eventos[i].dt_us;
    if (libre && n > 0) {
        // Espera vacía para que el ciclo dure exactamente T y se repita.
        const uint32_t resto = (acumulado < T) ? (T - acumulado) : 2;
        poner_evento(n++, resto < 2 ? 2 : resto, 0, false);
    }
    n_eventos = n;
}

// Llamada por la ISR del ZCD en cada cruce positivo válido.
void IRAM_ATTR control_on_cruce() {
    if (modo == CTRL_OFF) return;

    libre = false;                 // hay red: se sincroniza (si venía en modo libre, se corta)
    // Algo del ciclo anterior podría haber quedado a medias: se corta.
    timerAlarmDisable(timer);
    GPIO.out_w1tc = MASCARA_TODAS;

    cargar_secuencia(zcd_periodo_us());

    idx_evento = 0;
    timerWrite(timer, 0);
    timerAlarmWrite(timer, eventos[0].dt_us, false);
    timerAlarmEnable(timer);
}

// Arranca el inversor "libre": 50 Hz sin red. Se llama desde loop(), no desde una ISR.
// noInterrupts() evita que un cruce por cero llegue justo mientras se arma la secuencia.
static void control_iniciar_libre() {
    noInterrupts();
    timerAlarmDisable(timer);
    GPIO.out_w1tc = MASCARA_TODAS;
    libre = true;
    cargar_secuencia(PERIODO_LIBRE_US);
    idx_evento = 0;
    timerWrite(timer, 0);
    timerAlarmWrite(timer, eventos[0].dt_us, false);
    timerAlarmEnable(timer);
    interrupts();
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
    else if (strcmp(m, "INVSQ") == 0) nuevo = CTRL_INV_CUAD;
    else if (strcmp(m, "INVQS") == 0) nuevo = CTRL_INV_CUASI;
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

    // Se detiene lo que hubiera en marcha y recién después se cambian los parámetros;
    // el modo se escribe al final, así la ISR nunca ve el modo nuevo con parámetros viejos.
    noInterrupts();
    timerAlarmDisable(timer);
    libre = false;
    GPIO.out_w1tc = MASCARA_TODAS;
    alfa_cdeg = (uint32_t)a;
    ancho_us = (uint32_t)w;
    comp_us = (uint32_t)c;
    modo = nuevo;
    interrupts();

    // Inversor sin red: arranca solo a 50 Hz. Con red espera el próximo cruce (< 20 ms).
    if (es_inversor(nuevo) && !zcd_presente()) {
        control_iniciar_libre();
    }

    char resp[96];
    snprintf(resp, sizeof(resp), "CTRL_OK,%s,%.2f,%ld,%ld\n", m, a / 100.0f, w, c);
    enviar_linea(resp);
}

// Se llama desde loop(): informa si hay o no cruce por cero, maneja el modo libre
// del inversor y manda el estado cada 1 s.
void control_actualizar() {
    static bool presente_antes = false;
    static unsigned long t_ultimo = 0;

    const bool presente = zcd_presente();
    if (presente != presente_antes) {
        presente_antes = presente;
        // TP: "si no se detecta cruce por cero: informar"
        enviar_linea(presente ? "ZCD_OK\n" : "ZCD_SIN_CRUCE\n");
    }

    // Inversor armado y la red desapareció: sigue generando 50 Hz por su cuenta.
    if (es_inversor(modo) && !presente && !libre) {
        control_iniciar_libre();
        enviar_linea("INV_LIBRE_50HZ\n");
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
