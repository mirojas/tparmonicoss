// display.cpp — OLED SSD1306, osciloscopio y encoder rotativo.
// (Movido sin cambios de lógica desde funciones.cpp)
#include "funciones.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <string.h>

#define PIN_DT       5
#define PIN_CLK      18
#define PIN_SDA      21
#define PIN_SCL      22
#define OLED_ADDR    0x3C


static Adafruit_SSD1306 oled(128, 64, &Wire, -1);
static bool oled_ok = false;
volatile bool actualizar_pantalla = true;   // declarado extern en funciones.h
volatile bool cambio_pendiente = false;     // declarado extern en funciones.h
static volatile unsigned long t_ultimo_giro = 0;

// 3. Encoder ISR
IRAM_ATTR static void isr_encoder() {
    static unsigned long t_ant = 0;
    unsigned long t_act = millis();
    unsigned long dt = t_act - t_ant;

    if (dt > 10) {
        int delta = (dt < 40) ? 5 : 1;

        if (digitalRead(PIN_DT) == HIGH) {
            fa.armonicos_sel = min(MAX_ARMONICOS, fa.armonicos_sel + delta);
        } else {
            fa.armonicos_sel = max(1, fa.armonicos_sel - delta);
        }

        cambio_pendiente = (fa.armonicos_sel != fa.armonicos_act);
        t_ultimo_giro = t_act;
        actualizar_pantalla = true;
        t_ant = t_act;
    }
}

void verificar_auto_reconstruccion() {
    if (cambio_pendiente && (millis() - t_ultimo_giro > TIMEOUT_AUTO_MS)) {
        fa.armonicos_act = fa.armonicos_sel;
        cambio_pendiente = false;
        actualizar_pantalla = true;
        calcular_metricas_y_reportar(fa.armonicos_act);
    }
}

// 5. Osciloscopio con Auto-Escalado Dinámico y Alta Velocidad
static void dibujar_osciloscopio() {
    const int x_inicio = 1;
    const int x_fin = 126;
    const int y_min = 13;
    const int y_max = 63;
    const int alto_util = y_max - y_min - 2; // ~48 px útiles

    // Marco
    oled.drawRect(0, y_min, 128, (y_max - y_min + 1), SSD1306_WHITE);

    // Detección robusta: si el nombre contiene "RM" es un rectificador (unipolar)
    bool es_rectificador = (strstr(fa.tipo, "RM") != NULL || strstr(fa.tipo, "RT") != NULL);
    bool es_rtmo = (strcmp(fa.tipo, "RTMO") == 0);

    int y_cero;
    float ganancia;

    if (es_rtmo) {
        // RTMO = max(sin(theta), sin(theta - 120 deg), sin(theta - 240 deg)).
        // Su rango teorico normalizado es 0.5 <= v/Vp <= 1.0.
        y_cero = y_max - 1;
        ganancia = (float)(alto_util - 2) / 0.5f;
        for (int x = 2; x < 126; x += 4) oled.drawPixel(x, y_cero, SSD1306_WHITE);
    } else if (es_rectificador) {
        // Modo Rectificador: 0 V abajo de todo
        y_cero = y_max - 2;
        ganancia = (float)alto_util; // 0 V a 1.0 Vp ocupa los 48 px completos
        // Línea punteada de 0 V en el piso
        for (int x = 2; x < 126; x += 4) oled.drawPixel(x, y_cero, SSD1306_WHITE);
    } else {
        // Modo Bipolar (AC): 0 V al centro exacto
        y_cero = y_min + (alto_util / 2) + 1;
        ganancia = (float)(alto_util / 2);
        // Eje central punteado
        for (int x = 2; x < 126; x += 4) oled.drawPixel(x, y_cero, SSD1306_WHITE);
    }

    int x_ant = x_inicio;
    int y_ant = y_cero;

    for (int x = x_inicio; x <= x_fin; x++) {
        // Posición del pixel en la tabla: k = (x - x_inicio) * M / (x_fin - x_inicio).
        // Con M = 1000 y 125 tramos da k = 8*(x - x_inicio): entero exacto, sin interpolar.
        const int k = ((x - x_inicio) * MAX_PUNTOS) / (x_fin - x_inicio);
        float valor = fa.a0;

        int idx = 0;   // (n*k) mod M, incremental (ver fourier.cpp)
        for (int n = 1; n <= fa.armonicos_act; n++) {
            idx += k;
            if (idx >= MAX_PUNTOS) idx -= MAX_PUNTOS;
            valor += (fa.an[n] * tabla_cos[idx]) + (fa.bn[n] * tabla_sin[idx]);
        }

        int y_act;
        if (es_rtmo) {
            // La escala respeta el rango teorico de RTMO, no el rango de la
            // reconstruccion truncada, que puede tener pequenos sobrepicos.
            y_act = y_cero - (int)((valor - 0.5f) * ganancia);
        } else {
            y_act = y_cero - (int)(valor * ganancia);
        }
        y_act = constrain(y_act, y_min + 1, y_max - 1);

        if (x > x_inicio) {
            oled.drawLine(x_ant, y_ant, x, y_act, SSD1306_WHITE);
        }
        x_ant = x;
        y_ant = y_act;
    }
}

void init_hardware() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000); // 400 kHz Fast I2C para máxima fluidez de pantalla
    delay(100);

    if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oled_ok = true;
        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextSize(1);
        oled.setCursor(18, 10);
        oled.print("SISTEMA LISTO");
        oled.drawFastHLine(0, 24, 128, SSD1306_WHITE);
        oled.setCursor(20, 36);
        oled.print("ESPERANDO PC...");
        oled.display();
    }

    pinMode(PIN_CLK, INPUT_PULLUP);
    pinMode(PIN_DT, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_CLK), isr_encoder, FALLING);
}

// Durante un OTA se escribe la flash: no queremos interrupciones del encoder.
void display_suspender_encoder() {
    detachInterrupt(digitalPinToInterrupt(PIN_CLK));
}

void display_reanudar_encoder() {
    attachInterrupt(digitalPinToInterrupt(PIN_CLK), isr_encoder, FALLING);
}

void render_pantalla() {
    if (!actualizar_pantalla || !oled_ok) return;
    actualizar_pantalla = false;

    if (!fa.datos_listos) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    // 1. Encabezado superior completo
    oled.setTextSize(1);
    oled.setCursor(0, 1);
    oled.printf("%s", fa.tipo);

    oled.setCursor(76, 1);
    if (cambio_pendiente) {
        oled.printf("N>[%d]", fa.armonicos_sel);
    } else {
        oled.printf("N=%d", fa.armonicos_act);
    }

    // 2. Gráfico del osciloscopio
    dibujar_osciloscopio();
    oled.display();
}