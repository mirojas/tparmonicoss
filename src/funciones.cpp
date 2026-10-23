// 2. Síntesis y cálculo de métricas eléctricas
/* Segun los armonicos:
V_avg: Es directamente la componente continua a_0 * V_max
V_rms (Teorema de Parseval / Ley de Ortogonalidad): 
    Suma cuadrática de los valores eficaces individuales de cada armónico sintetizado:
        $$V_{rms}(N) = V_{max} * \sqrt{a_0^2 + \sum_{n=1}^{N} \frac{a_n^2 + b_n^2}{2}}$$
    Segun sus formulas y funciones:
Señal                            | Vavg Teórico                 | Vrms Teórico                                              | Vmax Teórico | Vmin Teórico
---------------------------------+------------------------------+-----------------------------------------------------------+--------------+-------------
Senoidal Pura                    | 0 V                          | Vp / sqrt(2) ≈ 0.7071 Vp                                  | +Vp          | -Vp
Cuadrada Simétrica               | 0 V                          | Vp = 1.0000 Vp                                            | +Vp          | -Vp
Cuasi Senoidal (120°)            | 0 V                          | Vp * sqrt(2/3) ≈ 0.8165 Vp                                | +Vp          | -Vp
RMMO (Media Onda)                | (Vp / pi) ≈ 0.3183 Vp        | Vp / 2 = 0.5000 Vp                                        | +Vp          | 0 V
RMOOC (Onda Completa, alpha=0)   | (2 * Vp / pi) ≈ 0.6366 Vp    | Vp / sqrt(2) ≈ 0.7071 Vp                                  | +Vp          | 0 V
RMOCC (Onda Completa Controlado) | (Vp / pi) * (1 + cos(alpha)) | (Vp / sqrt(2)) * sqrt((pi - alpha + sin(2*alpha)/2) / pi) | +Vp          | 0 V
*/
#include "funciones.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <string.h>

#define PIN_DT       5
#define PIN_CLK      18
#define PIN_SDA      21
#define PIN_SCL      22
#define OLED_ADDR    0x3C

static Adafruit_SSD1306 oled(128, 64, &Wire, -1);
static bool oled_ok = false;
static volatile bool actualizar_pantalla = true;
static volatile bool cambio_pendiente = false;
static volatile unsigned long t_ultimo_giro = 0;

static FourierAnalyzer fa = {};
static char rx_buffer[24000];

static const char* WIFI_SSID = "EDP-TESTING";
static const char* WIFI_PASSWORD = "Potencia26";
static const char* OTA_HOSTNAME = "esp32-potencia";
static const char* OTA_PASSWORD = "edptest";

// 1. Correlación Ortogonal de Fourier Discreta
static void calcular_coeficientes(const float* vector, float& a0, float* an, float* bn) {
    const int M = MAX_PUNTOS;
    const float factor_dos_m = 2.0f / (float)M;
    const float paso_rad = (2.0f * PI) / (float)M;

    float suma_dc = 0.0f;
    for (int k = 0; k < M; k++) {
        suma_dc += vector[k];
    }
    a0 = suma_dc / (float)M;

    for (int n = 1; n <= MAX_ARMONICOS; n++) {
        float suma_cos = 0.0f;
        float suma_sin = 0.0f;
        float factor_n = (float)n * paso_rad;

        for (int k = 0; k < M; k++) {
            float angulo = factor_n * (float)k;
            suma_cos += vector[k] * cosf(angulo);
            suma_sin += vector[k] * sinf(angulo);
        }

        an[n] = factor_dos_m * suma_cos;
        bn[n] = factor_dos_m * suma_sin;
    }
}

static void analizar_fourier() {
    calcular_coeficientes(fa.vector_in, fa.a0, fa.an, fa.bn);
    calcular_coeficientes(fa.corriente_in, fa.a0_i, fa.an_i, fa.bn_i);
}

// 2. Síntesis y cálculo de métricas
static void calcular_metricas_y_reportar(int n_arm) {
    const int M = MAX_PUNTOS;
    float v_max = -9999.0f;
    float v_min = 9999.0f;
    float i_max = -9999.0f;
    float i_min = 9999.0f;
    float suma_potencia_activa = 0.0f;
    float suma_potencia_reactiva = 0.0f;

    for (int k = 0; k < M; k++) {
        float theta = (2.0f * PI * (float)k) / (float)M;
        float val_norm = fa.a0;
        float val_corriente = fa.a0_i;

        for (int n = 1; n <= n_arm; n++) {
            val_norm += (fa.an[n] * cosf((float)n * theta)) + (fa.bn[n] * sinf((float)n * theta));
            val_corriente += (fa.an_i[n] * cosf((float)n * theta)) + (fa.bn_i[n] * sinf((float)n * theta));
        }

        float v_inst = val_norm * fa.vmax;
        float i_inst = val_corriente;
        if (v_inst > v_max) v_max = v_inst;
        if (v_inst < v_min) v_min = v_inst;
        if (i_inst > i_max) i_max = i_inst;
        if (i_inst < i_min) i_min = i_inst;
        suma_potencia_activa += v_inst * i_inst;
    }

    fa.v_avg_calc = fa.a0 * fa.vmax;

    float suma_potencia_ac = 0.0f;
    for (int n = 1; n <= n_arm; n++) {
        suma_potencia_ac += (fa.an[n] * fa.an[n] + fa.bn[n] * fa.bn[n]) / 2.0f;
    }
    fa.v_rms_calc = fa.vmax * sqrtf((fa.a0 * fa.a0) + suma_potencia_ac);
    fa.v_max_calc = v_max;
    fa.v_min_calc = v_min;
    fa.i_max_calc = i_max;
    fa.i_min_calc = i_min;

    float suma_corriente_ac = 0.0f;
    for (int n = 1; n <= n_arm; n++) {
        suma_corriente_ac += (fa.an_i[n] * fa.an_i[n] + fa.bn_i[n] * fa.bn_i[n]) / 2.0f;
        suma_potencia_reactiva +=
            ((fa.vmax * fa.bn[n]) * fa.an_i[n] - (fa.vmax * fa.an[n]) * fa.bn_i[n]) / 2.0f;
    }

    fa.i_avg_calc = fa.a0_i;
    fa.i_rms_calc = sqrtf((fa.a0_i * fa.a0_i) + suma_corriente_ac);
    fa.potencia_s = fa.v_rms_calc * fa.i_rms_calc;
    fa.potencia_p = suma_potencia_activa / (float)M;
    fa.potencia_q = suma_potencia_reactiva;
    float potencia_d_cuadrado = (fa.potencia_s * fa.potencia_s) -
                                (fa.potencia_p * fa.potencia_p) -
                                (fa.potencia_q * fa.potencia_q);
    fa.potencia_d = sqrtf(max(0.0f, potencia_d_cuadrado));
    fa.factor_potencia = (fa.potencia_s > 0.0f) ? fa.potencia_p / fa.potencia_s : 0.0f;

    Serial.printf("DATA_METRICAS,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                  n_arm, fa.v_max_calc, fa.v_min_calc, fa.v_avg_calc, fa.v_rms_calc,
                  fa.i_max_calc, fa.i_min_calc, fa.i_avg_calc, fa.i_rms_calc,
                  fa.potencia_s, fa.potencia_p, fa.potencia_q, fa.potencia_d,
                  fa.factor_potencia);
}

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

// 4. Recepción UART sin bloqueo de buffer
void recibir_datos_serial() {
    if (Serial.available() > 0) {
        size_t bytes_leidos = Serial.readBytesUntil('\n', rx_buffer, sizeof(rx_buffer) - 1);
        rx_buffer[bytes_leidos] = '\0';

        if (bytes_leidos > 10) {
            char* token = strtok(rx_buffer, ",");
            if (token != NULL) strncpy(fa.tipo, token, sizeof(fa.tipo) - 1);

            token = strtok(NULL, ",");
            if (token != NULL) fa.vmax = atof(token);

            token = strtok(NULL, ",");
            if (token != NULL) strncpy(fa.freq, token, sizeof(fa.freq) - 1);

            token = strtok(NULL, ",");
            if (token != NULL) {
                fa.armonicos_act = constrain(atoi(token), 1, MAX_ARMONICOS);
                fa.armonicos_sel = fa.armonicos_act;
            }

            int idx = 0;
            token = strtok(NULL, ",");
            while (token != NULL && idx < (2 * MAX_PUNTOS)) {
                if (idx < MAX_PUNTOS) {
                    fa.vector_in[idx] = atof(token);
                } else {
                    fa.corriente_in[idx - MAX_PUNTOS] = atof(token);
                }
                idx++;
                token = strtok(NULL, ",");
            }

            if (idx == (2 * MAX_PUNTOS)) {
                fa.datos_listos = true;
                
                // Conserva N armónicos actuales
                int n_mantener = (fa.armonicos_act >= 1) ? fa.armonicos_act : 1;
                fa.armonicos_sel = n_mantener;
                fa.armonicos_act = n_mantener;
                
                analizar_fourier();
                calcular_metricas_y_reportar(n_mantener);
                
                cambio_pendiente = false;
                actualizar_pantalla = true;
            }
        }
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
        float theta = (2.0f * PI * (float)(x - x_inicio)) / (float)(x_fin - x_inicio);
        float valor = fa.a0;

        for (int n = 1; n <= fa.armonicos_act; n++) {
            valor += (fa.an[n] * cosf((float)n * theta)) + (fa.bn[n] * sinf((float)n * theta));
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