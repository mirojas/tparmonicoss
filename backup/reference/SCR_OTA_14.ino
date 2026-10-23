#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "EDP-TESTING";
const char* password = "Potencia26";

const int pinZCD = 4;
const int pinDisparo = 16;
const int pinPot = 36; // Pin analógico para el potenciómetro (ADC1)

volatile unsigned long tiempoUltimoCruce = 0;
volatile unsigned long periodoCicloCompleto = 20000; 
volatile bool cruceDetectado = false;

int anguloDisparo = 90; 
const unsigned long anchoPulso_us = 50;
int adelanto = 100; // compensar retraso en el disparo en us

// Manejador para la tarea de FreeRTOS
TaskHandle_t ManejadorOTA;

void IRAM_ATTR isrCrucePorCero() {
  unsigned long tiempoActual = micros();
  unsigned long tiempoMedido = tiempoActual - tiempoUltimoCruce;
  
  if (tiempoMedido > 12000) { 
    periodoCicloCompleto = tiempoMedido;
    tiempoUltimoCruce = tiempoActual;
    cruceDetectado = true;
  }
}

// --- TAREA ASIGNADA AL NÚCLEO 0 ---
// Esta función correrá en paralelo, independiente del loop()
void tareaManejoOTA(void * parameter) {
  for(;;) {
    ArduinoOTA.handle();
    
    // IMPORTANTE: En FreeRTOS, los bucles infinitos deben ceder tiempo de 
    // procesamiento para no disparar el Watchdog Timer (reinicio por pánico).
    // Un retardo de 10 milisegundos es perfecto para que el WiFi respire.
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}


void setup() {
  Serial.begin(115200);
  
  pinMode(pinDisparo, OUTPUT);
  digitalWrite(pinDisparo, LOW);
  pinMode(pinZCD, INPUT); 

  // La resolución por defecto del ESP32 es de 12 bits (0-4095)
  analogReadResolution(12);
    
  attachInterrupt(digitalPinToInterrupt(pinZCD), isrCrucePorCero, FALLING);
  Serial.println("RMMOC Dual-Core Iniciado. Conectando...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Error WiFi! Reiniciando...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname("esp32-rmmoc"); 
  ArduinoOTA.setPassword("edptest"); 

  //ArduinoOTA.onStart([]() { digitalWrite(pinDisparo, LOW); });
  ArduinoOTA.onStart([]() { 
    // 1. Desvinculamos el pin ZCD para que no interrumpa el flasheo
    detachInterrupt(digitalPinToInterrupt(pinZCD)); 

    // 2. Apagamos por seguridad la etapa de potencia
    digitalWrite(pinDisparo, LOW); 

    Serial.println("Iniciando OTA, interrupciones suspendidas..."); 
  });
  ArduinoOTA.begin();
  
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // --- CREACIÓN DE LA TAREA EN EL NÚCLEO 0 ---
  xTaskCreatePinnedToCore(
    tareaManejoOTA,   // Función que contiene el código de la tarea
    "Tarea_OTA",      // Nombre descriptivo
    10000,            // Tamaño de la pila (Stack size)
    NULL,             // Parámetros pasados a la función
    1,                // Prioridad de la tarea (1 es baja/normal)
    &ManejadorOTA,    // Manejador de la tarea
    0                 // NÚCLEO AL QUE SE ASIGNA (0 = PRO_CPU)
  );
}

void loop() {
  // --- NÚCLEO 1: EXCLUSIVO PARA ELECTRÓNICA DE POTENCIA ---
  // Ahora el loop() puede iterar a máxima velocidad sin interrupciones de red.
  // 1. Lectura del potenciómetro fuera de la zona crítica de tiempo
  // Solo actualizamos el ángulo si NO estamos en medio de la espera del disparo.
  if (!cruceDetectado) {
    int lecturaADC = analogRead(pinPot);
    
    // Mapeamos el valor del ADC (0-4095) a grados (0-180).
    // Nota de diseño: Se limita el rango entre 5° y 175° para evitar 
    // solapamientos con el propio ancho del pulso de cruce por cero
    // y asegurar que el SCR siempre tenga margen para conmutar.
    anguloDisparo = map(lecturaADC, 0, 4095, 5, 175);
  }
  // 2. Ejecución del disparo  
  if (cruceDetectado) {
    unsigned long retardoDisparo_us = ((periodoCicloCompleto * anguloDisparo) / 360)-adelanto;
    unsigned long tiempoActual = micros();
    
    if (tiempoActual - tiempoUltimoCruce >= retardoDisparo_us) {
      
      digitalWrite(pinDisparo, HIGH);
      delayMicroseconds(anchoPulso_us);
      digitalWrite(pinDisparo, LOW);
      
      cruceDetectado = false; 
    }
  }
}
