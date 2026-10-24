# TP Armónicos – ESP32 + app Python

Analizador de armónicos y control de potencia sobre ESP32. La app de PC genera (o recibe)
los vectores de tensión y corriente, el ESP32 calcula todo con Fourier discreto y muestra
la forma de onda en un OLED; además sincroniza con el cruce por cero (ZCD) y maneja
rectificadores controlados o un inversor con puente H.

## Contenido
- `src/`, `include/`, `platformio.ini`: firmware (PlatformIO, Arduino, ESP32).
- `app_python/enviar.py`: app de PC (Tkinter + matplotlib + pyserial).

## Qué calcula (consigna)
- Vectores de n muestras, **n entre 120 y 720** (inicial 360), Vin = 220 V (311,13 V pico), R = 100 Ω, i = v/R.
- Señales: senoidal, cuadrada, cuasicuadrada, RMMOC (α), RMOCC (α), RTMO y **7. ADC** (medición real).
- MIN, AVG, RMS, MAX de tensión y corriente; S, P, Q, D; FP; **THD V y THD I**
  (THD = √(Σ_{h≥2}(a_h²+b_h²)) / √(a₁²+b₁²), Hart ec. 6.8).
- La app compara contra el valor teórico y muestra el error de cada magnitud.
- Cantidad de armónicos N ajustable desde la app o con el encoder (máx. n/2).

## Control (ZCD)
- Detector de cruce por cero por interrupción (GPIO 4): período, frecuencia y aviso si no hay cruce.
- **Rectificadores** RMMOC y RMOCC: disparo a α grados del cruce con timer de hardware,
  ancho de pulso y compensación configurables (GPIO 16 y 17).
- **Inversor con puente H**: onda cuadrada (inmediata, con tiempo muerto) y cuasicuadrada
  (30° después del cruce); sin red genera 50 Hz en forma desincronizada (GPIO 25 y 26).
- Comunicación por **USB (serie)** o **WiFi (TCP, puerto 5000)**; actualización por **OTA**.

## ADC (señal real)
- `src/adc.cpp`: al elegir **7. ADC** y presionar *CAPTURAR ADC*, el ESP32 espera el próximo cruce
  por cero (GPIO 4) y toma n muestras de tensión (GPIO 34) y n de corriente (GPIO 35) repartidas
  en un período (instante k = t_cruce + k·T/n). Devuelve las muestras (se grafican) y las métricas.
- Escala: la señal debe llegar a 0..3,3 V con el cero en 1,65 V. v = (2·raw/4095 − 1)·Vmax;
  i = (2·raw/4095 − 1)·Vmax/R. Con el generador de onda cuadrada 0/3,3 V se obtiene ±Vmax.
- Sin ZCD hace una captura **libre** (período nominal de 20 ms, no sincronizada) y lo avisa con
  `ADC_LIBRE`: sirve para probar el cableado (una entrada tocada capta el zumbido de 50 Hz).
  Si el ZCD se pierde durante la espera informa `ADC_SIN_ZCD`. Usa solo ADC1 (compatible con WiFi).
- Prueba del paso 1 de la consigna: generador de 50 Hz a GPIO 4, GPIO 34 y GPIO 35.

## Cómo usarlo
1. Copiar `include/secrets.h.example` como `include/secrets.h` y completar WiFi y clave OTA.
2. Compilar y subir con PlatformIO (`pio run -e esp32dev -t upload`).
3. `pip install pyserial matplotlib` y correr `python app_python/enviar.py`.
   La app se conecta sola por USB; para WiFi se escribe la IP del ESP32 (la imprime en el
   monitor serie: `WIFI_READY,<IP>`) y se presiona *WIFI*.
4. Elegir la señal, n, R y N. Para disparar: elegir RMMOC/RMOCC (rectificador) o
   Cuadrada/Cuasicuadrada (inversor) y presionar *ARMAR*.

Pines: encoder CLK=18, DT=5; OLED SDA=21, SCL=22; ZCD=4; gates 16/17; puente H 25/26; ADC V=34, I=35.
Protocolo (texto, una línea): `tipo,vmax,freq,Narm,n,v[0..n-1],i[0..n-1]` hacia el ESP32;
`DATA_METRICAS,...` y `ZCD_ESTADO,...` de vuelta; comandos `CTRL,...`; pedido de ADC `ADC,vmax,R,Narm,n`
(respuesta `ADC_DATA,n,v...,i...`).

## Estado y limitaciones
- El ADC está implementado y verificado por simulación en PC (lógica de muestreo y análisis);
  **no se probó todavía en la placa**. El ADC del ESP32 no es lineal en los extremos y cada
  muestra necesita dos conversiones: con n grande puede no alcanzar el tiempo (el ESP32 informa
  en `ADC_INFO` cuántas muestras llegaron tarde; con n ≤ 360 sobra margen).
- **No implementada** la resincronización suave del inversor (variar frecuencia y poner en fase
  lentamente tras un corte); al volver la red se sincroniza en el primer cruce válido.
- Los tiempos de disparo del rectificador y del inversor se verificaron por simulación en PC;
  la compensación (retraso del optoacoplador) debe ajustarse midiendo en el circuito real.
