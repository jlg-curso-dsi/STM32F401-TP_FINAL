# STM32F401-TP_FINAL
Trabajo final - Curso Cortex-M4 - DSI - FCEIA - UNR

15-11-2015:
- Se definieron las macros Y_UMBRAL, TICK_60_SEG, TICK_1_SEG, SHORT_TONE_DURATION
  y LONG_TONE_DURATION al comienzo de application.c para organizar mejor el codigo.
- En lugar de contar la cantidad de buffers que esta presente el tono debo calcular su duracion en ms,
  para independizar la operacion de la longitud del buffer. Para ello en la linea 138 obtengo ToneDuration
  usando la cant. de buffers, la cant. de muestras por buffer, y la cantidad de muestras por segundo. 
- En la linea 147 agrego la comparacion con la duracion del tono largo para producir la sincronizacion
  del temporizador de 1 minuto.
- La cantidad de muestras por segundo es obtenida en la linea 299 como la cant. de muestras por
  segundo de un canal  por la cant. de canales, y el valor es guardado en SampleNbr para ser usado
  luego dentro de getDataCB.
