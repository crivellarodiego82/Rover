#ifndef __RGB_H__
#define __RGB_H__

#include <Arduino.h>

/** 
 * Indica se i LED RGB sono a catodo comune o meno 
 * - 0 catodo comune
 * - 1 anodo comune
 */
#define COMMON_ANODE 0

/** Imposta i pin dei LED RGB, l'ordine è R, G, B */
#define RGB_PINS (uint8_t[3]){12, 13, 11}

/**
 * @name Definisce il valore esadecimale del colore
 */
#define RED           0xFF0202
#define ORANGE        0xFFA500
#define YELLOW        0xFFFF0A
#define YELLOW_GREEN  0xA5FF0A
#define GREEN         0x0AFF0A
#define GREEN_CYAN    0x0AFFA5
#define CYAN          0x0AFFFF
#define CYAN_BLUE     0x0AA5FF
#define BLUE          0x0A0AFF
#define PURPLE        0xA50AFF
#define VOILET        0xFF0AFF
#define MAGENTA       0xFF0AA5

/* Calibra la luminosità */
#define R_OFFSET  1.0
#define G_OFFSET  0.25 //0.16
#define B_OFFSET  0.45 //0.30

/** Inizializza i LED RGB */
void rgbBegin();

/** Imposta il colore del LED in formato esadecimale */
void rgbWrite(uint32_t color);

/** Imposta il colore del LED in formato R, G, B a 8 bit (0 ~ 255) */
void rgbWrite(uint8_t r, uint8_t g, uint8_t b);

/** Spegne i LED RGB */
void rgbOff();

#endif // __RGB_H__
