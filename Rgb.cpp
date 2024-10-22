#include "Rgb.h"

#if defined(ARDUINO_AVR_UNO)
#include <SoftPWM.h>
#endif

/** 
 * @brief Inizializza il modulo RGB 
 */
void rgbBegin() {
  #if defined(ARDUINO_AVR_UNO)
  for (uint8_t i = 0; i < 3; i++) {
    SoftPWMSet(RGB_PINS[i], 0);
    SoftPWMSetFadeTime(RGB_PINS[i], 100, 100);
  }
  #endif
}

/** 
 * @brief Scrive un colore RGB 
 * 
 * @param color Il colore da scrivere (in formato uint32_t)
 */
void rgbWrite(uint32_t color) {
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >>  8) & 0xFF;
  uint8_t b = (color >>  0) & 0xFF;
  rgbWrite(r, g, b);
}

/** 
 * @brief Scrive un colore RGB specificando i valori dei singoli colori
 * 
 * @param r Valore del colore rosso
 * @param g Valore del colore verde
 * @param b Valore del colore blu
 */
void rgbWrite(uint8_t r, uint8_t g, uint8_t b) {
  // calibra la luminosità
  r = int(r * R_OFFSET);
  g = int(g * G_OFFSET);
  b = int(b * B_OFFSET);
  
  // inversione per anodo comune
  #if COMMON_ANODE
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  #endif

  // imposta la tensione 
  #if defined(ARDUINO_AVR_UNO)
  SoftPWMSet(RGB_PINS[0], r);
  SoftPWMSet(RGB_PINS[1], g);
  SoftPWMSet(RGB_PINS[2], b);
  #elif defined(ARDUINO_MINIMA)
  analogWrite(RGB_PINS[0], r);
  analogWrite(RGB_PINS[1], g);
  analogWrite(RGB_PINS[2], b);
  #endif
}

/** 
 * @brief Spegne il LED RGB 
 */
void rgbOff() {
  rgbWrite(0, 0, 0);
}
