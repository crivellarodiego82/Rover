#include "Ir_Ostacoli.h"

/** 
 * @brief Inizializza il modulo dell'ostacolo IR
 */
void irObstacleBegin() {
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
}

/** 
 * @brief Legge i valori dei sensori IR
 * 
 * @return byte Valore combinato dei sensori sinistro e destro
 */
byte irObstacleRead() {
  byte left = digitalRead(IR_LEFT_PIN);
  byte right = digitalRead(IR_RIGHT_PIN);
  return (left << 1) | right; // Combina i valori dei sensori
}
