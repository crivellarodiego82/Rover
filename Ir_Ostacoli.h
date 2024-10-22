#ifndef __IR_OBSTACLE_H__
#define __IR_OBSTACLE_H__

#include "Arduino.h"

#define IR_LEFT_PIN 8
#define IR_RIGHT_PIN 7

/**
 * @brief Inizializza il modulo dell'ostacolo IR
 * 
 */
void irObstacleBegin();

/**
 * @brief Restituisce lo stato del livello IO del modulo di evitamento ostacoli a infrarossi
 *        collegato con il chip di espansione IO HC165.
 *        In realtà, è la funzione hc165Read()
 * 
 * @return byte 
 */
byte irObstacleRead();

#endif // __IR_OBSTACLE_H__
