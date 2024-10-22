#ifndef _TEST_H_
#define _TEST_H_

#include <Arduino.h>

#include "Controllo_Auto.h"
#include "Ir_Ostacoli.h"
#include "Ultrasuoni.h"
#include "Rgb.h"

/** Test dei motori, l'auto si muove avanti e indietro */
void motors_test(){
  carForward(80);
  rgbWrite(0, 255, 0); // Verde
  delay(2000);

  carBackward(80);
  rgbWrite(255, 0, 0); // Rosso
  delay(2000);

  carTurnLeft(80);
  rgbWrite(0, 0, 255); // Blu
  delay(2000);

  carTurnRight(80);
  rgbWrite(255, 255, 0); // Giallo
  delay(2000);
}

/** Test dei LED RGB, trasformazione ciclica dei colori R,G,B */
void rgb_test() {
  rgbWrite(RED);
  delay(1000);
  rgbWrite(GREEN);
  delay(1000);
  rgbWrite(BLUE);
  delay(1000); 
}

/** Test ultrasonico, stampa ciclica dei dati rilevati */
void ultrasonic_test() {
  rgbWrite(BLUE);
  float distance = ultrasonicRead();
  Serial.print("distanza: ");
  Serial.println(distance);
  delay(100);
  rgbOff(); 
  delay(500);
}

/** Test del modulo di evitamento ostacoli a infrarossi, stampa ciclica dei dati rilevati */
void ir_obstacle_test() {
  // uint16_t result = hc165Read();
  // Serial.println(result, BIN);
  byte result = irObstacleRead();
  bool leftIsClear = result & 0b00000001;
  bool rightIsClear = result & 0b00000010;
  Serial.print(leftIsClear);
  Serial.print(", ");
  Serial.println(rightIsClear);
  delay(100);
}

#endif // TEST_H_INCLUDED
