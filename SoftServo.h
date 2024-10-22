#ifndef __SOFT_SERVO_H__
#define __SOFT_SERVO_H__

#include <Arduino.h>
#include <SoftPWM.h>

class SoftServo {

public:
  SoftServo();                   // Costruttore della classe SoftServo
  void begin(uint8_t pin);      // Inizializza il servo su un pin specifico
  void attach(uint8_t pin);     // Collega il servo a un pin specifico
  void write(uint8_t angle);    // Imposta l'angolo del servo

private:
  uint8_t pin;                  // Pin a cui è collegato il servo
};

#endif // __SOFT_SERVO_H__
