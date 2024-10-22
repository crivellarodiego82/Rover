#include "Ultrasuoni.h"
#include <Arduino.h>

float ultrasonicRead() {
  delay(4); // È richiesto un ritardo tra letture consecutive, altrimenti la lettura potrebbe essere 0

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // noInterrupts(); // Metti in pausa tutte le interruzioni per evitare di influenzare i dati
                  // Tuttavia, disattivare le interruzioni influisce sulla funzionalità di softpwm, come i motori

  // float duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  // restituirà 0 durante il timeout
  float duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_READ_TIMEOUT);
 
  // interrupts(); // ripristina le interruzioni
 
  float distance = duration * 0.017; // S = vt = 340m/s * (t/2)us= (340 * 100 cm/s) * 0.5 * (t / 10^6)s = 0.017 * t
  

  if (distance > MAX_DISTANCE || distance == 0) {
    return -1;
  } 
  return distance;
}

bool ultrasonicIsObstacle() {
  return ultrasonicRead() < ULTRASONIC_AVOIDANCE_THRESHOLD;
}

bool ultrasonicIsClear() {
  float distance = ultrasonicRead();
  if (distance > ULTRASONIC_AVOIDANCE_THRESHOLD || distance < 0) {
    return true;
  } else {
    return false;
  }
}
