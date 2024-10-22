#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <Arduino.h>

#define BATTERY_PIN A3

/**
 * @brief Inizializza il modulo batteria
 * 
 */
void batteryBegin() {
  pinMode(BATTERY_PIN, INPUT);
}

/**
 * @brief Restituisce la tensione della batteria
 * 
 * @return float 
 */
float batteryGetVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float adcVoltage = adcValue / 1024.0 * 5 * 2;
  float batteryVoltage = int(adcVoltage * 100) / 100.0; // arrotonda a due decimali
  return batteryVoltage;
}

/**
 * @brief Restituisce la percentuale della batteria
 * 
 * @return uint8_t
 */
uint8_t batteryGetPercentage() {
  float voltage = batteryGetVoltage();
  uint8_t percentage = map(voltage, 6.6, 8.4, 0, 100);
  return percentage;
}

#endif // __BATTERY_H__
