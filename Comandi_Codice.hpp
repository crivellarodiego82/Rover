#ifndef __CMD_CODE_CONFIG_H__
#define __CMD_CODE_CONFIG_H__

#include <Arduino.h>
#include "Rgb.h"
#include "Controllo_Auto.h"

/** Definizione dei valori di modalità */
#define MODE_NONE                    0
#define MODE_OBSTACLE_FOLLOWING      1
#define MODE_OBSTACLE_AVOIDANCE      2
#define MODE_APP_CONTROL             3
#define MODE_VOICE_CONTROL           4
#define MODE_DISCONNECT              5

/** Imposta il colore per corrispondere alla modalità */
#define ERROR_COLOR RED
#define WARN_COLOR ORANGE

#define MODE_DISCONNECT_COLOR 0xFFFFFF
#define MODE_NONE_COLOR 0xFFFFFF // bianco

#define MODE_APP_CONTROL_COLOR              0xFFFFFF

#define MODE_OBSTACLE_FOLLOWING_COLOR       BLUE
#define MODE_OBSTACLE_AVOIDANCE_COLOR       PURPLE
 
#define MODE_VOICE_CONTROL_COLOR       CYAN


// Macchina a stati per quasi tutte le modalità. Definizioni degli stati, vedere ogni funzione
uint8_t currentMode = MODE_NONE;

int8_t leftMotorDir = 0;
int8_t rightMotorDir = 0;

bool remoteDriftEnable  = false;

#define CMD_SUM 5

// Prestare attenzione all'ordine,
// ad esempio: 'sinistra avanti' deve essere posizionato prima di 'sinistra' e 'avanti'
const char cmd_str_0[] PROGMEM = "stop";
const char cmd_str_1[] PROGMEM = "forward";
const char cmd_str_2[] PROGMEM = "backward";
const char cmd_str_3[] PROGMEM = "turn left";
const char cmd_str_4[] PROGMEM = "turn right";

const char *const cmd_str_table[] PROGMEM = {
  cmd_str_0, cmd_str_1, cmd_str_2, cmd_str_3,
  cmd_str_4,
};

int32_t voice_action_time[] = { // uint: ms
  1,
  -1, // Infinito
  -1,
  3000,
  3000,
};

void stop(int8_t power){
  currentMode = MODE_NONE;
  carStop();
}

void (*cmd_fuc_table [])(int8_t power) { 
  stop,
  carForward,
  carBackward,
  carTurnLeft,
  carTurnRight,
};

int8_t text_2_cmd_code(char* text){
  String str = String(text);
  char buffer[20];
  for(uint8_t i=0; i<CMD_SUM; i++){
    strcpy_P(buffer, (char *)pgm_read_word(&cmd_str_table[i]));
    if(str.indexOf(buffer) != -1){
      // Serial.print(i);
      // Serial.print(" , ");
      // Serial.println(buffer);
      return i;
    }
  }
  return -1; // codice di errore
}

void voice_action(int8_t cmd_code, int8_t power) {
    cmd_fuc_table[cmd_code](power);
}

#endif
