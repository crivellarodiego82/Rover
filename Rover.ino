/*******************************************************************
Crivellaro Rover 2024
********************************************************************/
#include <Arduino.h>
#include <SoftPWM.h>
#include <string.h>

#include "Rgb.h"
#include "SoftServo.h"
#include "Controllo_Auto.h"
#include "Ir_Ostacoli.h"
#include "Ultrasuoni.h"
#include "Comandi_Codice.hpp"
#include "Batteria.h"
/*************************** Configurazione *******************************/
/** @name Configurazione 
 * 
 */
///@{
/** Se abilitare il Watchdog */
#define WATCH_DOG 0
#if WATCH_DOG
  #include <avr/wdt.h>
#endif

/** Se abilitare la modalità TEST */
#define TEST 0
#if TEST
  #include "test.h"
#endif

/** stampa della memoria utilizzata */
#define MEM 0
#if MEM
  
  #include <MemoryFree.h>
  #include <pgmStrToRAM.h> // non necessario per il nuovo metodo. Ma utile come riferimento.
#endif

/** Configurazione modalità Wi-Fi, SSID, password */
//#define WIFI_MODE WIFI_MODE_AP
//#define SSID "SSID"
//#define PASSWORD "Password"

#define WIFI_MODE WIFI_MODE_STA
#define SSID "SSID"
#define PASSWORD "Password" 

/** Nome Rover */
#define NAME "Rover_Gabriel"

/** Tipo */
#define TYPE "Rover_Gabriel"

/** Configurazione della porta WebSocket
 * L'applicazione Sunfounder Controller utilizza la porta 8765
*/
#define PORT "8999"

/** Configurazione della velocità dei motori in diverse modalità */
#define OBSTACLE_AVOID_POWER 80
#define OBSTACLE_FOLLOW_POWER 80
#define VOICE_CONTROL_POWER 80

/** Configurazione della distanza di follow per l'ostacolo */
#define FOLLOW_DISTANCE 20

/** Intestazioni di comunicazione WebSocket */
#define WS_HEADER "WS+"

///@}

/*********************** Variabili globali ****************************/
/** Classe per la comunicazione seriale con ESP32-CAM */
AiCamera aiCam = AiCamera(NAME, TYPE);

/* Configurazione del Servo della fotocamera */
SoftServo servo;

#define SERVO_PIN 6
#define SERVO_REVERSE false

/* Variabili per il controllo vocale */
char voice_buf_temp[20];
int8_t current_voice_code = -1;
int32_t voice_time = 0; // uint:s
uint32_t voice_start_time = 0; // uint:s

/* Variabili dei motori e del servo */
int8_t leftMotorPower = 0;
int8_t rightMotorPower = 0;
uint8_t servoAngle = 90;

/* Variabili del lampeggio RGB quando disconnesso */
uint32_t rgb_blink_interval = 500; // uint: ms
uint32_t rgb_blink_start_time = 0;
bool rgb_blink_flag = 0;

/* Variabile della lampada flash di ESP32-CAM */
bool cam_lamp_status = false;
//@}

/*********************** setup() & loop() ************************/
/**
 * setup(), punto di ingresso del programma principale di Arduino
 * 
 * Inizializzazione di alcune periferiche
 */
void setup() {
  int m = millis();
  Serial.begin(115200);
  Serial.print("Rover Gabriel Versione "); Serial.println(VERSION);

  Serial.println(F("Inizializzazione..."));
#if defined(ARDUINO_AVR_UNO)
  SoftPWMBegin(); // inizializzazione softpwm, prima dell'inizializzazione dei motori e dei LED RGB
#endif
  rgbBegin();
  rgbWrite(ORANGE); // indicazione di avvio
  carBegin();
  irObstacleBegin();
  batteryBegin();
  servo.attach(SERVO_PIN);
  servo.write(90);

#if !TEST
  aiCam.begin(SSID, PASSWORD, WIFI_MODE, PORT);
  aiCam.setOnReceived(onReceive);
#endif

  while (millis() - m < 500) {  // Attendere che le periferiche siano pronte
    delay(1);
  }

#if WATCH_DOG
  wdt_disable();       /* Disabilitare il watchdog e attendere più di 2 secondi */
  delay(3000);         /* Fatto per evitare che l'Arduino continui a resettarsi in caso di configurazione errata */
  wdt_enable(WDTO_2S); /* Abilitare il watchdog con un timeout di 2 secondi */
#endif

  Serial.println(F("Ok!"));
  rgbWrite(GREEN);  // inizializzazione terminata
}

/**
 * loop(), ciclo principale di Arduino
 * 
 * - include
 *  - aiCam.loop()
 *  - modeHandler()
 * - oppure test dei moduli
 */
void loop() {
#if !TEST
  // poiché il valore in aiCam è costantemente aggiornato
  // Notare che l'intervallo di ciclo di "aiCam.loop()" deve essere inferiore a 80ms per evitare perdite di dati
  aiCam.loop();
  if (aiCam.ws_connected == false) {
    currentMode = MODE_DISCONNECT;
    int8_t current_voice_code = -1;
    int8_t voice_time = 0;
    if (currentMode != MODE_DISCONNECT) {
      rgb_blink_start_time = 0;
      rgb_blink_flag = 1;
    }
  } else {
    if (currentMode == MODE_DISCONNECT) currentMode = MODE_NONE;
  }
  modeHandler();
#else
  /* Selezionare l'elemento da testare, consentita selezione multipla */
  motors_test();
  // rgb_test();
  // ultrasonic_test();
  // ir_obstacle_test();
  // obstacleAvoidance();
#endif

#if WATCH_DOG
  wdt_reset(); /* Resettare il watchdog */
#endif

#if MEM
  Serial.print(F("RAM libera = "));  //La funzione F fa lo stesso ed è ora una libreria integrata, in IDE > 1.0.0
  Serial.println(freeMemory());    // stampare quanta RAM è disponibile in byte.
#endif
}

/***************************** Funzioni ******************************/
/**
 * modeHandler(), esegue il programma corrispondente in base alla modalità impostata
 * 
 * - include
 *  - MODE_NONE
 *  - MODE_OBSTACLE_FOLLOWING
 *  - MODE_OBSTACLE_AVOIDANCE
 *  - MODE_REMOTE_CONTROL
 *  - MODE_APP_CONTROL
 */
void modeHandler() {
  switch (currentMode) {
    case MODE_NONE:
      rgbWrite(MODE_NONE_COLOR);
      carStop();
      servoAngle = 90;
      servo.write(servoAngle);
      break;
    case MODE_DISCONNECT:
      if (millis() - rgb_blink_start_time > rgb_blink_interval) {
        rgb_blink_flag = !rgb_blink_flag;
        rgb_blink_start_time = millis();
      }
      if (rgb_blink_flag) rgbWrite(MODE_DISCONNECT_COLOR);
      else rgbOff();
      carStop();
      servoAngle = 90;
      servo.write(servoAngle);
      break;
    case MODE_OBSTACLE_FOLLOWING:
      rgbWrite(MODE_OBSTACLE_FOLLOWING_COLOR);
      servo.write(servoAngle);
      obstacleFollowing();
      break;
    case MODE_OBSTACLE_AVOIDANCE:
      rgbWrite(MODE_OBSTACLE_AVOIDANCE_COLOR);
      servo.write(servoAngle);
      obstacleAvoidance();
      break;
    case MODE_APP_CONTROL:
      rgbWrite(MODE_APP_CONTROL_COLOR);
      servo.write(servoAngle);
      carSetMotors(leftMotorPower, rightMotorPower);
      break;
    case MODE_VOICE_CONTROL:
      rgbWrite(MODE_VOICE_CONTROL_COLOR);
      servo.write(servoAngle);
      voice_control();
      break;
    default:
      break;
  }
}

/**
 * Programma di follow dell'ostacolo
 */
void obstacleFollowing() {
  byte result = irObstacleRead();
  bool leftIsClear = result & 0b00000010;
  bool rightIsClear = result & 0b00000001;
  float usDistance = ultrasonicRead();
  // usDistance = -1 quando la distanza è troppo grande
  if (usDistance < 4 && usDistance > 0) {
    carStop();
  } else if (usDistance < 10 && usDistance > 0) {
    carForward(30);
  } else if (usDistance < FOLLOW_DISTANCE && usDistance > 0) {
    carForward(OBSTACLE_FOLLOW_POWER);
  } else {
    if (!leftIsClear) {
      carTurnLeft((int8_t)OBSTACLE_FOLLOW_POWER);
    } else if (!rightIsClear) {
      carTurnRight(OBSTACLE_FOLLOW_POWER);
    } else {
      carStop();
    }
  }
}

/**
 * Programma di evitamento ostacoli
 */
int8_t last_clear = -1;  // last_clear, 1, sinistra; -1, destra;
bool last_forward = false;

void obstacleAvoidance() {
  byte result = irObstacleRead();
  bool leftIsClear = result & 0b00000010;   // sinistra, libera: True
  bool rightIsClear = result & 0b00000001;  // destra, libera: True
  bool middleIsClear = ultrasonicIsClear();

  if (middleIsClear && leftIsClear && rightIsClear) {  // 111
    last_forward = true;
    carForward(OBSTACLE_AVOID_POWER);
  } else {
    if ((leftIsClear && rightIsClear) || (!leftIsClear && !rightIsClear)) {  // 101, 000, 010
      if (last_clear == 1) carTurnLeft(OBSTACLE_AVOID_POWER);
      else carTurnRight(OBSTACLE_AVOID_POWER);
      last_forward = false;
    } else if (leftIsClear) {  // 100, 110
      if (last_clear == 1 || last_forward == true) {
        carTurnLeft(OBSTACLE_AVOID_POWER);
        last_clear = 1;
        last_forward = false;
      }
    } else if (rightIsClear) {  // 001, 011
      if (last_clear == -1 || last_forward == true) {
        carTurnRight(OBSTACLE_AVOID_POWER);
        last_clear = -1;
        last_forward = false;
      }
    }
  }
}

/**
 * Programma di controllo vocale
 */
void voice_control() {
  if (voice_time == -1) {
    voice_action(current_voice_code, VOICE_CONTROL_POWER);
  } else {
    if (millis() - voice_start_time <= voice_time) {
      voice_action(current_voice_code, VOICE_CONTROL_POWER);
    } else {
      currentMode = MODE_NONE;
      voice_start_time = 0;
      current_voice_code = -1;
    }
  }
}

/**
 * Elaborazione dei dati ricevuti via WebSocket
 */
void onReceive() {
  // --------------------- invio dati ---------------------
  // tensione della batteria
  // Serial.print(F("tensione:"));Serial.println(batteryGetVoltage());
  aiCam.sendDoc["BV"] = batteryGetVoltage();

  // Dati del rilevamento ostacoli IR
  byte result = irObstacleRead();
  aiCam.sendDoc["N"] = int(!bool(result & 0b00000010));  // sinistra, libera:0
  aiCam.sendDoc["P"] = int(!bool(result & 0b00000001));  // destra, libera:0

  // ultrasonico
  float usDistance = int(ultrasonicRead() * 100) / 100.0;  // arrotonda a due decimali
  aiCam.sendDoc["O"] = usDistance;

  // --------------------- ricezione dati ---------------------
  // Stop
  if (aiCam.getButton(REGION_I)) {
    currentMode = MODE_NONE;
    current_voice_code = -1;
    voice_time = 0;
    carStop();
    return;
  }

  // Selezione modalità: follow ostacolo, evitamento ostacolo
  if (aiCam.getSwitch(REGION_E)) {
    if (currentMode != MODE_OBSTACLE_AVOIDANCE) {
      currentMode = MODE_OBSTACLE_AVOIDANCE;
    }
  } else if (aiCam.getSwitch(REGION_F)) {
    if (currentMode != MODE_OBSTACLE_FOLLOWING) {
      currentMode = MODE_OBSTACLE_FOLLOWING;
    }
  } else {
    if (currentMode == MODE_OBSTACLE_FOLLOWING || currentMode == MODE_OBSTACLE_AVOIDANCE) {
      currentMode = MODE_NONE;
      carStop();
      return;
    }
  }

  // Lampada della fotocamera
  if (aiCam.getSwitch(REGION_M) && !cam_lamp_status) {
    Serial.println("lampada accesa");
    aiCam.lamp_on(5);  // accendi la lampada, livello da 0 a 10
    cam_lamp_status = true;
  } else if (!aiCam.getSwitch(REGION_M) && cam_lamp_status) {
    Serial.println("lampada spenta");
    aiCam.lamp_off();  // spegni la lampada
    cam_lamp_status = false;
  }

  // Controllo vocale
  if (currentMode != MODE_VOICE_CONTROL) {
    current_voice_code = -1;
    voice_time = 0;
    voice_start_time = 0;
    aiCam.sendDoc["J"] = 0;
  }

  int8_t code = -1;
  voice_buf_temp[0] = 0;  // buffer controllo vocale
  aiCam.getSpeech(REGION_J, voice_buf_temp);
  if (strlen(voice_buf_temp) > 0) {
    aiCam.sendDoc["J"] = 1;
    aiCam.sendData();
    aiCam.sendDoc["J"] = 0;
    code = text_2_cmd_code(voice_buf_temp);
    if (code != -1) {
      current_voice_code = code;
      voice_time = voice_action_time[code];
      voice_start_time = millis();
    }
  }

  if (current_voice_code != -1) {
    currentMode = MODE_VOICE_CONTROL;
  }

  // angolo del servo
  int temp = aiCam.getSlider(REGION_D);
  if (servoAngle != temp) {
    if (currentMode == MODE_NONE || currentMode == MODE_DISCONNECT) {
      currentMode = MODE_APP_CONTROL;
    }
    if (SERVO_REVERSE) {
      temp = constrain(temp, 40, 180);
      temp = 180 - temp;
    } else {
      temp = constrain(temp, 0, 140);
    }
    servoAngle = temp;
  }

  // acceleratori
  int throttle_L = aiCam.getThrottle(REGION_K);
  int throttle_R = aiCam.getThrottle(REGION_Q);
  // Serial.print("throttle_L: "); Serial.print(throttle_L);
  // Serial.print("throttle_R: "); Serial.println(throttle_R);
  if (throttle_L != 0 || throttle_R != 0 || throttle_L != leftMotorPower || throttle_R != rightMotorPower) {
    currentMode = MODE_APP_CONTROL;
    leftMotorPower = throttle_L;
    rightMotorPower = throttle_R;
  }
}
