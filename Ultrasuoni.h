#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

/** Imposta i pin dell'ultrasonico 
 * |Per risparmiare porte IO, echo e trig possono condividere un pin identico
*/
#define ULTRASONIC_TRIG_PIN 10
#define ULTRASONIC_ECHO_PIN 10

/** Configura la distanza di evitamento ostacoli ultrasonici, unità cm */
#define ULTRASONIC_AVOIDANCE_THRESHOLD 20

#define MAX_DISTANCE 300 // unità: cm 

#define ULTRASONIC_READ_TIMEOUT 18000 // us , 2*300/34000*1000000 ~= 17647 us

/** Restituisce la distanza letta dal modulo ultrasonico, unità cm */
float ultrasonicRead();

/** Determina se c'è un ostacolo davanti in base al valore impostato di ULTRASONIC_AVOIDANCE_THRESHOLD */
bool ultrasonicIsObstacle();

/** Determina se c'è spazio libero davanti in base al valore impostato di ULTRASONIC_AVOIDANCE_THRESHOLD */
bool ultrasonicIsClear();

#endif // __ULTRASONIC_H__
