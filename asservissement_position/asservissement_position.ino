#include <Encoder.h>
#include <PID_v1.h>
#include "MCC.h"

/*Interruption*/
Encoder monEnc(2, 3);
//Compteurs de pas des moteurs
double position = 0;

/*Moteur*/
const double tour = 1633;//408.25 pas par tour
double pwmMax = 150;
MCC m(5, 6);

/*PID*/
double consigne = 1*tour; //la consigne donne la vitesse voulue du moteur en tours/seconde
double commande; //la commande est le pwm envoyé sur le moteur
int echantillonnage = 1; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

//Réglage des coefficient des PID
const double kp = 1;
const double ki = 0;
const double kd = 0.021;

PID monPID(&position, &commande, &consigne, kp, ki, kd, DIRECT);

void setup() {
  /*Changement de fréquence des pwm des pins 5,6*/
  TCCR0B = (TCCR0B & 0xf8) | 0x01;
  
  /*Initialisation de la liaison série*/
  Serial.begin(115200);

  //Initialisation PID
  monPID.SetSampleTime(echantillonnage);
  monPID.SetOutputLimits(-pwmMax, pwmMax);
  monPID.SetMode(AUTOMATIC);
}


void loop() {
  //Calcul et application de la commande
  position = monEnc.read();
  monPID.Compute();
  m.bouger((int)commande);

  //Affichage liaison série
  Serial.println(String(position) + " " + String(commande) + " " + String(consigne));
}
