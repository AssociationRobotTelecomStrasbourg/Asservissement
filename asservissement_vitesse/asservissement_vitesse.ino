#include <PID_v1.h>
#include "MCC.h"

/*Interruption*/
//Pins des encodeurs des moteurs
#define pinA 0
#define pinB 3
//Compteurs de pas des moteurs
volatile double compteur = 0;
double dernierCompteur = 0;

/*Moteur*/
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesse = 0;//en tours/seconde
const double tour = 408.25;//408.25 pas par tour
double tensionMax = 255;
MCC m(5, 6);

/*PID*/
double consigne = 2; //la consigne donne la vitesse voulue du moteur en tours/seconde
double commande; //la commande est le pwm envoyé sur le moteur
int echantillonnage = 10; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

//Réglage des coefficient des PID
const double kp = 500;
const double ki = 0;
const double kd = 0;

PID monPID(&vitesse, &commande, &consigne, kp, ki, kd, DIRECT);

void increment() {
  if (digitalRead(pinB) == HIGH) {
    compteur++;
  }
  else {
    compteur--;
  }
}

void getVitesse() {
  vitesse = (compteur - dernierCompteur) / deltaTemps * 1000 / tour;
  dernierCompteur = compteur;
}

void setup() {
  /*Initialisation de la liaison série*/
  Serial.begin(19200);

  //Mise en place des interruptions
  pinMode(pinB, INPUT);
  attachInterrupt(pinA, increment, RISING);

  //Initialisation PID
  monPID.SetSampleTime(echantillonnage);
  monPID.SetOutputLimits(-tensionMax, tensionMax);
  monPID.SetMode(AUTOMATIC);

  dernierTemps = millis() - echantillonnage;
}


void loop() {
  //Calcul et application de la commande
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= echantillonnage) {
    getVitesse();
    dernierTemps = maintenant;
  }
  monPID.Compute();
  m.bouger((int)commande);

  //Affichage liaison série
  Serial.println(String(millis())+" "+String(millis())+" "+String(vitesse));
}
