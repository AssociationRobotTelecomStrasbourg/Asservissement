#include <Encoder.h>
#include <PID_v1.h>
#include <MCC.h>

/*Interruption*/
Encoder monEnc1(2, 4);
Encoder monEnc2(3, 5);
//Compteurs de pas des moteurs
double position1 = 0, position2 = 0;
double dernierePosition1 = 0, dernierePosition2 = 0;

/*Moteur*/
#define N_FILTRE 4
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesse1, vitesse2, vitesses1[N_FILTRE] = {0}, vitesses2[N_FILTRE] = {0};//en tours/seconde
double vitesseLineaire, vitesseOrientation;
int i1 = 0, i2 = 0;
const double tour = 1633;//408.25 pas par tour
double pwmMax = 255;
MCC m1(A0, A1, 9), m2(A2, A3, 10);

/*PID*/
double consigneLineaire = 2, consigneOrientation = 0; //la consigne donne la vitesse voulue du moteur en tours/seconde
double commandeLineaire, commandeOrientation; //la commande est le pwm envoyé sur le moteur
unsigned int echantillonnage = 1; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

//Réglage des coefficient des PID
const double kp = 2000;
const double ki = 400;
const double kd = 0;

PID monPID1(&vitesseLineaire, &commandeLineaire, &consigneLineaire, kp, ki, kd, DIRECT);
PID monPID2(&vitesseOrientation, &commandeOrientation, &consigneOrientation, kp, ki, kd, DIRECT);

void getVitesse1() {
  vitesse1 -= vitesses1[i1] / N_FILTRE;
  vitesses1[i1] = (position1 - dernierePosition1) / deltaTemps * 1000 / tour;
  vitesse1 += vitesses1[i1] / N_FILTRE;
  if (++i1 == N_FILTRE) i1 = 0;
  dernierePosition1 = position1;
}

void getVitesse2() {
  vitesse2 -= vitesses2[i2] / N_FILTRE;
  vitesses2[i2] = (position2 - dernierePosition2) / deltaTemps * 1000 / tour;
  vitesse2 += vitesses2[i2] / N_FILTRE;
  if (++i2 == N_FILTRE) i2 = 0;
  dernierePosition2 = position2;
}

void setup() {
  /*Changement fréquence des pwm des pins 9, 10*/
  TCCR1B = (TCCR1B & 0xf8) | 0x01;

  /*Initialisation de la liaison série*/
  Serial.begin(115200);

  //Initialisation PID
  monPID1.SetSampleTime(echantillonnage);
  monPID1.SetOutputLimits(-pwmMax, pwmMax);
  monPID1.SetMode(AUTOMATIC);

  monPID2.SetSampleTime(echantillonnage);
  monPID2.SetOutputLimits(-pwmMax, pwmMax);
  monPID2.SetMode(AUTOMATIC);

  dernierTemps = millis() - echantillonnage;
}


void loop() {
  //Calcul et application de la commande
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= echantillonnage) {
    position1 = monEnc1.read();
    position2 = monEnc2.read();
    getVitesse1();
    getVitesse2();
    vitesseLineaire = (vitesse1 + vitesse2) / 2;
    vitesseOrientation = (-vitesse1 + vitesse2) / 2;
    dernierTemps = maintenant;
  }
  monPID1.Compute();
  monPID2.Compute();
  m1.bouger((int)commandeLineaire - commandeOrientation);
  m2.bouger((int)commandeLineaire + commandeOrientation);

  //Affichage liaison série
  Serial.println(String(vitesseLineaire) + " 2");
}
