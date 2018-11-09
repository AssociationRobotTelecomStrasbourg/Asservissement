#include <Encoder.h>
#include <PID_v1.h>
#include <MCC.h>

/*Interruption*/
Encoder monEnc(2, 4);
//Compteurs de pas des moteurs
double position = 0;
double dernierePosition = 0;

/*Moteur*/
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesse = 0;//en tours/seconde
const double tour = 1633;//408.25 pas par tour
double pwmMax = 255;
MCC m(A0, A1, 9);

/*PID*/
double consigne = 2; //la consigne donne la vitesse voulue du moteur en tours/seconde
double commande; //la commande est le pwm envoyé sur le moteur
int echantillonnage = 10; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

//Réglage des coefficient des PID
const double kp = 400;
const double ki = 0;
const double kd = 0;

PID monPID(&vitesse, &commande, &consigne, kp, ki, kd, DIRECT);

void getVitesse() {
  vitesse = (position - dernierePosition) / deltaTemps * 1000 / tour;
  dernierePosition = position;
}

void setup() {
  /*Changement fréquence des pwm des pins 9, 10*/
  TCCR1B = (TCCR1B & 0xf8) | 0x01;
  
  /*Initialisation de la liaison série*/
  Serial.begin(38400);

  //Initialisation PID
  monPID.SetSampleTime(echantillonnage);
  monPID.SetOutputLimits(-pwmMax, pwmMax);
  monPID.SetMode(AUTOMATIC);

  dernierTemps = millis() - echantillonnage;
}


void loop() {
  //Calcul et application de la commande
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= echantillonnage) {
    position = monEnc.read();
    getVitesse();
    dernierTemps = maintenant;
  }
  monPID.Compute();
  m.bouger((int)commande);

  //Affichage liaison série
  Serial.println(String(vitesse) + " 0 2");
}
