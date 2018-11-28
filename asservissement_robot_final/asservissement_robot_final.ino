#include <Encoder.h>
#include <PID_v1.h>
#include <MCC.h>
#include "conversion.h"

/*Interruption*/
Encoder encGauche(2, 4);
Encoder encDroite(3, 5);

//Position en pas des moteurs
double positionGauche = 0, positionDroite = 0;
double dernierepositionGauche = 0, dernierepositionDroite = 0;

/*Moteur*/
//Coefficient de proportion tension vitesse
#define CG 0.97
#define CD 1
#define VIT_MAX cmToPas(50) //vitesse max en pas/s
#define ACCE cmToPas(50) //accélération lineaire en cm/s²
#define PWM_MAX 255 //pwm max envoyé aux moteurs
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesseGaucheMesure, vitesseDroiteMesure; //en pas/seconde
double vitesseLineaireMesure, vitesseRotationMesure; //en pas/seconde
MCC moteurGauche(A0, A1, 9), moteurDroite(A2, A3, 10);

/*Odométrie*/
double x = 0, y = 0, theta = 0;

/*Trajectoire*/
/*Fonctionnement : va vers le point (consigneX[i],consigneY[i])
  puis s'oriente selon consigneTheta[i]
  i est pointActuel
*/
#define ANGLE_FIXE_LIN 1 //angle en mode linéaire fixé à telle distance des points en cm
#define ERREUR_LIN 1 //erreur lineaire en cm de passage sur les points
#define ERREUR_ROT 0.05 //erreur lineaire en cm de passage sur les points

int pointActuel = 0; //indice du point à aller

/*Aller retour sans demi-tour*/
#define N_POINT 2 //nombre de points du parcours
double consigneX[N_POINT] = {100, 0};
double consigneY[N_POINT] = {0, 0};
double consigneTheta[N_POINT] = {0, 0};

/*Aller retour avec demi-tour*/
//#define N_POINT 2 //nombre de points du parcours
//double consigneX[N_POINT] = {100, 0};
//double consigneY[N_POINT] = {0, 0};
//double consigneTheta[N_POINT] = {PI, 0};

/*Carre*/
//#define N_POINT 4 //nombre de points du parcours
//double consigneX[N_POINT] = {25, 25, 0, 0};
//double consigneY[N_POINT] = {0, 25, 25, 0};
//double consigneTheta[N_POINT] = {HALF_PI, PI, -HALF_PI, 0};


/*PID*/
#define ECHANT_MS 3//l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde
#define ECHANT_S 0.003

double consignePosLineaire = 0, consignePosRotation = 0; //en pas
double positionLineaire = 0, positionRotation = 0; //en pas
double consigneVitLineaire = 0, consigneVitRotation = 0; //en pas/s
double derrniereConsigneVitLineaire = 0, derrniereConsigneVitRotation = 0; //en pas/s
double commandeVitLineaire = 0, commandeVitRotation = 0; //la commande est le pwm envoyé sur le moteur

//Réglage des coefficient des PID vitesse
const double kpVit = 0.7;
const double kiVit = 0.5;
const double kdVit = 0;

//Réglage des coefficient des PID position
const double kpPos = 10;
const double kiPos = 0;
const double kdPos = 0;

PID positionLineairePID(&positionLineaire, &consigneVitLineaire, &consignePosLineaire, kpPos, kiPos, kdPos, DIRECT);
PID positionRotationPID(&positionRotation, &consigneVitRotation, &consignePosRotation, kpPos, kiPos, kdPos, DIRECT);

PID vitesseLineairePID(&vitesseLineaireMesure, &commandeVitLineaire, &consigneVitLineaire, kpVit, kiVit, kdVit, DIRECT);
PID vitesseRotationPID(&vitesseRotationMesure, &commandeVitRotation, &consigneVitRotation, kpVit, kiVit, kdVit, DIRECT);

//Restreint l'angle dans l'intervalle [-PI;PI[
double moduloAngle(double angle) {
  double tmp = fmod(angle, TWO_PI);
  if (tmp >= PI) {
    tmp -= TWO_PI;
  }
  if (tmp < -PI) {
    tmp += TWO_PI;
  }
  return tmp;
}

void affichage() {
  Serial.print(consigneX[pointActuel]);
  Serial.print(" 0 ");
  Serial.println(x);
}

void getvitesseGaucheMesure() {
  vitesseGaucheMesure = (positionGauche - dernierepositionGauche) / ECHANT_S;
  dernierepositionGauche = positionGauche;
}

void getvitesseDroiteMesure() {
  vitesseDroiteMesure = (positionDroite - dernierepositionDroite) / ECHANT_S;
  dernierepositionDroite = positionDroite;
}

void trajectoire() {
  //Mode de déplacement lineaire:true, rotation:false
  static bool lineaireRotation = true;
  double erreurLineaire;
  double erreurRotation;

  if (lineaireRotation) {
    erreurLineaire = sqrt(sq(consigneX[pointActuel] - x) + sq(consigneY[pointActuel] - y));
    erreurRotation = erreurLineaire < ANGLE_FIXE_LIN ? 0 : moduloAngle(atan2(consigneY[pointActuel] - y, consigneX[pointActuel] - x) - theta);
    if (abs(erreurRotation) > HALF_PI) {
      erreurLineaire *= -1;
      erreurRotation = moduloAngle(erreurRotation + PI);
    }
    if (abs(erreurLineaire) < ERREUR_LIN) {
      //      pointActuel = (pointActuel + 1) % N_POINT;
      lineaireRotation = false;
    }
  }
  else {
    erreurLineaire = 0;
    erreurRotation = moduloAngle(consigneTheta[pointActuel] - theta);
    if (abs(erreurRotation) < ERREUR_ROT) {
      pointActuel = (pointActuel + 1) % N_POINT;
      lineaireRotation = true;
    }
  }

  consignePosLineaire = cmToPas(erreurLineaire);
  consignePosRotation = radToPas(erreurRotation);
}

//Renvoie la vitesse corrigée
void rampeVitesse(double &vitesse, double &derniereVitesse) {
  double acceleration = (vitesse - derniereVitesse) / ECHANT_S;
  if (acceleration > ACCE) {
    vitesse = derniereVitesse + ACCE * ECHANT_S;
  }
  else if (acceleration < -ACCE) {
    vitesse = derniereVitesse - ACCE * ECHANT_S;
  }
  derniereVitesse = vitesse;
}

void odometrie() {
  x += pasToCm(cos(theta) * vitesseLineaireMesure * ECHANT_S);
  y += pasToCm(sin(theta) * vitesseLineaireMesure * ECHANT_S);
  theta += pasToRad(vitesseRotationMesure * ECHANT_S);
}

void setup() {
  /*Changement fréquence des pwm des pins 9, 10*/
  TCCR1B = (TCCR1B & 0xf8) | 0x01;

  /*Initialisation de la liaison série*/
  Serial.begin(115200);

  //Initialisation PID
  positionLineairePID.SetSampleTime(ECHANT_MS);
  positionLineairePID.SetOutputLimits(-VIT_MAX, VIT_MAX);
  positionLineairePID.SetMode(AUTOMATIC);

  positionRotationPID.SetSampleTime(ECHANT_MS);
  positionRotationPID.SetOutputLimits(-VIT_MAX, VIT_MAX);
  positionRotationPID.SetMode(AUTOMATIC);

  vitesseLineairePID.SetSampleTime(ECHANT_MS);
  vitesseLineairePID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  vitesseLineairePID.SetMode(AUTOMATIC);

  vitesseRotationPID.SetSampleTime(ECHANT_MS);
  vitesseRotationPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  vitesseRotationPID.SetMode(AUTOMATIC);

  dernierTemps = millis() - ECHANT_MS;
}


void loop() {
  //Calcul des consignes tout les temps echantillonage
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= ECHANT_MS) {
    //Calcul des PID position
    positionLineairePID.Compute();
    positionRotationPID.Compute();

    //Génération de la rampe de vitesse
    rampeVitesse(consigneVitLineaire, derrniereConsigneVitLineaire);
    rampeVitesse(consigneVitRotation, derrniereConsigneVitRotation);

    //Calcul des PID vitesse
    vitesseLineairePID.Compute();
    vitesseRotationPID.Compute();

    //Envoie des commandes sur les moteurs
    moteurGauche.bouger((commandeVitLineaire - commandeVitRotation) * CG);
    moteurDroite.bouger((commandeVitLineaire + commandeVitRotation) * CD);

    //Lecture des positions des moteurs
    positionGauche = encGauche.read();
    positionDroite = encDroite.read();

    //    positionLineaire = (positionGauche + positionDroite) / 2;
    //    positionRotation = positionLineaire - positionGauche;

    //Calculs des vitesses des moteurs
    getvitesseGaucheMesure();
    getvitesseDroiteMesure();
    vitesseLineaireMesure = (vitesseGaucheMesure + vitesseDroiteMesure) / 2;
    vitesseRotationMesure = vitesseLineaireMesure - vitesseGaucheMesure;

    //Génération des consignes de positions trajectoire : modifications des consignes de position
    trajectoire();

    //Odométrie
    odometrie();

    //Affichage liaison série
    affichage();

    dernierTemps = maintenant;
  }
}
