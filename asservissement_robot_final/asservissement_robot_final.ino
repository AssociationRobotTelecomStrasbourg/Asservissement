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
#define VIT_MAX 2 //vitesse max en tourDeRoue/s (<2.5)
#define ACCE_L 50 //accélération lineaire en cm/s²
#define DECE_L 50 //décélération lineaire en cm/s²
#define ACCE_R 5 //accélération rotation en rad/s²
#define DECE_R 5 //décélération rotation en rad/s²
#define PWM_MAX 255 //pwm max envoyé aux moteurs
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesseGaucheMesure, vitesseDroiteMesure; //en tourDeRoue/seconde
double vitesseLineaireMesure, vitesseRotationMesure; //en tourDeRoue/seconde
MCC moteurGauche(A0, A1, 9), moteurDroite(A2, A3, 10);

/*Rampe de vitesse*/
double distanceLineaire = 0, derniereDistanceLineaire = 0, vitesseLineaire; //en cm(/s)
double distanceRotation = 0, derniereDistanceRotation  = 0, vitesseRotation; //en rad(/s)

/*Odométrie*/
double x = 0, y = 0, theta = 0;

/*Trajectoire*/
/*Fonctionnement : va vers le point (consigneX[i],consigneY[i])
  puis s'oriente selon consigneTheta[i]
  i est pointActuel
*/
#define ANGLE_FIXE_LIN 1 //angle en mode linéaire fixé à telle distance des points en cm
#define ERREUR_LIN 0.5 //erreur lineaire en cm de passage sur les points
#define ERREUR_ROT 0.01 //erreur lineaire en cm de passage sur les points

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
#define ECHANT_MS 2 //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde
#define ECHANT_S ECHANT_MS / 1000

double consignePosLineaire = 0, consignePosRotation = 0;
double positionLineaire = 0, positionRotation = 0;
double consigneVitLineaire = 0, consigneVitRotation = 0;
double commandeVitLineaire = 0, commandeVitRotation = 0; //la commande est le pwm envoyé sur le moteur

//Réglage des coefficient des PID vitesse
const double kpVit = 2000;
const double kiVit = 2000;
const double kdVit = 0;

//Réglage des coefficient des PID position
const double kpPos = 0.1;
const double kiPos = 0;
const double kdPos = 0.01;

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

void getvitesseGaucheMesure() {
  vitesseGaucheMesure = pasToTour(positionGauche - dernierepositionGauche) / ECHANT_S;
  dernierepositionGauche = positionGauche;
}

void getvitesseDroiteMesure() {
  vitesseDroiteMesure = pasToTour(positionDroite - dernierepositionDroite) / ECHANT_S;
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

  derniereDistanceLineaire = distanceLineaire;
  derniereDistanceRotation = distanceRotation;
  distanceLineaire = erreurLineaire;
  distanceRotation = erreurRotation;
  vitesseLineaire = (distanceLineaire - derniereDistanceLineaire) / ECHANT_S;
  vitesseRotation = (distanceRotation - derniereDistanceRotation) / ECHANT_S;
}

double rampeVitesse(double distance, double vitesse, double acceleration, double deceleration) {
  double distanceFrein = sq(vitesse) / (2 * deceleration);
  if (distance < distanceFrein) {
    return vitesse - deceleration * ECHANT_S;
  }
  else if (vitesse < VIT_MAX) {
    return vitesse + acceleration * ECHANT_S;
  }
  else {
    return VIT_MAX;
  }
}

void odometrie() {
  x += tourToCm(cos(theta) * vitesseLineaireMesure * ECHANT_S);
  y += tourToCm(sin(theta) * vitesseLineaireMesure * ECHANT_S);
  theta += tourToRad(vitesseRotationMesure * ECHANT_S);
}

void affichage() {
  Serial.print(x);
  Serial.print(" ");
  Serial.print(theta);
  Serial.print(" ");
  Serial.println(y);
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
    consigneVitLineaire = cmToPas(rampeVitesse(distanceLineaire, vitesseLineaire, ACCE_L, DECE_L));
    consigneVitRotation = radToPas(rampeVitesse(distanceRotation, vitesseRotation, ACCE_R, DECE_R));
    
    //Calcul des PID vitesse
    vitesseLineairePID.Compute();
    vitesseRotationPID.Compute();

    //Envoie des commandes sur les moteurs
    moteurGauche.bouger((commandeVitLineaire - commandeVitRotation) * CG);
    moteurDroite.bouger((commandeVitLineaire + commandeVitRotation) * CD);

    //Lecture des positions des moteurs
    positionGauche = encGauche.read();
    positionDroite = encDroite.read();

    //Calculs des vitesses des moteurs
    getvitesseGaucheMesure();
    getvitesseDroiteMesure();
    vitesseLineaireMesure = (vitesseGaucheMesure + vitesseDroiteMesure) / 2;
    vitesseRotationMesure = (-vitesseGaucheMesure + vitesseDroiteMesure) / 2;

    //Génération des consignes de positions trajectoire : modifications des consignes de position
    trajectoire();

    //Odométrie
    odometrie();

    //Affichage liaison série
//    affichage();

    dernierTemps = maintenant;
  }
}
