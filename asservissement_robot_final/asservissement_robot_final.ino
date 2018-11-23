#include <Encoder.h>
#include <PID_v1.h>
#include <MCC.h>

#define COEFF_D 0.02356 //en cm/tourDeRoue
#define COEFF_R 0.00242 //en rad/tourDeRoue

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
#define VIT_MAX 2 //vitesse max en tourDeRoue/s
#define ACC_MAX 50 //accélération en cm/s²
#define ACC_FREIN 50 //décélération en cm/s²
#define PWM_MAX 255 //pwm max envoyé aux moteurs
#define TOUR 1633 //en pas/tourDeRoue
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesseGauche, vitesseDroite; //en tourDeRoue/seconde
double vitesseLineaireMesure, vitesseRotationMesure; //en tourDeRoue/seconde
MCC moteurGauche(A0, A1, 9), moteurDroite(A2, A3, 10);

/*Rampe de vitesse*/
double distanceLineaire, derniereDistanceLineaire, vitesseLineaire;
double distanceRotation, derniereDistanceRotation, vitesseRotation;

/*Odométrie*/
#define ANGLE_FIXE_LIN 1 //angle en mode linéaire fixé à telle distance des points en cm
#define ERREUR_LIN 0.1 //erreur lineaire en cm de passage sur les points
#define ERREUR_ROT 0.01 //erreur lineaire en cm de passage sur les points
double x = 0, y = 0, theta = 0;

/*Trajectoire*/
/*Fonctionnement : va vers le point (consigneX[i],consigneY[i])
  puis s'oriente selon consigneTheta[i]
  i est pointActuel
*/
int pointActuel = 0;

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
double consignePosLineaire = 0, consignePosRotation = 0;
double positionLineaire = 0, positionRotation = 0;
double consigneVitLineaire = 0, consigneVitRotation = 0;
double commandeVitLineaire, commandeVitRotation; //la commande est le pwm envoyé sur le moteur
unsigned int echantillonnage = 2; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

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

void getVitesseGauche() {
  vitesseGauche = (positionGauche - dernierepositionGauche) / echantillonnage * 1000 / TOUR;
  dernierepositionGauche = positionGauche;
}

void getVitesseDroite() {
  vitesseDroite = (positionDroite - dernierepositionDroite) / echantillonnage * 1000 / TOUR;
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

  consignePosLineaire = erreurLineaire / COEFF_D / 1000 * TOUR;
  consignePosRotation = erreurRotation / COEFF_R / 1000 * TOUR;
}

void rampeVitesse(double &distance, double &vitesse) {
  //TO DO
  //http://cubot.fr/ateliers/asservissement/chap-5/
  if (sq(vitesse) / (2 * ACC_FREIN)) {

  }
}

void odometrie() {
  x += cos(theta) * vitesseLineaireMesure * echantillonnage * COEFF_D;
  y += sin(theta) * vitesseLineaireMesure * echantillonnage * COEFF_D;
  theta += vitesseRotationMesure * echantillonnage * COEFF_R;
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
  positionLineairePID.SetSampleTime(echantillonnage);
  positionLineairePID.SetOutputLimits(-VIT_MAX, VIT_MAX);
  positionLineairePID.SetMode(AUTOMATIC);

  positionRotationPID.SetSampleTime(echantillonnage);
  positionRotationPID.SetOutputLimits(-VIT_MAX, VIT_MAX);
  positionRotationPID.SetMode(AUTOMATIC);

  vitesseLineairePID.SetSampleTime(echantillonnage);
  vitesseLineairePID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  vitesseLineairePID.SetMode(AUTOMATIC);

  vitesseRotationPID.SetSampleTime(echantillonnage);
  vitesseRotationPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  vitesseRotationPID.SetMode(AUTOMATIC);

  dernierTemps = millis() - echantillonnage;
}


void loop() {
  //Calcul des consignes tout les temps echantillonage
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= echantillonnage) {
    //Calcul des PID
    positionLineairePID.Compute();
    positionRotationPID.Compute();

    //Génération de la rampe de vitesse
    rampeVitesse(consignePosLineaire, vitesseLineaireMesure);
    rampeVitesse(consignePosRotation, vitesseRotationMesure);

    vitesseLineairePID.Compute();
    vitesseRotationPID.Compute();

    //Envoie des commandes sur les moteurs
    moteurGauche.bouger((commandeVitLineaire - commandeVitRotation) * CG);
    moteurDroite.bouger((commandeVitLineaire + commandeVitRotation) * CD);

    //Lecture des positions des moteurs
    positionGauche = encGauche.read();
    positionDroite = encDroite.read();

    //Calculs des vitesses des moteurs
    getVitesseGauche();
    getVitesseDroite();
    vitesseLineaireMesure = (vitesseGauche + vitesseDroite) / 2;
    vitesseRotationMesure = (-vitesseGauche + vitesseDroite) / 2;

    //Génération des consignes de positions trajectoire : modifications des consignes de position
    trajectoire();

    //Odométrie
    odometrie();

    //Affichage liaison série
    affichage();

    dernierTemps = maintenant;
  }
}
