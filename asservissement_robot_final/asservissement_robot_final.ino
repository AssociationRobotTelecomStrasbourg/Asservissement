#include <Encoder.h>
#include <PID_v1.h>
#include <MCC.h>

#define COEFF_D 0.02356 //en cm/tourDeRoue
#define COEFF_R 0.00242 //en rad/tourDeRoue

/*Interruption*/
Encoder encGauche(2, 4);
Encoder encDroite(3, 5);
//Compteurs de pas des moteurs
double positionGauche = 0, positionDroite = 0;
double dernierepositionGauche = 0, dernierepositionDroite = 0;

/*Moteur*/
#define N_FILTRE 3
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesseGauche, vitesseDroite, vitessesGauche[N_FILTRE] = {0}, vitessesDroite[N_FILTRE] = {0};//en tours/seconde
double vitesseLineaire, vitesseRotation;
int iGauche = 0, iDroite = 0;
const double tour = 1633;//408.25 pas par tour
double pwmMax = 255;
double vitesseMax = 3;
MCC moteurGauche(A0, A1, 9), moteurDroite(A2, A3, 10);

/*Odométrie*/
double x = 0, y = 0, theta = 0, distanceLineaire, distanceRotation;
double consigneX = 1, consigneY = 1, consigneTheta = 0;
double consignePosLineaire = sqrt(sq(x - consigneX) + sq(y - consigneY)), consignePosRotation = atan2(consigneY - y, consigneX - x) - theta;
double positionLineaire, positionRotation;

/*PID*/
double consigneVitLineaire = 2, consigneVitRotation = 0;
double commandeVitLineaire, commandeVitRotation; //la commande est le pwm envoyé sur le moteur
unsigned int echantillonnage = 2; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

//Réglage des coefficient des PID
const double kpVit = 2200;
const double kiVit = 0;
const double kdVit = 0;

const double kpPos = 0;
const double kiPos = 0;
const double kdPos = 0;

PID positionLineairePID(&positionLineaire, &consigneVitLineaire, &consignePosLineaire, kpPos, kiPos, kdPos, DIRECT);
PID positionRotationPID(&positionRotation, &consigneVitRotation, &consignePosRotation, kpPos, kiPos, kdPos, DIRECT);

PID vitesseLineairePID(&vitesseLineaire, &commandeVitLineaire, &consigneVitLineaire, kpVit, kiVit, kdVit, DIRECT);
PID vitesseRotationPID(&vitesseRotation, &commandeVitRotation, &consigneVitRotation, kpVit, kiVit, kdVit, DIRECT);

void getvitesseGauche() {
  vitesseGauche -= vitessesGauche[iGauche] / N_FILTRE;
  vitessesGauche[iGauche] = (positionGauche - dernierepositionGauche) / echantillonnage * 1000 / tour;
  vitesseGauche += vitessesGauche[iGauche] / N_FILTRE;
  if (++iGauche == N_FILTRE) iGauche = 0;
  dernierepositionGauche = positionGauche;
}

void getvitesseDroite() {
  vitesseDroite -= vitessesDroite[iDroite] / N_FILTRE;
  vitessesDroite[iDroite] = (positionDroite - dernierepositionDroite) / echantillonnage * 1000 / tour;
  vitesseDroite += vitessesDroite[iDroite] / N_FILTRE;
  if (++iDroite == N_FILTRE) iDroite = 0;
  dernierepositionDroite = positionDroite;
}

void setup() {
  /*Changement fréquence des pwm des pins 9, 10*/
  TCCR1B = (TCCR1B & 0xf8) | 0x01;

  /*Initialisation de la liaison série*/
  Serial.begin(115200);

  //Initialisation PID
  positionLineairePID.SetSampleTime(echantillonnage);
  positionLineairePID.SetOutputLimits(-vitesseMax, vitesseMax);
  //positionLineairePID.SetMode(AUTOMATIC);

  positionRotationPID.SetSampleTime(echantillonnage);
  positionRotationPID.SetOutputLimits(-vitesseMax, vitesseMax);
  //positionRotationPID.SetMode(AUTOMATIC);

  vitesseLineairePID.SetSampleTime(echantillonnage);
  vitesseLineairePID.SetOutputLimits(-pwmMax, pwmMax);
  vitesseLineairePID.SetMode(AUTOMATIC);

  vitesseRotationPID.SetSampleTime(echantillonnage);
  vitesseRotationPID.SetOutputLimits(-pwmMax, pwmMax);
  vitesseRotationPID.SetMode(AUTOMATIC);

  dernierTemps = millis() - echantillonnage;
}


void loop() {
  //Calcul des consignes tout les temps echantillonage
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= echantillonnage) {
    //Lecture des positions des moteurs
    positionGauche = encGauche.read();
    positionDroite = encDroite.read();
    positionLineaire = (positionGauche + positionDroite) / 2;
    positionRotation = positionDroite - positionLineaire;


    //Calculs des vitesses des moteurs
    getvitesseGauche();
    getvitesseDroite();
    vitesseLineaire = (vitesseGauche + vitesseDroite) / 2;
    vitesseRotation = (-vitesseGauche + vitesseDroite) / 2;

    //Odométrie
    distanceLineaire = vitesseLineaire * deltaTemps * COEFF_D;
    distanceRotation = vitesseRotation * deltaTemps * COEFF_R;
    x += cos(theta) * distanceLineaire;
    y += sin(theta) * distanceLineaire;
    theta += distanceRotation;


    dernierTemps = maintenant;
  }
  vitesseLineairePID.Compute();
  vitesseRotationPID.Compute();
  moteurGauche.bouger((int)commandeVitLineaire - commandeVitRotation);
  moteurDroite.bouger((int)commandeVitLineaire + commandeVitRotation);

  //Affichage liaison série
  Serial.print(vitesseLineaire);
  Serial.print(" 0 2 ");
  Serial.println(vitesseRotation);
}
