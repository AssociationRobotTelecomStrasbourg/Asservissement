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
#define N_FILTRE 4
unsigned long dernierTemps, maintenant, deltaTemps; //en ms
double vitesseGauche, vitesseDroite, vitessesGauche[N_FILTRE] = {0}, vitessesDroite[N_FILTRE] = {0};//en tours/seconde
double vitesseLineaire, vitesseRotation;
int iGauche = 0, iDroite = 0;
const double tour = 1633;//408.25 pas par tour
double pwmMax = 255;
MCC moteurGauche(A0, A1, 9), moteurDroite(A2, A3, 10);

/*Odométrie*/
double x = 0, y = 0, theta = 0, distanceLineaire, distanceOrientation;

/*PID*/
double consigneVitLineaire = 2, consigneVitRotation = 0;
double consigneVitGauche = consigneVitLineaire - consigneVitRotation, consigneVitDroite = consigneVitLineaire + consigneVitRotation; //la consigne donne la vitesse voulue du moteur en tours/seconde
double commandeVitGauche, commandeVitDroite; //la commande est le pwm envoyé sur le moteur
unsigned int echantillonnage = 1; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde

//Réglage des coefficient des PID
const double kp = 900;
const double ki = 400;
const double kd = 0;

PID vitesseGauchePID(&vitesseGauche, &commandeVitGauche, &consigneVitGauche, kp, ki, kd, DIRECT);
PID vitesseDroitePID(&vitesseDroite, &commandeVitDroite, &consigneVitDroite, kp, ki, kd, DIRECT);

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
  vitesseGauchePID.SetSampleTime(echantillonnage);
  vitesseGauchePID.SetOutputLimits(-pwmMax, pwmMax);
  vitesseGauchePID.SetMode(AUTOMATIC);

  vitesseDroitePID.SetSampleTime(echantillonnage);
  vitesseDroitePID.SetOutputLimits(-pwmMax, pwmMax);
  vitesseDroitePID.SetMode(AUTOMATIC);

  dernierTemps = millis() - echantillonnage;
}


void loop() {
  //Calcul et application de la commande
  maintenant = millis();
  deltaTemps = maintenant - dernierTemps;
  if (deltaTemps >= echantillonnage) {
    positionGauche = encGauche.read();
    positionDroite = encDroite.read();
    getvitesseGauche();
    getvitesseDroite();
    vitesseLineaire = (vitesseGauche + vitesseDroite) / 2;
    vitesseRotation = (-vitesseGauche + vitesseDroite) / 2;
    dernierTemps = maintenant;
    distanceLineaire = vitesseLineaire * deltaTemps * COEFF_D;
    distanceOrientation = vitesseRotation * deltaTemps * COEFF_R;
    x += cos(theta) * distanceLineaire;
    y += sin(theta) * distanceLineaire;
    theta += distanceOrientation;
  }
  vitesseGauchePID.Compute();
  vitesseDroitePID.Compute();
  moteurGauche.bouger((int)commandeVitGauche);
  moteurDroite.bouger((int)commandeVitDroite);

  //Affichage liaison série
  Serial.print(vitesseGauche);
  Serial.print(" 0 2 ");
  Serial.println(vitesseDroite);
}
