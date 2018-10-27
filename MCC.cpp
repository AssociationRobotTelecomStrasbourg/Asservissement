#include "MCC.h"
#include <Arduino.h>

MCC::MCC(int avancer, int reculer) : pinAvancer(avancer), pinReculer(reculer){
  pinMode(pinAvancer, OUTPUT);
  pinMode(pinReculer, OUTPUT);

  analogWrite(pinAvancer, 0);
  analogWrite(pinReculer, 0);
}

void MCC::bouger(int vitesse){
  if(vitesse>0){
    analogWrite(pinAvancer, vitesse);
    analogWrite(pinReculer, 0);
  }
  else{
    analogWrite(pinAvancer, 0);
    analogWrite(pinReculer, -vitesse);
  }
}
