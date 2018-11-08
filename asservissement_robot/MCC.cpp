#include "MCC.h"
#include <Arduino.h>

MCC::MCC(int avancer, int reculer, int pwm) : pinAvancer(avancer), pinReculer(reculer), pinPWM(pwm) {
  pinMode(pinAvancer, OUTPUT);
  pinMode(pinReculer, OUTPUT);

  digitalWrite(pinAvancer, LOW);
  digitalWrite(pinReculer, LOW);
}

void MCC::bouger(int vitesse){
  if(vitesse>0){
    digitalWrite(pinReculer, LOW);
    digitalWrite(pinAvancer, HIGH);
    analogWrite(pinPWM, vitesse);
  }
  else{
    digitalWrite(pinReculer, HIGH);
    digitalWrite(pinAvancer, LOW);
    analogWrite(pinPWM, -vitesse);
  }
}
