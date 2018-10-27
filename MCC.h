#ifndef MCC_H
#define MCC_H

class MCC{    
  public :
    MCC(int avancer, int reculer);
    void bouger(int vitesse);

  private :
    int pinReculer;
    int pinAvancer;
};

#endif
