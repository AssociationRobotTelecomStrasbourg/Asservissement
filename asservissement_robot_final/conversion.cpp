#include "conversion.h"

double pasToCm(double pas) {
  return pas / PAS_TOUR * CM_TOUR;
}

double pasToRad(double pas) {
  return pas / PAS_TOUR * RAD_TOUR;
}

double pasToTour(double pas) {
  return pas / PAS_TOUR;
}


double tourToCm(double tour) {
  return tour * CM_TOUR;
}

double tourToRad(double tour) {
  return tour * RAD_TOUR;
}

double tourToPas(double tour) {
  return tour * PAS_TOUR;
}


double cmToPas(double cm) {
  return cm / CM_TOUR * PAS_TOUR;
}

double cmToTour(double cm) {
  return cm / CM_TOUR;
}


double radToPas(double rad) {
  return rad / RAD_TOUR * PAS_TOUR;
}

double radToTour(double rad) {
  return rad / RAD_TOUR;
}
