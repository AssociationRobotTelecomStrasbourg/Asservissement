#include "conversion.h"

double pasToCm(double pas) {
  return pas / PAS_TOUR * CM_TOUR;
}

double pasToRad(double pas) {
  return pas / PAS_TOUR * RAD_TOUR;
}

double cmToPas(double cm) {
  return cm / CM_TOUR * PAS_TOUR;
}

double radToPas(double rad) {
  return rad / RAD_TOUR * PAS_TOUR;
}
