#ifndef CONVERSION_H
#define CONVERSION_H

#define CM_TOUR 23.56 //en cm/tourDeRoue
#define RAD_TOUR 2.42 //en rad/tourDeRoue
#define PAS_TOUR 1633 //en pas/tourDeRoue

double pasToCm(double pas);
double pasToRad(double pas);
double pasToTour(double pas);

double tourToCm(double tour);
double tourToRad(double tour);
double tourToPas(double tour);

double cmToPas(double cm);
double cmToTour(double cm);

double radToPas(double rad);
double radToTour(double rad);

#endif
