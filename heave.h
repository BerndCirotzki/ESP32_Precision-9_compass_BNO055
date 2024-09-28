//*****************************************************************************
//
// File Name     : 'heave.h'
// Improoved by  : Bernd Cirotzki

#ifndef HEAVE_H
#define HEAVE_H

#include <Arduino.h>
#include <math.h>
#include "N2kMessages.h"

#define GRAVITYCOUNTER 500   // 25 Sekunden Durchschnitt für Gravitation Ermittlung
#define ACCSMOOTHCOUNTER 5  // Last ACC Values

class heave
{
public:
    heave();
    ~heave(){};
    double GetHeave(float acc, float PitchAccHead,float RollAccHead, float dt);
    bool HeaveAvailable() { return HaveGravityAverage; };
private:
    bool MakeGravity(float acc);    
    float CheckHeave(float heavein);

    // Heave
    double HeaveValue;
    double Heave_vorher;
    double Heave_Max_positiv;
    double Heave_Max_negativ;
    unsigned long Last_Heave_Max_positiv_Time;
    unsigned long Last_Heave_Max_negativ_Time;
    bool Aufwaerts = false;
    bool Abwaerts = false;
    
    // Gravity
    float GravityAverage;
    float Gravity[GRAVITYCOUNTER];
    int Gravity_index;
    bool HaveGravityAverage;

    // Acc
    float AccValue;
    float AccSmooth[ACCSMOOTHCOUNTER];
    int AccSmooth_index;    
    float AccAverage;

    // Velo
    float PreviousVelocity;
    float VelocityValue; 
};

#endif
