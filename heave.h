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
#define TENDENZCOUNTER 50

class heave
{
public:
    heave();
    ~heave(){};
    double GetHeave(float acc, float PitchAccHead,float RollAccHead, float dt);
    double ProcessIMU_data_heave(float axi, float ayi, float azi, float gx, float gy, float gz);
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

    // Tendenz begutachtung
    uint8_t Postitivdiff[TENDENZCOUNTER];
    uint8_t Negativdiff[TENDENZCOUNTER];
    uint8_t diffcounter;

    // from https://github.com/bareboat-necessities/bbn-wave-period-esp32/
    unsigned long now, last_refresh, start_time, last_update, last_update_k;
    unsigned long got_samples = 0;
    bool kalm_w_first, kalm_w_alt_first, kalm_smoother_first;
    float t;
    float freq_good_est;
    float twoKp;
    float twoKi;        
};

#endif
