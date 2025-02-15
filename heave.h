//*****************************************************************************
//
// File Name     : 'heave.h'
// Improoved by  : Bernd Cirotzki

/*
  1Hz

$GPHEV,H,*hh<CR><LF>
Description
1 $: Talker identifier
2 GPHEV: Datagram identifier
3 H: Heave (Metres)
4 *hh: Checksum
Related topics
Supported datagram formats, page 12
Datagram formats, page 25
Third-party datagram formats, page 71

Here are a few more Heave examples from my boat (using YNDU RAW)
10:50:07.240 R 0DF1140A 00 11 00 FF FF FF FE FF
10:50:07.339 R 0DF1140A 00 11 00 FF FF FF 7F FD
10:50:07.442 R 0DF1140A 00 11 00 FF FF FF FE FF
10:50:07.540 R 0DF1140A 00 11 00 FF FF FF 7F FD
10:50:07.690 R 0DF1140A 00 11 00 FF FF FF 7F FD
10:50:07.739 R 0DF1140A 00 11 00 FF FF FF 7F FD
10:50:07.801 R 0DF1140A 00 11 00 FF FF FF FE FF
10:50:09.708 R 0DF1140A 00 0F 00 FF FF FF FF FF
10:50:09.101 R 0DF1140A 00 0F 00 FF FF FF 7F FD

 */

#ifndef HEAVE_H
#define HEAVE_H

// To Put out debug to Serial
//#define DEBUG 1

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
