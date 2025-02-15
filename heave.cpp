//*****************************************************************************
//
// File Name     : 'heave.cpp'
// Improoved by  : Bernd Cirotzki

//*****************************************************************************

#include "heave.h"

#include "AranovskiyFilter.h"
#include "KalmanSmoother.h"
#include "TrochoidalWave.h"
#include "Mahony_AHRS.h"
#include "Quaternion.h"
#include "MinMaxLemire.h"
#include "KalmanForWave.h"
#include "KalmanForWaveAlt.h"
#include "WaveFilters.h"

#ifdef DEBUG
extern float AccNew;
extern float AccDurch;
extern float AddVeloG;
extern float PVelocity;
extern float ACC_DURCH;
extern float VELO_DURCH;
extern float HEAVE_DURCH;
extern float HEAVE_MAX;
extern float HEAVE_MIN;
#endif

MinMaxLemire min_max_h;
AranovskiyParams arParams;
AranovskiyState arState;
KalmanSmootherVars kalman_freq;
Mahony_AHRS_Vars mahony;
KalmanWaveState waveState;
KalmanWaveAltState waveAltState;

heave::heave()
{
    int i;

    HeaveValue = 0.0;
	  GravityAverage = 0.0;
    HaveGravityAverage = false;
    Heave_vorher = 0;
    Heave_Max_positiv = 0;
    Heave_Max_negativ = 0;
    PreviousVelocity = 0.0;
    VelocityValue = 0.0;
    
    now = 0UL; last_refresh = 0UL; last_update_k = 0UL;
    got_samples = 0;
    kalm_w_first = true; kalm_w_alt_first = true; kalm_smoother_first = true;

    t = 0.0;
    freq_good_est = 0.0;
        
    twoKp = (2.0f * 4.0f);
    twoKi = (2.0f * 0.0001f);
    mahony_AHRS_init(&mahony, twoKp, twoKi);
    
    init_filters(&arParams, &arState, &kalman_freq);
    start_time = micros();
    last_update = start_time; 

    for(i=0;i<ACCSMOOTHCOUNTER;i++) { AccSmooth[i] = 0; }
    AccSmooth_index = 0;
    HaveGravityAverage = false;
    for (i=0;i<GRAVITYCOUNTER;i++){ Gravity[i] = 0; }
    Gravity_index = 0;
    for (diffcounter = 0; diffcounter < TENDENZCOUNTER; diffcounter++)
    {
      Postitivdiff[diffcounter] = 0;
      Negativdiff[diffcounter] = 0;
    }
    diffcounter = 0;
}

bool heave::MakeGravity(float acc)
{
   if (acc == 0)      // Not valid
      return false;
         
   int i;
   HaveGravityAverage = true;   
   
   GravityAverage = 0;
   for (i=0;i<GRAVITYCOUNTER;i++)
   { 
      if (Gravity[i] == 0) 
      {
         HaveGravityAverage = false;
         break;
      }
      GravityAverage += Gravity[i];
    }
    Gravity[Gravity_index] = acc;
    Gravity_index++;
    if (Gravity_index >= GRAVITYCOUNTER) Gravity_index = 0;
    AccValue = acc - (GravityAverage / GRAVITYCOUNTER);    
    return HaveGravityAverage;
}

double heave::GetHeave(float acc, float PitchAccHead,float RollAccHead, float dt)
{
  if (!MakeGravity(acc)) return N2kDoubleNA;
  float Addvelo = 0;
  
    // Correction of Component tolerance BNO055 (make mesurement to get the values)   
    if (PitchAccHead > 5 )
      AccValue = AccValue - PitchAccHead * 0.002;
    if (PitchAccHead < -5 )
      AccValue = AccValue - PitchAccHead * 0.0001;
    if (RollAccHead > 10 )
      AccValue = AccValue - RollAccHead * 0.002; 
    if (RollAccHead < -5 )
      AccValue = AccValue - RollAccHead * 0.002;
 
    AccSmooth[AccSmooth_index] = AccValue;
    AccSmooth_index++;
    if (AccSmooth_index >= ACCSMOOTHCOUNTER) AccSmooth_index = 0;
    for (int i = 0; i < ACCSMOOTHCOUNTER; i++)
    {
       AccAverage = AccAverage + AccSmooth[i];
    }
    AccAverage = AccAverage / ACCSMOOTHCOUNTER;        
    // Integration der Beschleunigung zur Geschwindigkeit in m/s
    Addvelo = AccAverage * dt;   // Geschwindigkeiszuwachs
    if (abs(Addvelo) < 0.01) Addvelo = 0;
    VelocityValue = Addvelo + PreviousVelocity * 0.98;
    if (abs(VelocityValue) < 0.03) VelocityValue = 0;
    PreviousVelocity = VelocityValue;
#ifdef DEBUG
AccNew = AccValue;
AccDurch = AccAverage;
AddVeloG = Addvelo;
PVelocity = PreviousVelocity;
#endif 
    HeaveValue = HeaveValue + (PreviousVelocity  * dt);
    HeaveValue = CheckHeave(HeaveValue);
    return HeaveValue;      
}

float heave::CheckHeave(float heavein)
{
    if (Heave_vorher > heavein) // Es geht abwärts
    {      
      if(Abwaerts == false)   // Neu abwärtz
      {  
          Abwaerts = true;
          Aufwaerts = false;
          Heave_Max_positiv = heavein;   
          Last_Heave_Max_positiv_Time = millis();    
      }
      else
      {
          if ((Last_Heave_Max_negativ_Time + 15000) < millis())  // Runs longer than 10 seconds down
          {
              Heave_Max_negativ = heavein; 
              Last_Heave_Max_negativ_Time = millis();
              Last_Heave_Max_positiv_Time = millis() + 7000;             
          }
      }
    }
    if (Heave_vorher < heavein)
    {
      if(Aufwaerts == false)  // Es geht aufwärts
      {
        Abwaerts = false;
        Aufwaerts = true;
        Heave_Max_negativ = heavein; 
        Last_Heave_Max_negativ_Time = millis();
      }
      else
      {
          if ((Last_Heave_Max_positiv_Time + 15000) < millis())  // Runs longer than 10 seconds up
          {
              Heave_Max_positiv = heavein; 
              Last_Heave_Max_positiv_Time = millis();
              Last_Heave_Max_negativ_Time = millis() + 7000;              
          } 
      }
    }
    if (Heave_vorher == heavein)
    {
          if ((Last_Heave_Max_negativ_Time + 15000) < millis())  // Runs longer than 10 seconds down
          {
              Heave_Max_negativ = heavein; 
              Last_Heave_Max_negativ_Time = millis();
              Last_Heave_Max_positiv_Time = millis() + 5000;             
          }
          else if ((Last_Heave_Max_positiv_Time + 15000) < millis())  // Runs longer than 10 seconds up
          {
              Heave_Max_positiv = heavein; 
              Last_Heave_Max_positiv_Time = millis();
              Last_Heave_Max_negativ_Time = millis() + 5000;              
          }
    }
    Heave_vorher = heavein;
#ifdef DEBUG
HEAVE_MAX = Heave_Max_positiv;
HEAVE_MIN = Heave_Max_negativ;
#endif
   if(++diffcounter >= TENDENZCOUNTER) diffcounter = 0;
   float Heaveoffset = (Heave_Max_positiv + Heave_Max_negativ) / 2;
   if (Heaveoffset > 0)
   {
       Postitivdiff[diffcounter] = 1;
       Negativdiff[diffcounter] = 0;
   }
   else
   {
       Postitivdiff[diffcounter] = 0;
       Negativdiff[diffcounter] = 1;
   }
   uint8_t positiv = 0;
   uint8_t negativ = 0;
   for(uint8_t f = 0; f < TENDENZCOUNTER;f++)
   {
       positiv += Postitivdiff[f];
       negativ += Negativdiff[f];
   }
   if (positiv > negativ)
      Heaveoffset += 0.005;
   if (positiv < negativ)
      Heaveoffset -= 0.005;   
   if (abs(Heaveoffset) > 2.0)
       return (heavein * 0.93 - Heaveoffset * 0.07);
   if (abs(Heaveoffset) > 1.5)
       return (heavein * 0.94 - Heaveoffset * 0.06);
   if (abs(Heaveoffset) > 1.0)
       return (heavein * 0.95 - Heaveoffset * 0.05);   
   if (abs(Heaveoffset) > 0.5)
       return (heavein * 0.955 - Heaveoffset * 0.045);
   if (abs(Heaveoffset) > 0.25)
       return (heavein * 0.957 - Heaveoffset * 0.043);
   if (abs(Heaveoffset) > 0.15)
       return (heavein * 0.96 - Heaveoffset * 0.04);
   if (abs(Heaveoffset) > 0.12)
       return (heavein * 0.97 - Heaveoffset * 0.03);        
   if (abs(Heaveoffset) > 0.1)
       return (heavein * 0.98 - Heaveoffset * 0.02);
   if (abs(Heaveoffset) > 0.05)
       return (heavein * 0.99 - Heaveoffset * 0.01); 
   return (heavein * 0.992 - Heaveoffset * 0.008);
}

double heave::ProcessIMU_data_heave(float axi, float ayi, float azi, float gx, float gy, float gz) 
{
  got_samples++;
  now = micros();
  float ax = axi / 9.81;
  float ay = ayi / 9.81;
  float az = azi / 9.81;
    
  if ((ax * ax + ay * ay + az * az) < ACCEL_MAX_G_SQUARE) {
    // ignore noise (in unbiased way) with unreasonably high Gs

    float delta_t = (now - last_update) / 1000000.0;  // time step sec
    last_update = now;

    float mypitch, myroll, myyaw;
    Quaternion quaternion;

    mahony_AHRS_update(&mahony, gx , gy , gz , ax, ay, az, &mypitch, &myroll, &myyaw, delta_t);
    Quaternion_set(mahony.q0, mahony.q1, mahony.q2, mahony.q3, &quaternion);    

    float v[3] = {ax, ay, az};
    float rotated_a[3];
    Quaternion_rotate(&quaternion, v, rotated_a);

    float  accel_rotated[3];
    accel_rotated[0] = rotated_a[0];
    accel_rotated[1] = rotated_a[1];
    accel_rotated[2] = rotated_a[2];

    float a = (accel_rotated[2] - 1.0);  // acceleration in fractions of g
    
    if (kalm_w_first) {
      kalm_w_first = false;
      float k_hat = - pow(2.0 * PI * FREQ_GUESS, 2);
      waveState.displacement_integral = 0.0f;
      waveState.heave = a * g_std / k_hat;
      waveState.vert_speed = 0.0f;               // ??
      waveState.accel_bias = 0.0f;
      kalman_wave_init_state(&waveState);
    }
    kalman_wave_step(&waveState, a * g_std, delta_t);

    double freq_adj = 0.0;
    if (t > warmup_time_sec(true)) {
      // give some time for other filters to settle first
      aranovskiy_update(&arParams, &arState, waveState.heave / ARANOVSKIY_SCALE, delta_t);
      if (kalm_smoother_first) {
        kalm_smoother_first = false;
        kalman_smoother_set_initial(&kalman_freq, arState.f);
      }
      freq_adj = kalman_smoother_update(&kalman_freq, arState.f);
    }

    if (isnan(freq_adj)) {
      // reset filters
      kalm_w_first = true;
      kalm_w_alt_first = true;
      kalm_smoother_first = true;
      init_filters(&arParams, &arState, &kalman_freq);
      start_time = micros();
      last_update = start_time;
      t = 0.0;
    }
    else if (freq_adj > FREQ_LOWER && freq_adj < FREQ_UPPER) { /* prevent decimal overflows */
      double period = 1.0 / freq_adj;
      uint32_t windowMicros = getWindowMicros(period);
      SampleType sample = { .value = waveState.heave, .timeMicroSec = now };
      min_max_lemire_update(&min_max_h, sample, windowMicros);

      if (fabs(arState.f - freq_adj) < FREQ_COEF_TIGHT * freq_adj) {  /* sanity check of convergence for freq */
        freq_good_est = freq_adj;
      }

      // use previous good estimate of frequency
      if (fabs(arState.f - freq_good_est) < FREQ_COEF * freq_good_est) {
        float k_hat = - pow(2.0 * PI * freq_good_est, 2);
        if (kalm_w_alt_first) {
          kalm_w_alt_first = false;
          waveAltState.displacement_integral = 0.0f;
          waveAltState.heave = waveState.heave;
          waveAltState.vert_speed = waveState.vert_speed;
          waveAltState.vert_accel = k_hat * waveState.heave; //a * g_std;
          waveAltState.accel_bias = 0.0f;
          kalman_wave_alt_init_state(&waveAltState);
        }
        float delta_t_k = last_update_k == 0UL ? delta_t : (now - last_update_k) / 1000000.0;
        kalman_wave_alt_step(&waveAltState, a * g_std, k_hat, delta_t_k);
        last_update_k = now;
      }

      if (now - last_refresh >= 1000000) {
        last_refresh = now;
        got_samples = 0;
      }
    }
  }
  t = (now - start_time) / 1000000.0;  // time since start sec
  if (!HaveGravityAverage)  // from my Logic, but this time must be waited
     return N2kDoubleNA;
  if (waveState.heave < 15)
    return (double) (waveState.heave);
  else
    return (double) N2kDoubleNA;
}
