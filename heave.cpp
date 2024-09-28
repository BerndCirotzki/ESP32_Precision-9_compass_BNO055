//*****************************************************************************
//
// File Name     : 'heave.cpp'
// Improoved by  : Bernd Cirotzki

//*****************************************************************************

#include "heave.h"

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

    for(i=0;i<ACCSMOOTHCOUNTER;i++) { AccSmooth[i] = 0; }
    AccSmooth_index = 0;
    HaveGravityAverage = false;
    for (i=0;i<GRAVITYCOUNTER;i++){ Gravity[i] = 0; }
    Gravity_index = 0;      
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
   float Heaveoffset = (Heave_Max_positiv + Heave_Max_negativ) / 2;
   if (abs(Heaveoffset) > 2.0)
       return (heavein * 0.93 - Heaveoffset * 0.07);
   if (abs(Heaveoffset) > 1.5)
       return (heavein * 0.94 - Heaveoffset * 0.06);
   if (abs(Heaveoffset) > 1.0)
       return (heavein * 0.95 - Heaveoffset * 0.05);   
   if (abs(Heaveoffset) > 0.5)
       return (heavein * 0.96 - Heaveoffset * 0.04);
   if (abs(Heaveoffset) > 0.25)
       return (heavein * 0.97 - Heaveoffset * 0.03);
   if (abs(Heaveoffset) > 0.1)
       return (heavein * 0.98 - Heaveoffset * 0.02);
   if (abs(Heaveoffset) > 0.05)
       return (heavein * 0.99 - Heaveoffset * 0.01); 
   return (heavein * 0.992 - Heaveoffset * 0.008);
}
