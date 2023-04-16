//
// NMEA 2000 Kurssensor 
// Bernd Cirotzki 2022 -
//

#define ESP32_CAN_TX_PIN GPIO_NUM_32  // Set CAN TX port TX to TX !!
#define ESP32_CAN_RX_PIN GPIO_NUM_34  // Set CAN RX port RX to RX !!

#define NMEA2000_DEVICEID 65
#define DEV_COMPASS 0 // 60-140

#include <Arduino.h>
#include <EEPROM.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <EEPROM.h>  
#include <esp_task_wdt.h>
#include <Time.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA2000_CAN.h>
#include "BluetoothStream.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//5 seconds WDT127237
#define WDT_TIMEOUT 40

// To Put out debug to Serial
//#define DEBUG 1

// Offset set by Plotter
enum EEP_ADDR
{
    EEP_CALIB_FLAG = 0x00,   
    EEP_HEADING_OFFSET = 0x01,
    EEP_HEEL_OFFSET = 0x02,
    EEP_TRIM_OFFSET = 0x03,
    EEP_AUTOCALIBRATION = 0x04,
    ADDR_ACC_X = 0x05,                  // To get the right floor value
    ADDR_ACC_Y = 0x09,
    ADDR_ACC_Z = 0x0D,
    ADDR_SEND_HEADING = 0x11,
    ADDR_BNO055_CONFIG = 0x12 // address of BNO055Offset
};

// B&G calibration stop/start
bool calibrationStart = false;
bool calibrationStop = false;
bool OnlyMag = false;

float heading, heading_true,
      heading_offset_rad = 0,
      heel_offset_rad = 0, 
      trim_offset_rad = 0; 
int heading_offset_deg = 0,      
    heel_offset_deg = 0,
    trim_offset_deg = 0;
bool send_heading = true;                           // to do 20 vs 10hz

unsigned char compass_autocalibration = 0x00;


// List here messages your device will transmit.
const unsigned long TransmitMessagesCompass[] PROGMEM = { 127250L, 127251L, 127257L , 0 }; //Vessel Heading, Rate of Turn, Attitude
const unsigned long ReceiveMessages[] PROGMEM = { 127258L, // Magnetic variation
                                                  130845L, // B&G Config from Plotter
                                                  130850L  // B&G Calibration start
                                                };
                                                
int DEVICE_ID = 65;  // globale Definitionen. NMEA2000
int SID = 0;
tN2kMsg N2kMsg;
tN2kMsg N2kMsgReply;

const double radToDeg = 180.0 / M_PI;
const double degToRad= M_PI / 180.0;

// globale Definitionen für BNO055 Sensor
// Acc
float PitchAcc;
float RollAcc;
float PitchAccold=0;
float PitchAccnew;

// Gyo
float dt;
unsigned long millisOld;

// ACC
float acc_x, acc_y, acc_z;
float acc_x_ground, acc_y_ground, acc_z_ground;


// Pitch und Roll sind falsch herum ... bei Schiff
float Pitch;   // Best of both worlds
float Roll; 
float PitchRad;
float RollRad;

float PitchAccHead=0;
float RollAccHead=0;
float PitchHead=0;
float RollHead=0;

// Magnetic Values
float MagX_uT,MagX_uT_old = 0;
float MagY_uT,MagY_uT_old = 0;
float MagZ_uT,MagZ_uT_old = 0;

// Heading berechnung
float Xm;
float Ym;
float XmNew;
float YmNew;
float XmOld;
float YmOld;
#ifdef DEBUG
float Heading; // only use filtered Heading
#endif
float HeadingFiltered;
double HeadingVariation = N2kDoubleNA;
double EnteredVariation = N2kDoubleNA;
unsigned long LastVariation;

// rate of Turn
float RateofTurn;
float RateofTurnOld = 0;

#define BNO055_SAMPLERATE_DELAY_MS 50

unsigned long SampleTimer;
unsigned long lastTimeCaltext;
unsigned long SensorStartMillis;

// Function addeed in Class Adafruit Library
bool Adafruit_BNO055::getSensorMagneticOffsets(adafruit_bno055_offsets_t &offsets_type)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);
    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    offsets_type.mag_offset_x =
        (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
    offsets_type.mag_offset_y =
        (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
    offsets_type.mag_offset_z =
        (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));
    /* Magnetometer radius = +/- 960 LSB */
    offsets_type.mag_radius =
        (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

    setMode(lastMode);
    return true;
}

Adafruit_BNO055 BNO055 =  Adafruit_BNO055();

adafruit_bno055_offsets_t BNO055Offset;

const uint16_t EEPROM_SIZE = (18 + sizeof(adafruit_bno055_offsets_t) + 363 + 2 * 360 + 1);
const uint16_t ADDR_Deviationtabel = 19 + sizeof(adafruit_bno055_offsets_t);
const uint16_t ADDR_Deviationtabel_copy1 = ADDR_Deviationtabel + 363;
const uint16_t ADDR_Deviationtabel_copy2 = ADDR_Deviationtabel_copy1 + 360;

BluetoothStream *pBlueTooth;
uint8_t LastMAGCal = 0;
unsigned long LastCOG = millis();
double DEGCOG = N2kDoubleNA;
double SOG;
bool MakeDeviationtabel = false;
unsigned long LastMakeTabelOutout;
bool Derr1 = false;
bool Derr2 = false;
bool SendHeadingwithDeviation;
bool showDiv = false;
unsigned long LastShowdiv;
                                         
void setup() 
{
  pinMode(18, OUTPUT); // Set GPIO18 as digital output   to BNO Reset PIN
  digitalWrite(18, HIGH); // Set GPIO18 active high
  Wire.begin(16,17);
  Serial.begin(115200);  
  Serial.println("Start Kurssensor ...");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  delay(100);
  pBlueTooth = new BluetoothStream();
  delay(3500);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector
  pBlueTooth->SendString("Start Kurssensor ...");
  Serial.println("EEPROM start");
  if (!EEPROM.begin(EEPROM_SIZE))
  {
      Serial.println("EEPROM start failed");
      pBlueTooth->SendString("EEPROM start failed\n");
  }
  // Settings for BNO055
  BNO055.begin(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF);
  delay(500);
  BNO055.enterNormalMode();
  delay(500);
  // Get Calibrations
  acc_x_ground = 0;
  acc_y_ground = 0;
  acc_z_ground = 0;
  if (loadCalibration())
  {
     Serial.println("Sensor - Calibration sucessfully read from EEPROM");
     pBlueTooth->SendString("Sensor - Calibration sucessfully read from EEPROM\n");
     BNO055.setSensorOffsets(BNO055Offset);
     calibrationStart = false;   // Set to Calibration is OK. The GetCalibration is swinging
     calibrationStop = true;
     delay(100);
  }
  else
  {
     Serial.println("No Sensor - Calibration from EEPROM");  
     pBlueTooth->SendString("No Sensor - Calibration from EEPROM\n");
     calibrationStart = false; 
     calibrationStop = false;      
  }
  if (loadGroundCalibration())
  {
     Serial.println("Ground - Calibration sucessfully read from EEPROM");
     pBlueTooth->SendString("Ground - Calibration sucessfully read from EEPROM\n");
  }
  else
  {
     Serial.println("No Ground - Calibration from EEPROM");
     pBlueTooth->SendString("No Ground - Calibration from EEPROM\n");
  }
  if(EEPROM.readByte(ADDR_Deviationtabel + 361) != 0)
  {
     Serial.println("Send Heading with Deviation");
     pBlueTooth->SendString("Send Heading with Deviation\n");
     SendHeadingwithDeviation = true;
  }
  else
  {    
     Serial.println("Not Send Heading with Deviation");  
     pBlueTooth->SendString("Not Send Heading with Deviation\n");
     SendHeadingwithDeviation = false;
  }
  delay(500);  
  send_heading=true;
  int8_t temp=BNO055.getTemp();
  BNO055.setExtCrystalUse(true);  // Sets the external Crystal for use
  
  // NMEA 2000 Settings
  NMEA2000.SetProductInformation("107018103", // Manufacturer's Model serial code
                                 13233, // Manufacturer's product code
                                 "Precision-9 Compass",  // Manufacturer's Model ID
                                 "2.0.3-0",  // Manufacturer's Software version code
                                 "", // Manufacturer's Model version
                                 1,  // load equivalency *50ma
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_COMPASS
                                );
 
  // Set device information
  NMEA2000.SetDeviceInformation(1048678, // Unique number. Use e.g. Serial number.
                                140, // Device function=Temperature See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                60, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                275, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4,
                                DEV_COMPASS
                               );
  NMEA2000.SetN2kCANMsgBufSize(20);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetForwardSystemMessages(false);
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,DEVICE_ID);
  NMEA2000.EnableForward(false);
  if(false == NMEA2000.Open()) 
    Serial.println("NMEA2000 not Ready");
  else
    Serial.println("NMEA2000 is Ready");
  NMEA2000.SendProductInformation();
  millisOld = millis();
  lastTimeCaltext = millis();
  SensorStartMillis = millis();
  esp_task_wdt_init(WDT_TIMEOUT, false); //enable panic so ESP32 restarts  ... muss false Sein, damit wirklicher Neustart... kein Panic. auf treue zum debuggen.
  esp_task_wdt_add(NULL); //add current thread to WDT watch  ... ESP.restart()
}

bool GetBNO055Values()
{
  if (SampleTimer + BNO055_SAMPLERATE_DELAY_MS <= millis())                  // Set timer
  {
    SampleTimer = millis();  
#ifdef DEBUG    
    uint8_t Calsystem, Calgyro, Calaccel, Calmag = 0;
    BNO055.getCalibration(&Calsystem, &Calgyro, &Calaccel, &Calmag);
#endif    
    imu::Vector<3> acc = BNO055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = BNO055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = BNO055.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
     
    //  Acc
    acc_x = acc.x() - acc_x_ground;
    acc_y = acc.y() - acc_y_ground;
    acc_z = acc.z() - acc_z_ground;

    if(acc.x() == 0 && acc.y() == 0 && acc.z() == 0 && (millis() - SensorStartMillis) > 5000)
    {
      Serial.println("No Data from Sensor. Reset.");
      digitalWrite(18, LOW); // Set GPIO18 
      delay(900);
      digitalWrite(18, HIGH); // Set GPIO18
      delay(400);
      //BNO055.enterSuspendMode(); // Old Reset
      //delay(400);
      ESP.restart(); //call reset
    } 
    PitchAcc =-atan2(acc_x/9.81,acc_z/9.81) * radToDeg;
    RollAcc=-atan2(acc_y/9.81,acc_z/9.81) * radToDeg;
    
    // Gyro
    dt = (millis()-millisOld)/1000.;
    millisOld = millis();
    Pitch = (Pitch+gyr.y()*dt) * 0.90 + PitchAcc * 0.1;
    Roll = (Roll-gyr.x()*dt) * 0.90 + RollAcc * 0.1;
        
    // Rate of turn
    RateofTurn = gyr.z() * 0.3 + RateofTurnOld * 0.7;
    
    // Now everthing New, because the Heading is to the state of the Chip
    PitchAccHead = -atan2(acc.x()/9.81,acc.z()/9.81) * radToDeg;
    RollAccHead  = -atan2(acc.y()/9.81,acc.z()/9.81) * radToDeg;
     
    PitchHead = (PitchHead+gyr.y()*dt) * 0.01 + PitchAccHead * 0.99;
    RollHead  = (RollHead-gyr.x()*dt) * 0.01 + RollAccHead * 0.99;
    
    PitchRad = PitchHead * degToRad + trim_offset_rad;
    RollRad  = RollHead * degToRad + heel_offset_rad;

    // Heading berechnen
    // with tilt compensated ... standard formel.
    MagX_uT = mag.x() * 0.99 + MagX_uT_old * 0.01;
    MagY_uT = mag.y() * 0.99 + MagY_uT_old * 0.01;
    MagZ_uT = mag.z() * 0.99 + MagZ_uT_old * 0.01;
    MagX_uT_old = MagX_uT;
    MagY_uT_old = MagY_uT;
    MagZ_uT_old = MagZ_uT;
    
    Xm=MagX_uT*cos(PitchRad)-MagY_uT*sin(RollRad)*sin(PitchRad)+MagZ_uT*cos(RollRad)*sin(PitchRad);
    Ym=MagY_uT*cos(RollRad)+MagZ_uT*sin(RollRad);
    
    //Xm=mag.x()*cos(PitchRad)-mag.y()*sin(RollRad)*sin(PitchRad)+mag.z()*cos(RollRad)*sin(PitchRad);
    //Ym=mag.y()*cos(RollRad)+mag.z()*sin(RollRad);
    
    XmNew=0.9*XmOld + 0.1*Xm;
    YmNew=0.9*YmOld + 0.1*Ym;
#ifdef DEBUG
    Heading=atan2(Ym,Xm) * radToDeg;
    if (Heading < 0) Heading += 360.0;
#endif
    HeadingFiltered=atan2(YmNew,XmNew) * radToDeg;
    HeadingFiltered -= heading_offset_deg;  // Set from Plotter
    if (HeadingFiltered < 0) HeadingFiltered += 360.0;
    if (HeadingFiltered >= 360) HeadingFiltered -= 360.0;
    
    RateofTurnOld=RateofTurn;
    XmOld=XmNew;
    YmOld=YmNew; 
#ifdef DEBUG
    Serial.print(acc.x()/9.8*10);
    Serial.print(",");
    Serial.print(acc.y()/9.8*10);
    Serial.print(",");
    Serial.print(acc.z()/9.8*10);
    Serial.print(",");
    Serial.print(gyr.x());
    Serial.print(",");
    Serial.print(gyr.y());
    Serial.print(",");
    Serial.print(gyr.z());
    Serial.print(",");
    Serial.print(Calsystem);
    Serial.print(",");
    Serial.print(PitchAcc);
    Serial.print(",");
    Serial.print(RollAcc);
    Serial.print(",");
    Serial.print(mag.x());
    Serial.print(",");
    Serial.print(mag.y());
    Serial.print(",");
    Serial.print(mag.z());
    Serial.print(",");
    Serial.print(-Roll);
    Serial.print(",");
    Serial.print(Heading);
    Serial.print(",");
    Serial.println(HeadingFiltered);
#endif
    return true; 
  }
  return false;
}

void loop()
{ 
  CheckForCalibration(); // Only returns, if nothing must be done 
  if (GetBNO055Values()) // returns true if New data
  {
     if (MakeDeviationtabel == true)
     {
          MakeBoatDeviationtable();
     }
     else
     {
      if (send_heading)
      {
        if (0 != EEPROM.readByte(ADDR_SEND_HEADING))
        {
          N2kMsg.Clear();
          if(SendHeadingwithDeviation)
            SetN2kPGN127250(N2kMsg, SID, (GetHeadingwithDeviation(HeadingFiltered) * degToRad), N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
          else
            SetN2kPGN127250(N2kMsg, SID, (HeadingFiltered * degToRad), GetDeviation(HeadingFiltered), GetVariation(), N2khr_magnetic);
          NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
        }
        // Rate of Turn
        N2kMsg.Clear();
        SetN2kRateOfTurn(N2kMsg, SID, -(RateofTurn * degToRad)); // radians 
        NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
      }
      send_heading = !send_heading;
      SetN2kAttitude(N2kMsg, SID, N2kDoubleNA, -((Pitch * degToRad) - trim_offset_rad), -((Roll * degToRad) - heel_offset_rad));
      NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
      SID++; if (SID > 250) SID = 1;
     }
  }
  NMEA2000.ParseMessages();  
   // Watchdog reset !
  if(LastCOG + 2000 < millis())
       DEGCOG = N2kDoubleNA;
  if (LastVariation + 5000 < millis())
       HeadingVariation = N2kDoubleNA;
  esp_task_wdt_reset();
}

double GetVariation()
{
    double Variation = HeadingVariation;
    if (EnteredVariation != N2kDoubleNA)
        Variation = EnteredVariation;
    if (Variation != N2kDoubleNA)
        Variation *= degToRad;
    return Variation;    
}

double GetDeviation(float HeadingFiltered)
{
    int D;
    int8_t DHv, DLv;
    uint8_t j,i;
    char dummy[10];
    D = (int)round(HeadingFiltered);
    if(D >= 360) D -= 360;
    int8_t v = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
    if (v == 0x7F)  // Not set  try to interpolier
    {
       for (j = 1; j < 40; j++)
       {
          D = (int)round(HeadingFiltered) - j;
          if(D < 0) D += 360;
          DLv = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
          if (DLv != 0x7F)
             break;          
       }
       for (i = 1; i < 40; i++)
       {
          D = (int)round(HeadingFiltered) + i;
          if(D >= 360) D -= 360;
          DHv = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
          if (DHv != 0x7F)
             break;          
       }
       if (DHv != 0x7F && DLv != 0x7F)
       {
          v = (int8_t)((double)DLv *(double)((double)i/((double)j+(double)i))  + (double)DHv * (double)((double)j/((double)j+(double)i)));
          if(showDiv && LastShowdiv + 400 < millis())
          {
            LastShowdiv = millis();            
            sprintf(dummy,"%3.1f",HeadingFiltered);
            pBlueTooth->SendString(" Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            /*sprintf(dummy,"%i",i);
            pBlueTooth->SendString(" i:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",j);
            pBlueTooth->SendString(" j:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",DLv);
            pBlueTooth->SendString(" DLv:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",DHv);
            pBlueTooth->SendString(" DHv:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",v);
            pBlueTooth->SendString(" v:");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");*/
            
            sprintf(dummy,"%2.1f",(float) ((int8_t)v / (double)10));
            pBlueTooth->SendString("Deviation :");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" (interpoliert)\n");
          }
          return (((double)((int8_t)v / (double)10.0)) * degToRad);
       }
       else
       {
          if(showDiv && LastShowdiv + 400 < millis())
          {
            LastShowdiv = millis();
            sprintf(dummy,"%3.1f",HeadingFiltered);
            pBlueTooth->SendString(" Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            pBlueTooth->SendString("No Deviation.\n");
          }  
       }
       return N2kDoubleNA;
    }
    if(showDiv && LastShowdiv + 400 < millis())
    {
        LastShowdiv = millis();
        sprintf(dummy,"%3.1f",HeadingFiltered);
        pBlueTooth->SendString(" Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
        sprintf(dummy,"%2.1f",(float) ((int8_t)v / (double)10));
        pBlueTooth->SendString("Deviation :");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");    
    }
    return (((double)((int8_t)v / (double)10.0)) * degToRad);
}

float GetHeadingwithDeviation(float HeadingFiltered)
{
    int D;
    int8_t DHv, DLv;
    uint8_t j,i;
    float HeadingReturn;
    char dummy[10];
    D = (int)round(HeadingFiltered);
    if(D >= 360) D -= 360;
    int8_t v = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
    if (v == 0x7F)  // Not set  try to interpolier
    {
       for (j = 1; j < 40; j++)
       {
          D = (int)round(HeadingFiltered) - j;
          if(D < 0) D += 360;
          DLv = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
          if (DLv != 0x7F)
             break;          
       }
       for (i = 1; i < 40; i++)
       {
          D = (int)round(HeadingFiltered) + i;
          if(D >= 360) D -= 360;
          DHv = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
          if (DHv != 0x7F)
             break;          
       }
       if (DHv != 0x7F && DLv != 0x7F)
       {
          v = (int8_t)((double)DLv *(double)((double)i/((double)j+(double)i))  + (double)DHv * (double)((double)j/((double)j+(double)i)));
          HeadingReturn = HeadingFiltered + (float) ((int8_t)v / (double)10);
          if (HeadingReturn < 0) HeadingReturn += 360;
          if (HeadingReturn >= 360) HeadingReturn -= 360;
          if(showDiv && LastShowdiv + 400 < millis())
          {
            LastShowdiv = millis();            
            sprintf(dummy,"%3.1f",HeadingFiltered);
            pBlueTooth->SendString("(send with Deviation) old Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            /*sprintf(dummy,"%i",i);
            pBlueTooth->SendString(" i:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",j);
            pBlueTooth->SendString(" j:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",DLv);
            pBlueTooth->SendString(" DLv:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",DHv);
            pBlueTooth->SendString(" DHv:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            sprintf(dummy,"%i",v);
            pBlueTooth->SendString(" v:");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");*/            
            sprintf(dummy,"%2.1f",(float) ((int8_t)v / (double)10));
            pBlueTooth->SendString("Deviation :");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" (interpoliert) ");
            sprintf(dummy,"%3.1f",HeadingReturn);
            pBlueTooth->SendString("new Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");
          }          
          return (HeadingReturn);
       }
       else
       {
          if(showDiv && LastShowdiv + 400 < millis())
          {
            LastShowdiv = millis();
            sprintf(dummy,"%3.1f",HeadingFiltered);
            pBlueTooth->SendString(" Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            pBlueTooth->SendString("No Deviation.\n");
          }  
       }
       return HeadingFiltered;
    }
    HeadingReturn = HeadingFiltered + (float) ((int8_t)v / (double)10);
    if (HeadingReturn < 0) HeadingReturn += 360;
    if (HeadingReturn >= 360) HeadingReturn -= 360;
    if(showDiv && LastShowdiv + 400 < millis())
    {
        LastShowdiv = millis();
        sprintf(dummy,"%3.1f",HeadingFiltered);
        pBlueTooth->SendString("(send with Deviation) old Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
        sprintf(dummy,"%2.1f",(float) ((int8_t)v / (double)10));
        pBlueTooth->SendString("Deviation :");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
        sprintf(dummy,"%3.1f",HeadingReturn);
        pBlueTooth->SendString("new Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");    
    }    
    return (HeadingReturn);
}

void MakeBoatDeviationtable()
{
    if(DEGCOG == N2kDoubleNA)
    {
       Serial.println("COG lost");
       pBlueTooth->SendString("COG lost\n");
       MakeDeviationtabel = false;
       return;
    }
    if(HeadingVariation == N2kDoubleNA && EnteredVariation == N2kDoubleNA)
    {
       Serial.println("Variation lost");
       pBlueTooth->SendString("Variation lost\n");
       MakeDeviationtabel = false;
       return;
    }
    if(SOG < 2)
    {  
       if (LastMakeTabelOutout < millis())
       {
         Serial.println("Boat too slow");
         pBlueTooth->SendString("Boat too slow\n");
       }
       return;
    }
    double Variation = HeadingVariation;
    if (EnteredVariation != N2kDoubleNA)
        Variation = EnteredVariation;
    // COG = Heading + Variation + Deviation
    double Deviation = DEGCOG - HeadingFiltered - Variation;
    if ( Deviation < -12)
    { 
       Derr1 = true;       
       Deviation = -12;
    }
    if ( Deviation > 12)
    {
       Derr2 = true;
       Deviation = 12;
    }
    // Write to EEPROM
    int8_t v  = 0x7F;
    if (Deviation != 12 && Deviation != -12)   // Not when Deviation ist bad
       v = (int8_t) (Deviation * 10); // Kommastelle
    int D = (int)round(HeadingFiltered);
    if(D >= 360) D -= 360;
    int8_t ve = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
    if( ve != 0x7F) // Not first Use
    {
       if (v != 0x7F)   // Not when Deviation ist bad
          v = (int8_t) ((double) v * 0.9 + (double)ve * 0.1);
       else
          v = ve;       // Do not loose the old value
    }   
    EEPROM.writeByte(ADDR_Deviationtabel + D , (int8_t)v);
    if (LastMakeTabelOutout < millis())
    {
       char dummy[10];
       LastMakeTabelOutout = millis() + 500;
       sprintf(dummy,"%2.1f",Deviation);
       Serial.print("Set:");Serial.print(Deviation);Serial.print(" for Heading:");Serial.print(D);
       Serial.print(" for COG:");Serial.print((int)round(DEGCOG));
       pBlueTooth->SendString("Set:");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" for Heading:");
       sprintf(dummy,"%i",D); pBlueTooth->SendString(dummy);
       pBlueTooth->SendString(" / COG:");
       sprintf(dummy,"%i",(int)round(DEGCOG)); pBlueTooth->SendString(dummy);
       pBlueTooth->SendString("\n");
       if (Derr1)
       {
         Serial.println("Deviation < -12 do not set");
         pBlueTooth->SendString("Deviation < -12 do not set\n");
       }
       if (Derr2)
       {
         Serial.println("Deviation > 12 do not set");
         pBlueTooth->SendString("Deviation > 12 do not set\n");
       }
       Derr1 = false;
       Derr2 = false;
    }  
}

void CheckForCalibration()
{
  String Val;
  char dummy[10];
  uint8_t Calsystem, Calgyro, Calaccel, Calmag = 0;

  if(false == BNO055.isFullyCalibrated() && calibrationStart == false && calibrationStop == false)
  {
    if(lastTimeCaltext < millis())
    {
     lastTimeCaltext = millis() + 5000;
     Serial.println("Kursensor is not fully-calibrated. Enter \"cal\" to calibrate.");
     pBlueTooth->SendString("Kursensor is not fully-calibrated. Enter \"cal\" to calibrate.");
    }
  }
  if (Serial.available() || pBlueTooth->available())
  {
      if (Serial.available())
      {
        Val = Serial.readString();
        pBlueTooth->ConfigBluetooth(Val);
      }
      else
        pBlueTooth->ParseMessages(Val);
      pBlueTooth->SendString("\n");
      if(Val == String("help") )
      {
         Serial.println("reset     :  Reset Kurssensor.");
         Serial.println("sendhead  :  Switch Send Heading ON.");
         Serial.println("stophead  :  Switch Send heading OFF.");
         Serial.println("senddev   :  Send Heading with Deviation");
         Serial.println("stopdev   :  Do not Send Heading with Deviation. Send Deviation in the rigth field");
         Serial.println("cal       :  Start Sensor Calibration.");
         Serial.println("calmag    :  Start Sensor only Magnetic Calibration.");
         Serial.println("clearcal  :  Delete the Calibration.");
         Serial.println("setground :  Set the Ground Calibration.");
         Serial.println("setvari V :  Define Magnetic Variation V (this has prio to the Variation over NMEA2000) East = positiv");
         Serial.println("mcogdev   :  Make a Deviationtabel while turnig a Cycle");
         Serial.println("scogdev   :  Stop turning Mode.");
         Serial.println("coghead   :  Save one the Value at this time calculate from COG and Heading");
         Serial.println("hdev H D  :  Save Deviation Value D (NA = delete) at Heading H");
         Serial.println("devdiv D  :  Enter Deviation D for the momentan Heading");
         Serial.println("cleardev  :  delete Deviationtabel");
         Serial.println("printdev  :  show Deviationtabel Values");
         Serial.println("showdev   :  show Deviation Values used at the moment (turn off when aktiv)");
         Serial.println("copyd1    :  Save Deviationtabel to Storage Area 1");
         Serial.println("copyd2    :  Save Deviationtabel to Storage Area 2");
         Serial.println("restd1    :  Restore Deviationtabel from Storage Area 1");
         Serial.println("restd2    :  Restore Deviationtabel from Storage Area 2");
         pBlueTooth->SendString("reset     :  Reset Kurssensor.\n");
         pBlueTooth->SendString("sendhead  :  Switch Send Heading ON.\n");
         pBlueTooth->SendString("stophead  :  Switch Send heading OFF.\n");
         pBlueTooth->SendString("senddev   :  Send Heading with Deviation\n");
         pBlueTooth->SendString("stopdev   :  Do not Send Heading with Deviation. Send Deviation in the rigth field\n");
         pBlueTooth->SendString("cal       :  Start Sensor Calibration.\n");
         pBlueTooth->SendString("calmag    :  Start Sensor only Magnetic Calibration.\n");
         pBlueTooth->SendString("clearcal  :  Delete the Calibration.\n");
         pBlueTooth->SendString("setground :  Set the Ground Calibration.\n");
         pBlueTooth->SendString("setvari V :  Define Magnetic Variation V (this has prio to the Variation over NMEA2000) East = positiv\n");
         pBlueTooth->SendString("mcogdev   :  Make a Deviationtabel while turnig a Cycle\n");
         pBlueTooth->SendString("scogdev   :  Stop turning Mode.\n");
         pBlueTooth->SendString("coghead   :  Save one the Value at this time calculate from COG and Heading\n");
         pBlueTooth->SendString("hdev H D  :  Save Deviation Value D (NA = delete) at Heading H\n");
         pBlueTooth->SendString("defdev D  :  Enter Deviation D for the momentan Heading\n");
         pBlueTooth->SendString("cleardev  :  delete Deviationtabel\n");
         pBlueTooth->SendString("printdev  :  show Deviationtabel Values\n");
         pBlueTooth->SendString("showdev   :  show Deviation Values used at the moment (turn off when aktiv)\n");
         pBlueTooth->SendString("copyd1    :  Save Deviationtabel to Storage Area 1\n");
         pBlueTooth->SendString("copyd2    :  Save Deviationtabel to Storage Area 2\n");
         pBlueTooth->SendString("restd1    :  Restore Deviationtabel from Storage Area 1\n");
         pBlueTooth->SendString("restd2    :  Restore Deviationtabel from Storage Area 2\n");
      }
      if(Val == String("copyd1"))
      {
         int8_t v;
         for (uint16_t i = 0; i < 360;i++)
         {
             v = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + i);
             EEPROM.writeByte(ADDR_Deviationtabel_copy1 + i, v);
         }
         EEPROM.commit();
         Serial.println("Deviationtabel copy saved in StorageArea 1");
         pBlueTooth->SendString("Deviationtabel copy saved in StorageArea 1\n");
      }
      if(Val == String("copyd2"))
      {
         int8_t v;
         for (uint16_t i = 0; i < 360;i++)
         {
             v = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + i);
             EEPROM.writeByte(ADDR_Deviationtabel_copy2 + i, v);
         }
         EEPROM.commit();
         Serial.println("Deviationtabel copy saved in StorageArea 2");
         pBlueTooth->SendString("Deviationtabel copy saved in StorageArea 2\n");
      }
      if(Val == String("restd1"))
      {
         int8_t v;
         for (uint16_t i = 0; i < 360;i++)
         {
             v = (int8_t)EEPROM.readByte(ADDR_Deviationtabel_copy1 + i);
             EEPROM.writeByte(ADDR_Deviationtabel + i, v);
         }
         EEPROM.commit();
         Serial.println("Deviationtabel resored from StorageArea 1");
         pBlueTooth->SendString("Deviationtabel resored from StorageArea 1\n");
      }
      if(Val == String("restd2"))
      {
         int8_t v;
         for (uint16_t i = 0; i < 360;i++)
         {
             v = (int8_t)EEPROM.readByte(ADDR_Deviationtabel_copy2 + i);
             EEPROM.writeByte(ADDR_Deviationtabel + i, v);
         }
         EEPROM.commit();
         Serial.println("Deviationtabel resored from StorageArea 2");
         pBlueTooth->SendString("Deviationtabel resored from StorageArea 2\n");
      }
      if(Val == String("showdev"))
      {
        showDiv = !showDiv;
      }
      if(Val == String("senddev"))
      {
         Serial.println("Heading send with Deviation");
         pBlueTooth->SendString("Heading send with Deviation\n");
         SendHeadingwithDeviation = true;
         EEPROM.writeByte(ADDR_Deviationtabel + 361, 0x01);
         EEPROM.commit();
      }
      if(Val == String("stopdev"))
      {
         Serial.println("Heading not send with Deviation");
         pBlueTooth->SendString("Heading not send with Deviation\n");
         SendHeadingwithDeviation = false;
         EEPROM.writeByte(ADDR_Deviationtabel + 361, 0x00);
         EEPROM.commit();
      }
      if(Val.substring(0, 7) == String("setvari"))
      {
         EnteredVariation = Val.substring(8, Val.length()).toDouble();
         sprintf(dummy,"%2.1f",EnteredVariation);
         Serial.print("Set Variation:");Serial.println(EnteredVariation);
         pBlueTooth->SendString("Set Variation:");pBlueTooth->SendString(dummy);pBlueTooth->SendString("\n");
         delay(1000);
         return;
      }
      if(Val.substring(0, 6) == String("defdev"))
      {
         double vd = Val.substring(7, Val.length()).toDouble();
         if (vd < -12 || vd > 12 || HeadingFiltered > 359)
         {
             Serial.println("Wert nicht moeglich");
             pBlueTooth->SendString("Wert nicht moeglich\n");
             return;
         }
         int8_t v = (int8_t) (vd * 10); // Kommastelle
         int D = (int)round(HeadingFiltered);
         if(D >= 360) D -= 360;
         EEPROM.writeByte(ADDR_Deviationtabel + D , v);
         EEPROM.commit();
         sprintf(dummy,"%2.1f",vd);
         Serial.print("Set:");Serial.print(vd);Serial.print(" for Heading:");Serial.println(D);
         pBlueTooth->SendString("Set:");pBlueTooth->SendString(dummy);pBlueTooth->SendString(" for Heading:");
         sprintf(dummy,"%i",D); pBlueTooth->SendString(dummy);     
         pBlueTooth->SendString("\n");
         return;
      }
      if(Val == String("mcogdev"))
      {
          if(HeadingVariation == N2kDoubleNA && EnteredVariation == N2kDoubleNA)
          {
             Serial.println("Variation nicht vorhanden.");
             pBlueTooth->SendString("Variation nicht vorhanden.\n");
             return;
          }
          if(DEGCOG == N2kDoubleNA)
          {
             Serial.println("COG nicht vorhanden.");
             pBlueTooth->SendString("COG nicht vorhanden.\n");
             return;
          }
          Serial.println("make Deviationtabel. Turn Boat ....");
          pBlueTooth->SendString("make Deviationtabel. Turn Boat ....\n");
          // Turn ON make Table
          MakeDeviationtabel = true;
      }      
      if(Val == String("scogdev"))
      {
        Serial.println("Deviationtabel ready");
        pBlueTooth->SendString("Deviationtabel ready\n");
        EEPROM.commit();
        MakeDeviationtabel = false;
      }
      if(Val == String("coghead"))
      {
        if(HeadingVariation == N2kDoubleNA && EnteredVariation == N2kDoubleNA)
          {
             Serial.println("Variation nicht vorhanden.");
             pBlueTooth->SendString("Variation nicht vorhanden.\n");
             return;
          }
          if(DEGCOG == N2kDoubleNA)
          {
             Serial.println("COG nicht vorhanden.");
             pBlueTooth->SendString("COG nicht vorhanden.\n");
             return;
          }
          double Variation = HeadingVariation;
          if (EnteredVariation != N2kDoubleNA)
              Variation = EnteredVariation;
         // COG = Heading + Variation + Deviation
         double Deviation = DEGCOG - HeadingFiltered - Variation;
         if ( Deviation < -12)
            Deviation = -12;
         if ( Deviation > 12)
            Deviation = 12;
         // Write to EEPROM
         int8_t v  = 0x7F;
         if (Deviation != 12 && Deviation != -12)   // Not when Deviation ist bad
            v = (int8_t) (Deviation * 10); // Kommastelle
         int D = (int)round(HeadingFiltered);
         if(D >= 360) D -= 360;
         int8_t ve = (int8_t)EEPROM.readByte(ADDR_Deviationtabel + D);
         if( ve != 0x7F) // Not first Use
         {
             Serial.println("Deviation bereits vorhanden.");
             pBlueTooth->SendString("Deviation bereits vorhanden.\n");
          if (v != 0x7F)   // Not when Deviation ist bad
          {
             v = (int8_t) ((double) v * 0.9 + (double)ve * 0.1);
          }   
          else
          {
             Serial.println("New Deviation too big");
             pBlueTooth->SendString("New Deviation too big\n");
             v = ve;       // Do not loose the old value
          }
         }        
         Serial.print("Save Heading:"); Serial.print(HeadingFiltered);
         sprintf(dummy,"%3.1f",HeadingFiltered);
         pBlueTooth->SendString("Save Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
         sprintf(dummy,"%2.1f",(float) ((int8_t)v / (double)10));
         Serial.print(" Deviation :");
         pBlueTooth->SendString(" Deviation :");
         if (v == 0x7F)
         {
            Serial.println("NA");
            pBlueTooth->SendString("NA\n");
         }
         else
         {
            Serial.println(v);
            pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");
         }
         EEPROM.writeByte(ADDR_Deviationtabel + D , (int8_t)v);
         EEPROM.commit();
      }
      if(Val.substring(0, 4) == String("hdev"))
      {
          double v;
          int hd = Val.substring(5, Val.indexOf(" ",6)).toInt();
          if(Val.substring(Val.indexOf(" ",6),Val.length()) == " NA")
          {
             Serial.print("delete Deviation at Heading: ");Serial.println(hd);
             sprintf(dummy,"%i",hd);
             pBlueTooth->SendString("delete Deviation at Heading: ");
             pBlueTooth->SendString(dummy);pBlueTooth->SendString("\n");
             EEPROM.writeByte(ADDR_Deviationtabel + hd , 0x7F);
             EEPROM.commit();
             return; 
          }
          else
             v = Val.substring(Val.indexOf(" ",6),Val.length()).toDouble();
          if(hd >= 360 || hd < 0)
          {
             Serial.println("Heading wrong");
             pBlueTooth->SendString("Heading wrong\n");
             return;  
          }
          if (v < -12 || v > 12)
          {
             Serial.println("Deviation wrong");
             pBlueTooth->SendString("Deviation wrong\n");
             return; 
          }
          Serial.print("Save Heading:"); Serial.print(hd);
          sprintf(dummy,"%i",hd);
          pBlueTooth->SendString("Save Heading:"); pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
          sprintf(dummy,"%2.1f",(float) v);
          Serial.print(" Deviation :");
          Serial.println(v);
          pBlueTooth->SendString(" Deviation :");
          pBlueTooth->SendString(dummy);pBlueTooth->SendString(" \n");
          EEPROM.writeByte(ADDR_Deviationtabel + hd , (int8_t)(v * 10));
          EEPROM.commit();          
      }
      if(Val == String("cleardev"))
      {
          for (uint16_t i = 0; i < 360; i++)
          {
             EEPROM.writeByte(ADDR_Deviationtabel + i , 0x7F);    // 127 !!!         
          }
          EEPROM.commit();
          Serial.println("Deviationtabel deleted");
          pBlueTooth->SendString("Deviationtabel deleted\n");
          return;
      }
      if(Val == String("printdev"))
      {
         for (uint16_t j = 0; j < 360; j++)
         {
            double ve = (double)((int8_t)EEPROM.readByte(ADDR_Deviationtabel + j) / (double)10);
            if(j % 10 == 0)
            {
                Serial.println(""); pBlueTooth->SendString("\n");
            }
            if (ve <= 12.2 && ve >= -12.2) 
            {
              Serial.print(j);Serial.print("=");Serial.print(ve);Serial.print(" ");
              sprintf(dummy,"%i",j);
              pBlueTooth->SendString(dummy);pBlueTooth->SendString("=");
              sprintf(dummy,"%2.1f",ve);
              pBlueTooth->SendString(dummy);pBlueTooth->SendString(" ");
            }
            else  // Not avalibal 127 !!! Not set
            {
              Serial.print(j);Serial.print("=");Serial.print("NA ");
              sprintf(dummy,"%i",j);
              pBlueTooth->SendString(dummy);pBlueTooth->SendString("=");
              pBlueTooth->SendString("NA ");
            }
         }
      }
      if(Val == String("reset"))
      {
        Serial.println("Kurscompass restart.");
        pBlueTooth->SendString("Kurscompass restart.\n");
        digitalWrite(18, LOW); // Set GPIO18 
        delay(900);
        digitalWrite(18, HIGH); // Set GPIO18
        delay(400);
        ESP.restart(); //call reset 
      }
      if(Val == String("sendhead"))
      {
        Serial.println("Switch send Heading ON.");
        pBlueTooth->SendString("Switch send Heading ON.\n");
        EEPROM.writeByte(ADDR_SEND_HEADING, 1);
        EEPROM.commit();
        delay (2000);
      }
      if(Val == String("stophead"))
      {
        Serial.println("Switch send Heading OFF.");
        pBlueTooth->SendString("Switch send Heading OFF.\n");
        EEPROM.writeByte(ADDR_SEND_HEADING, 0);
        EEPROM.commit();
        delay (2000);
      }
      if(Val == String("cal") && calibrationStart == false)
      {
          // delete the calibration now.
          clearCalibrationEEPROM();
          calibrationStart = true; 
          calibrationStop = false;
          OnlyMag = false;
          BNO055.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
          Serial.println("Calibrating Module.....");
          pBlueTooth->SendString("Calibrating Module.....\n");
          delay(1000);
      }
      if(Val == String("calmag") && calibrationStart == false)
      {
          // delete the calibration now.
          calibrationStart = true; 
          calibrationStop = false;
          OnlyMag = true;
          BNO055.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
          Serial.println("Calibrating Magnetic Module.....");
          pBlueTooth->SendString("Calibrating Magnetic Module.....\n");
          delay(2000);
      }
      if(Val == String("clearcal") && calibrationStop == true)
      {
          // delete the calibration now.
          clearCalibrationEEPROM();
          calibrationStart = false; 
          calibrationStop = true;
          acc_x_ground = 0;
          acc_y_ground = 0;
          acc_z_ground = 0;
          BNO055.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF);
          Serial.println("Stop Calibrating Module.....");
          pBlueTooth->SendString("Stop Calibrating Module.....\n");
          delay(2000);
      }
      if(Val == String("setground") && calibrationStart == false)
      {
          Serial.print("Set Calibrating for Ground ");
          pBlueTooth->SendString("\nSet Calibrating for Ground ");
          calibrationStart = false; 
          calibrationStop = true;
          acc_x_ground = 0;
          acc_y_ground = 0;
          acc_z_ground = 0;  
          for (int counter = 0; counter < 1000 ; counter ++)    //Run this code 1000 times
          {
             unsigned long Loop_start = millis();
             imu::Vector<3> acc = BNO055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
             if (counter % 100 == 0) { Serial.print("."); pBlueTooth->SendString("."); }
             acc_x_ground += acc.x();
             acc_y_ground += acc.y();
             acc_z_ground += acc.z();
             while (millis() - Loop_start < 10);
          }
          acc_x_ground /= 1000;
          acc_y_ground /= 1000;
          acc_z_ground /= 1000;
          acc_z_ground -= 9.81;
          saveGroundCalibration();
          Serial.println("done");
          pBlueTooth->SendString("done\n");
          delay(2000); 
      }      
  }
  if(calibrationStart == true && calibrationStop == false && OnlyMag == false) // Komplett Calibration
  { 
    if(lastTimeCaltext < millis() || BNO055.isFullyCalibrated())
    {
       lastTimeCaltext = millis() + 2000;
       Calmag = 0;
       BNO055.getCalibration(&Calsystem, &Calgyro, &Calaccel, &Calmag);
       Serial.print("acceleration : "); Serial.println(Calaccel);
       Serial.print("Gyro         : "); Serial.println(Calgyro);
       Serial.print("Magnetic     : "); Serial.println(Calmag);
       Serial.print("System       : "); Serial.println(Calsystem);
       Serial.println("-----------------");       
       pBlueTooth->SendString("\n");
       pBlueTooth->SendString("acceleration : "); pBlueTooth->SendString(itoa(Calaccel,dummy,10)); pBlueTooth->SendString("\n");
       pBlueTooth->SendString("Gyro         : "); pBlueTooth->SendString(itoa(Calgyro,dummy,10)); pBlueTooth->SendString("\n");
       pBlueTooth->SendString("Magnetic     : "); pBlueTooth->SendString(itoa(Calmag,dummy,10)); pBlueTooth->SendString("\n");
       pBlueTooth->SendString("System       : "); pBlueTooth->SendString(itoa(Calsystem,dummy,10)); pBlueTooth->SendString("\n");
       pBlueTooth->SendString("-----------------\n");
    }
    if(BNO055.getSensorOffsets(BNO055Offset))
    {
      calibrationStart = false; 
      calibrationStop = true;   
      saveCalibrationToEEPROM();
      Serial.println("Calibrating Module.....done");
      pBlueTooth->SendString("Calibrating Module.....done\n");
      BNO055.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF);
      delay(1000);   
    }     
  }
  if(calibrationStart == true && calibrationStop == false && OnlyMag == true) // Pnly Magnetic Calibration
  { 
     Calmag = 0;
     BNO055.getCalibration(&Calsystem, &Calgyro, &Calaccel, &Calmag);  
  
    if(lastTimeCaltext < millis())
    {
      lastTimeCaltext = millis() + 2000;      
      char du[10];
      Serial.print("Magnetic     : "); Serial.println(Calmag);
      pBlueTooth->SendString("Magnetic     : "); pBlueTooth->SendString(itoa(Calmag,du,10)); pBlueTooth->SendString("\n");
    }
    if(Calmag == 3)
    {
      if(BNO055.getSensorMagneticOffsets(BNO055Offset))
      {
         saveCalibrationToEEPROM();
         calibrationStart = false; 
         calibrationStop = true;  
         Serial.println("Magnetic Calibrating ....done");
         pBlueTooth->SendString("Magnetic Calibration.....done\n");
         BNO055.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF);
         delay(1000);       
      }
    }
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  // N2kMsg.Print(&Serial);
  // Serial.printf("PGN: %u\n", (unsigned int)N2kMsg.PGN)
  unsigned char SID;
  switch (N2kMsg.PGN) {
    case 129026L:  // Get COG.
      tN2kHeadingReference HeadingReference;
      double COG;
      
      if ( ParseN2kCOGSOGRapid(N2kMsg, SID, HeadingReference, COG, SOG) )
      {
          if(COG == N2kDoubleNA) 
            return;
          LastCOG = millis();
          DEGCOG = COG * radToDeg;   
          SOG *= 3600.0/1852.0;  //const double msTokn=3600.0/1852.0;
      } 
      break;    
    case 130850L:
      if (ParseN2kPGN130850(N2kMsg))
      {
        Serial.printf("PNG 130850 : calibrationStart: %s  calibrationStop: %s", String(calibrationStart), String(calibrationStop));
        Serial.println("");
      }
      break;
    case 127258L:
      tN2kMagneticVariation source;
      uint16_t DaysSince1970;
      double variation;
      ParseN2kMagneticVariation(N2kMsg, SID, source, DaysSince1970, variation);
      HeadingVariation = variation * radToDeg;
      LastVariation = millis();
      break;
    case 130845L:
      uint16_t Key, Command, Value;
      if (ParseN2kPGN130845(N2kMsg, Key, Command, Value))
      {
#ifdef DEBUG        
         Serial.printf("Key: %d Command: %d Value: %d\n", Key, Command, Value);
#endif
      }
      break;
  } 
}

bool ParseN2kPGN130850(const tN2kMsg &N2kMsg)
{
  if (N2kMsg.PGN!=130850L) return false;
  int Index=2;
  unsigned char Command1=N2kMsg.GetByte(Index);
  unsigned char Command2=N2kMsg.GetByte(Index);
  unsigned char Command3=N2kMsg.GetByte(Index);
  unsigned char Command4=N2kMsg.GetByte(Index);
  unsigned char CalibrationStopStart=N2kMsg.GetByte(Index);
  //Serial.printf("Command1: %u  Command4: %u  CalibrationStopStart: %u  ", (unsigned int)Command1, (unsigned int)Command4, (unsigned int)CalibrationStopStart);
  if (Command1 == DEVICE_ID && Command4 == 18 && CalibrationStopStart == 0) {
    calibrationStart = true;
    Serial.println("Calibration Start");
    pBlueTooth->SendString("Calibration Start\n");
    // Send ack
    Send130851Ack(0); 
    return true;
  } else {
    if (Command1 == DEVICE_ID && Command4 == 18 && CalibrationStopStart == 1) {
      calibrationStop = true;
      Serial.println("Calibration Stop");
      pBlueTooth->SendString("Calibration Stop\n");
      // Send ack
      Send130851Ack(1);
      return true;
    }
  }
  return false;
}

bool ParseN2kPGN130845(const tN2kMsg &N2kMsg, uint16_t &Key, uint16_t &Command, uint16_t &Value)
{
#ifdef DEBUG
  Serial.println("Entering ParseN2kPGN130845");
#endif
  if (N2kMsg.PGN != 130845L) return false;
  int Index=2;
  unsigned char Target = N2kMsg.GetByte(Index);
  if (Target == NMEA2000.GetN2kSource()) {
    tN2kMsg N2kMsgReply;
    unsigned char source = N2kMsg.Source;
    Index = 6;
    Key = N2kMsg.Get2ByteUInt(Index);
    Command = N2kMsg.Get2ByteUInt(Index);    
    if (Command == 0x0000) {
      // Get
      if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, Key, 2))   // 2 = Ack
        NMEA2000.SendMsg(N2kMsgReply, DEV_COMPASS);
    }
    if (Command == 0x0100) {
      // Set
      switch (Key) {
        case 0x0000:  // Heading offset
          heading_offset_rad = N2kMsg.Get2ByteDouble(0.0001, Index);
          heading_offset_deg = (int)round(heading_offset_rad * RAD_TO_DEG);
          Serial.printf("heading_offset_rad: %f  heading_offset_deg: %d", heading_offset_rad, heading_offset_deg);
          Serial.println("");
          EEPROM.writeByte(EEP_HEADING_OFFSET, (int8_t)heading_offset_deg);
          EEPROM.commit();
          break;
        case 0x0039:  // Heel offset
          heel_offset_rad = N2kMsg.Get2ByteDouble(0.0001, Index);
          heel_offset_deg = (int)round(heel_offset_rad * RAD_TO_DEG);
          Serial.printf("heel_offset_rad: %f  heel_offset_deg: %d", heel_offset_rad, heel_offset_deg);
          Serial.println("");
          EEPROM.writeByte(EEP_HEEL_OFFSET, (int8_t)heel_offset_deg);
          EEPROM.commit();
          break;
        case 0x0031:  // Trim offset
          trim_offset_rad = N2kMsg.Get2ByteDouble(0.0001, Index);
          trim_offset_deg = (int)round(trim_offset_rad * RAD_TO_DEG);
          Serial.printf("trim_offset_rad: %f  trim_offset_deg: %d", trim_offset_rad, trim_offset_deg);
          Serial.println("");
          EEPROM.writeByte(EEP_TRIM_OFFSET, (int8_t)trim_offset_deg);
          EEPROM.commit();
          break;
        case 0xd200:  // Auto calibration (01=on, 02=auto locked)
          compass_autocalibration = N2kMsg.GetByte(Index);
          Serial.printf("Compass Calibration : %i", compass_autocalibration);
          Serial.println("");
          if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, 0xd400, 2))
            NMEA2000.SendMsg(N2kMsgReply, DEV_COMPASS);
          EEPROM.writeByte(EEP_AUTOCALIBRATION, (uint8_t)compass_autocalibration);
          EEPROM.commit();
          break;
        case 0xd300:  // Warning
          break;
        case 0xd400:  // Status
          break;      }
        }
    if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, Key, 2))
      NMEA2000.SendMsg(N2kMsgReply, DEV_COMPASS);      
    return true;
  } else {
    // Serial.printf("Skipping this one...\n");
    return false;
  }
  return false;
}

void Send130851Ack(int StopStart) {
  tN2kMsg N2kMsg;
  SetN2k130851Ack(N2kMsg, DEVICE_ID, 18, (unsigned char)StopStart);
  NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
}

bool SetN2kPGN130845(tN2kMsg &N2kMsg, unsigned char DEVICE_ID, uint16_t Key, uint16_t Command) {
  N2kMsg.SetPGN(130845L);
  N2kMsg.Priority=2;
  N2kMsg.AddByte(0x41); // Reserved
  N2kMsg.AddByte(0x9f); // Reserved
  N2kMsg.AddByte((unsigned char)DEVICE_ID);
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.Add2ByteUInt((uint16_t)Key);
  if (Command == 2) {
    N2kMsg.Add2ByteUInt(0x0200);
  }
  switch (Key) {
    case 0x0000:  // Heading offset
      N2kMsg.Add2ByteDouble(heading_offset_rad, 0.0001);
      N2kMsg.AddByte(0xff); // Reserved
     break;
    case 0x0039:  // Heel offset
      N2kMsg.Add2ByteDouble(heel_offset_rad, 0.0001);
      break;
    case 0x0031:  // Trim offset
      N2kMsg.Add2ByteDouble(trim_offset_rad, 0.0001);
      break;
    case 0xd200:  // Auto calibration (01=on, 02=auto locked)
      N2kMsg.AddByte(compass_autocalibration); // Reserved
      N2kMsg.AddByte(0xff); // Reserved
      break;
    case 0xd300:  // Warning
      N2kMsg.AddByte(0x00); // Reserved  // 00=No warning, 01=First calibration in progress, 02=Parameters in use are not valid
      N2kMsg.AddByte(0xff); // Reserved
      break;
    case 0xd400:  // Status
      N2kMsg.AddByte(0x00); // Reserved  // 00=Is calibrated, 01=Not calibrated, 02=Is calibrating
      N2kMsg.AddByte(0xff); // Reserved
      break;
    default:
      return false;
  }
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  return true;
}

void SetN2kPGN130851(tN2kMsg &N2kMsg, int DEVICE_ID, unsigned char Command, unsigned char CalibrationStopStart) {
    N2kMsg.SetPGN(130851L);
    N2kMsg.Priority=2;
    N2kMsg.AddByte(0x41); // Reserved
    N2kMsg.AddByte(0x9f); // Reserved
    N2kMsg.AddByte((unsigned char)DEVICE_ID);
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte((unsigned char)Command);
    N2kMsg.AddByte((unsigned char)CalibrationStopStart);
    N2kMsg.AddByte(0x00); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
}
                     
void SetN2k130851Ack(tN2kMsg &N2kMsg, int DEVICE_ID, unsigned char Command, unsigned char CalibrationStopStart) {
  SetN2kPGN130851(N2kMsg, DEVICE_ID, Command, CalibrationStopStart);
}

void SetN2k130845(tN2kMsg &N2kMsg, int DEVICE_ID, uint16_t Key, uint16_t Command) {
  SetN2kPGN130845(N2kMsg, DEVICE_ID, Key, Command);
}

bool loadCalibration()
{
    if ((EEPROM.readByte(EEP_CALIB_FLAG) & 0xF1) == 0x01)
    {
        Serial.println("Load calibrated values");
        heading_offset_deg = (int8_t)EEPROM.readByte(EEP_HEADING_OFFSET);
        if (heading_offset_deg == -1) heading_offset_deg = 0;
        heading_offset_rad = (float)heading_offset_deg * DEG_TO_RAD;
        Serial.printf("heading_offset_rad  : %f  heading_offset_deg : %d", heading_offset_rad, heading_offset_deg);
        Serial.println("");
        heel_offset_deg = (int8_t)EEPROM.readByte(EEP_HEEL_OFFSET);
        if (heel_offset_deg == -1) heel_offset_deg = 0;
        heel_offset_rad = (float)heel_offset_deg * DEG_TO_RAD;
        Serial.printf("heel_offset_rad     : %f  heel_offset_deg    : %d", heel_offset_rad, heel_offset_deg);
        Serial.println("");
        trim_offset_deg = (int8_t)EEPROM.readByte(EEP_TRIM_OFFSET);
        if (trim_offset_deg == -1) trim_offset_deg = 0;
        trim_offset_rad = (float)trim_offset_deg * DEG_TO_RAD;
        Serial.printf("trim_offset_rad     : %f  trim_offset_deg    : %d", trim_offset_rad, trim_offset_deg);
        Serial.println("");
        compass_autocalibration = EEPROM.readByte(EEP_AUTOCALIBRATION);        
        if (compass_autocalibration == 255) compass_autocalibration = 0;
        // Now the BNO Values
        EEPROM.get( ADDR_BNO055_CONFIG, BNO055Offset );
        PrintSensoroffsets();
        return true;
    }
    return false;
}

void clearCalibrationEEPROM()
{
    for (size_t i = 0; i < (18 + sizeof(adafruit_bno055_offsets_t)); ++i) {
      EEPROM.writeByte(i, 0xFF);
    }
    EEPROM.commit();
}

bool saveCalibrationToEEPROM()
{
    uint8_t i = (EEPROM.readByte(EEP_CALIB_FLAG) & 0x0F);
    if (i == 0x0F) i = 0;
    EEPROM.writeByte(EEP_CALIB_FLAG, (i | 0x01));
    EEPROM.put(ADDR_BNO055_CONFIG, BNO055Offset );
    EEPROM.commit();
    PrintSensoroffsets(); 
    return true;
}

bool loadGroundCalibration()
{  
   if ((EEPROM.readByte(EEP_CALIB_FLAG) & 0xF2) == 0x02)
   {
     acc_x_ground = EEPROM.readFloat(ADDR_ACC_X);
     acc_y_ground = EEPROM.readFloat(ADDR_ACC_Y);
     acc_z_ground = EEPROM.readFloat(ADDR_ACC_Z);
     return true;
   }
   return false;
}

bool saveGroundCalibration()
{
  uint8_t i = (EEPROM.readByte(EEP_CALIB_FLAG) & 0x0F);
  if (i == 0x0F) i = 0;
  EEPROM.writeByte(EEP_CALIB_FLAG, (i | 0x02));
  EEPROM.writeFloat(ADDR_ACC_X, acc_x_ground);
  EEPROM.writeFloat(ADDR_ACC_Y, acc_y_ground);
  EEPROM.writeFloat(ADDR_ACC_Z, acc_z_ground);
  EEPROM.commit();
  return true;
}

void PrintSensoroffsets()
{
#ifdef DEBUG
  Serial.print("accel_offset_x "); Serial.println(BNO055Offset.accel_offset_x); /**< x acceleration offset */
  Serial.print("accel_offset_y "); Serial.println(BNO055Offset.accel_offset_y); /**< y acceleration offset */
  Serial.print("accel_offset_z "); Serial.println(BNO055Offset.accel_offset_z); /**< z acceleration offset */
  Serial.println("");
  Serial.print("mag_offset_x   "); Serial.println(BNO055Offset.mag_offset_x); /**< x magnetometer offset */
  Serial.print("mag_offset_y   "); Serial.println(BNO055Offset.mag_offset_y); /**< y magnetometer offset */
  Serial.print("mag_offset_z   "); Serial.println(BNO055Offset.mag_offset_z); /**< z magnetometer offset */
  Serial.println("");
  Serial.print("gyro_offset_x  "); Serial.println(BNO055Offset.gyro_offset_x); /**< x gyroscrope offset */
  Serial.print("gyro_offset_y  "); Serial.println(BNO055Offset.gyro_offset_y); /**< y gyroscrope offset */
  Serial.print("gyro_offset_z  "); Serial.println(BNO055Offset.gyro_offset_z); /**< z gyroscrope offset */
  Serial.println("");
  Serial.print("accel_radius   "); Serial.println(BNO055Offset.accel_radius); /**< acceleration radius */
  Serial.println("");
  Serial.print("mag_radius     "); Serial.println(BNO055Offset.mag_radius); /**< magnetometer radius */
#endif
}
