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
#include "N2kMessages.h"
#include <NMEA2000_CAN.h>
#include "BluetoothStream.h"

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
bool send_heading_true = false;                     // to do 20 vs 10hz
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

float PitchAccHead;
float RollAccHead;
float PitchHead;
float RollHead;

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
float HeadingVariation;

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

const uint8_t EEPROM_SIZE = (18 + sizeof(adafruit_bno055_offsets_t));

BluetoothStream *pBlueTooth;
                                                
void setup() 
{
  Wire.begin(16,17);
  Serial.begin(115200);  
  Serial.println("Start Kurssensor ...");
  pBlueTooth = new BluetoothStream();
  pBlueTooth->SendString("Start Kurssensor ...");
  delay(500);
  Serial.println("EEPROM start");
  if (!EEPROM.begin(EEPROM_SIZE))
  {
      Serial.println("EEPROM start failed");
  }
  // Settings for BNO055
  BNO055.begin();
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
     BNO055.setSensorOffsets(BNO055Offset);
     calibrationStart = false;   // Set to Calibration is OK. The GetCalibration is swinging
     calibrationStop = true;
     delay(100);
  }
  else
  {
     Serial.println("No Sensor - Calibration from EEPROM");  
     calibrationStart = false; 
     calibrationStop = false;      
  }
  if (loadGroundCalibration())
     Serial.println("Ground - Calibration sucessfully read from EEPROM");
  else
     Serial.println("No Ground - Calibration from EEPROM");
  delay(3000);  
  send_heading=true;
  send_heading_true=false;
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
      BNO055.enterSuspendMode();
      delay(1000);
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
    Xm=mag.x()*cos(PitchRad)-mag.y()*sin(RollRad)*sin(PitchRad)+mag.z()*cos(RollRad)*sin(PitchRad);
    Ym=mag.y()*cos(RollRad)+mag.z()*sin(RollRad);
    
    XmNew=0.85*XmOld + 0.15*Xm;
    YmNew=0.85*YmOld + 0.15*Ym;
#ifdef DEBUG
    Heading=atan2(Ym,Xm) * radToDeg;
    if (Heading < 0) Heading += 360.0;
#endif
    HeadingFiltered=atan2(YmNew,XmNew) * radToDeg;
    HeadingFiltered -= heading_offset_deg;  // Set from Plotter
    if (HeadingFiltered < 0) HeadingFiltered += 360.0;

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
    Serial.print(Calaccel);
    Serial.print(",");
    Serial.print(Calgyro);
    Serial.print(",");
    Serial.print(Calmag);
    Serial.print(",");
    Serial.print(Calsystem);
    Serial.print(",");
    Serial.print(PitchAcc);
    Serial.print(",");
    Serial.print(RollAcc);
    Serial.print(",");
    Serial.print(RateofTurn);
    Serial.print(",");
    Serial.print(RateofTurn);
    Serial.print(",");
    Serial.print(-Pitch);
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
     if (send_heading)
     {
        if (0 != EEPROM.readByte(ADDR_SEND_HEADING))
        {
          N2kMsg.Clear();
          SetN2kPGN127250(N2kMsg, SID, (HeadingFiltered * degToRad), N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
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
  NMEA2000.ParseMessages();  
}

void CheckForCalibration()
{
  String Val;
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
      if(Val == String("help") )
      {
         Serial.println("reset     :  Reset Kurssensor.");
         Serial.println("sendhead  :  Switch Send Heading ON.");
         Serial.println("stophead  :  Switch Send heading OFF.");
         Serial.println("cal       :  Start Sensor Calibration.");
         Serial.println("calmag    :  Start Sensor only Magnetc Calibration.");
         Serial.println("clearcal  :  Delete the Calibration.");
         Serial.println("setground :  Set the Ground Calibration.");
         pBlueTooth->SendString("reset     :  Reset Kurssensor.\n");
         pBlueTooth->SendString("sendhead  :  Switch Send Heading ON.\n");
         pBlueTooth->SendString("stophead  :  Switch Send heading OFF.\n");
         pBlueTooth->SendString("cal       :  Start Sensor Calibration.\n");
         pBlueTooth->SendString("calmag    :  Start Sensor only Magnetc Calibration.\n");
         pBlueTooth->SendString("clearcal  :  Delete the Calibration.\n");
         pBlueTooth->SendString("setground :  Set the Ground Calibration.\n");
      }
      if(Val == String("reset"))
      {
        Serial.println("Kurscompass restart.");
        pBlueTooth->SendString("Kurscompass restart.\n");
        delay (2000);
        ESP.restart(); //call reset 
      }
      if(Val == String("sendhead"))
      {
        Serial.println("Switch send Heading ON.");
        pBlueTooth->SendString("Switch send Heading ON.\n");
        EEPROM.writeByte(ADDR_SEND_HEADING, 1);
        delay (2000);
      }
      if(Val == String("stophead"))
      {
        Serial.println("Switch send Heading OFF.");
        pBlueTooth->SendString("Switch send Heading OFF.\n");
        EEPROM.writeByte(ADDR_SEND_HEADING, 0);
        delay (2000);
      }
      if(Val == String("cal") && calibrationStart == false)
      {
          // delete the calibration now.
          clearCalibrationEEPROM();
          calibrationStart = true; 
          calibrationStop = false;
          OnlyMag = false;
          Serial.println("Calibrating Module.....");
          pBlueTooth->SendString("Calibrating Module.....\n");
          delay(2000);
      }
      if(Val == String("calmag") && calibrationStart == false)
      {
          // delete the calibration now.
          calibrationStart = true; 
          calibrationStop = false;
          OnlyMag = true;
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
       char dummy[10];
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
      delay(2000);   
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
         delay(1000);       
      }
    }
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  // N2kMsg.Print(&Serial);
  // Serial.printf("PGN: %u\n", (unsigned int)N2kMsg.PGN);

  switch (N2kMsg.PGN) {
    case 130850L:
      if (ParseN2kPGN130850(N2kMsg))
      {
        Serial.printf("PNG 130850 : calibrationStart: %s  calibrationStop: %s", String(calibrationStart), String(calibrationStop));
        Serial.println("");
      }
      break;
    case 127258L:
      unsigned char SID;
      tN2kMagneticVariation source;
      uint16_t DaysSince1970;
      double variation;
      ParseN2kMagneticVariation(N2kMsg, SID, source, DaysSince1970, variation);
      HeadingVariation = (float)variation * radToDeg;
      send_heading_true = true;
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
    for (size_t i = 0; i < EEPROM_SIZE; ++i) {
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
