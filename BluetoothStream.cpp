//*****************************************************************************
//
// File Name     : 'BluetoothStream.cpp'
// Improoved by  : Bernd Cirotzki

//*****************************************************************************

#include "BluetoothStream.h"

BluetoothStream *pBT = 0;

void BTConfirmRequestCallback(uint32_t numVal)
{
   if (pBT) pBT->BTConfirmRequestCallback(numVal);
}

void BTAuthCompleteCallback(bool success)
{
   if (pBT) pBT->BTAuthCompleteCallback(success);
}

void BluetoothStream::BTConfirmRequestCallback(uint32_t numVal)
{
  Serial.println("");
  Serial.print("Want Pairing with : ");
  WantPairing = true;
  WantPairingTime = millis();
  Serial.println(numVal);
  Serial.println("Send : OK to accept. or NO to deny");
  Serial.print("wait ..");
}

void BluetoothStream::BTAuthCompleteCallback(bool success)
{
  if (success)
    Serial.print(" .. Pairing accepted from User");
  else
    Serial.print(" .. Pairing failed, rejected by user");
  if (WantPairing == false)
    Serial.println("");
}

BluetoothStream::BluetoothStream()
{
   Serial.print("  Start Bluetooth   ");
   pBT = this;
   WantPairing = false;
   SerialBT.enableSSP();
   SerialBT.onConfirmRequest(::BTConfirmRequestCallback);
   SerialBT.onAuthComplete(::BTAuthCompleteCallback); 
   SerialBT.begin("Kompass Sensor"); //Bluetooth device name
   ShowDelete(false);
}

void BluetoothStream::ShowBluetoothConfig()
{
    Serial.print("  Bluetooth Name       : Kompass Sensor");
    ShowDelete(false);
}

bool BluetoothStream::ConfigBluetooth(String ConfString)
{
  
  if(WantPairing == true && ConfString == String("OK"))
  {
      WantPairing = false;
      SerialBT.confirmReply(true);
      Serial.print(" .. Pairing accepted");
  }
  if(WantPairing == true && ConfString == String("NO"))
  {
      WantPairing = false;
      SerialBT.confirmReply(false);
      Serial.print(" .. Pairing deny.");
  }
  if (ConfString == String("SHDEV"))
  {
    Serial.println("");
    ShowDelete(false);
  }
  if (ConfString == String("DELDEV"))
  {
    Serial.println("");
    ShowDelete(true);
  }
  return false;
}

void BluetoothStream::ParseMessages(String &Val)
{
   if(WantPairing == true)
   {
      if (WantPairingTime + 20000 < millis())
      {
         WantPairing = false;
         SerialBT.confirmReply(false);
         Serial.println(" .... No Pairing timeout");
      }
   }
   if (SerialBT.hasClient() && SerialBT.available())
   {
       // Get Messages from BT
       while (SerialBT.available())
       {
          Val.concat((char)SerialBT.read());
       }             
   }   
}

void BluetoothStream::SendString(const char* Sendstring)
{
  if (SerialBT.hasClient())
  {
     SerialBT.write((const uint8_t*)Sendstring, strlen(Sendstring));
  }
}

char *BluetoothStream::bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}
 
void BluetoothStream::ShowDelete(bool Loeschen) {
 
  Serial.print("  Bluetooth address    : "); Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) {
    Serial.println("  No bonded device found.");
  } else {
    Serial.print("  Bonded device count  : "); Serial.println(count);
    if(PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES; 
      Serial.print("  Reset bonded devices : "); Serial.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) {
      for(int i = 0; i < count; i++) {
        Serial.print("  Found bonded device  : ");
        Serial.print(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(Loeschen)
        {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) {
            Serial.print(" Removed "); 
          } else {
            Serial.print(" Failed to remove ");
          }
        }
        Serial.println("");
      }        
    }
  }
}
