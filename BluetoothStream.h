//*****************************************************************************
//
// File Name     : 'BluetoothStream.h'
// Improoved by  : Bernd Cirotzki

//*****************************************************************************

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "BluetoothSerial.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define PAIR_MAX_DEVICES 5

class BluetoothStream
{
public:
    BluetoothStream();
    ~BluetoothStream(){};
    void ShowBluetoothConfig();
    bool ConfigBluetooth(String ConfString);
    void SendString(const char* Sendstring);
    void BTConfirmRequestCallback(uint32_t numVal);
    void BTAuthCompleteCallback(bool success);
    void ParseMessages(String &Val);
    bool available() { return SerialBT.available(); }
private:
    BluetoothSerial SerialBT;
    uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
    char bda_str[18];
    bool WantPairing;
    uint32_t WantPairingTime;
    void ShowDelete(bool Loeschen);
    char* bda2str(const uint8_t* bda, char *str, size_t size);
};

#endif
