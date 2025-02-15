# ESP32_Precision-9_compass_BNO055
Digital compass acting as B&G Precision 9 based on ESP32 and BNO055 (tilt compensated)
copy a lot of Code an Idea from https://github.com/htool/ESP32_Precision-9_compass_CMPS14 thanks for that.

- Included a second new heave algorithm from https://github.com/bareboat-necessities/bbn-wave-period-esp32/tree/main/bbn_wave_freq_m5atomS3. thanks for that idea. So all the (.h) files are needed for that.
- modyfied my heave algorithm a little bit.
- Now it could be switched beween the heave algorithms by bluetooth config commands. (sendmahony, sendnormal or sendmixed). find out which one is better for you.

## used Librarys 
- Adafruit_BNO055 copy the included modifyed directory to Librarys don't use the original.
  or add  "bool getSensorMagneticOffsets(adafruit_bno055_offsets_t &offsets_type);"  to
  the File "Adafruit_BNO055.h" in the public area.

- Timo's NMEA2000 Library https://github.com/ttlappalainen/NMEA2000
- NMEA2000_esp32  https://github.com/ttlappalainen/NMEA2000_esp32
- ArduinoEigen https://github.com/hideakitai/ArduinoEigen/tree/main (for new heave)
- ArxTypeTraits https://github.com/hideakitai/ArxTypeTraits (for new heave)
- Adafruit_BusIO
- Adafruit_Unified_Sensor
- BluetoothSerial

## BNO055
- After trying the MPU9250 first, I wasn't happy with the noise level. ( as https://github.com/htool ... I am with him!)

## Bluetooth
- Configuration can be done via Bluetooth using free App SerialBluetooth. Enter "help" to get the commandlist

### NMEA2000 PGN sending
  127250 - Vessel heading with 20Hz
  127257 - Attitude (Yaw, Pitch and Roll) with 10Hz
  127251 - Rate of turn with 20Hz
  127252 - Heave with 10Hz
    
