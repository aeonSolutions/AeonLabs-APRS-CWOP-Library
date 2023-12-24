/*
 Copyright (c) 2023 Miguel Tomas, http://www.aeonlabs.science

License Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
You are free to:
   Share — copy and redistribute the material in any medium or format
   Adapt — remix, transform, and build upon the material

The licensor cannot revoke these freedoms as long as you follow the license terms. Under the following terms:
Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. 
You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

NonCommercial — You may not use the material for commercial purposes.

ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under
 the same license as the original.

No additional restrictions — You may not apply legal terms or technological measures that legally restrict others
 from doing anything the license permits.

Notices:
You do not have to comply with the license for elements of the material in the public domain or where your use
 is permitted by an applicable exception or limitation.
No warranties are given. The license may not give you all of the permissions necessary for your intended use. 
For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.


Before proceeding to download any of AeonLabs software solutions for open-source development
 and/or PCB hardware electronics development make sure you are choosing the right license for your project. See 
https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/wiki/AeonLabs-Solutions-for-Open-Hardware-&-Source-Development
 for Open Hardware & Source Development for more information.

NOTE:
The current code development is heavily based on the code by cstroie found on this github repository: https://github.com/cstroie/WxUno
*/

#include "measurements.h"
#include "Arduino.h"
#include"m_math.h"
#include "m_atsha204.h"
#include "FS.h"
#include <LittleFS.h>
#include "m_atsha204.h"
#include "lcd_icons.h"
#include "m_file_functions.h"
#include "driver/touch_pad.h"



MEASUREMENTS::APRS_CWOP_CLASS() {
    // Telemetry bits
    this->aprsTlmBits     = B00000000;
}

//****************************************************************
void APRS_CWOP_CLASS::init(INTERFACE_CLASS* interface, DISPLAY_LCD_CLASS* display,M_WIFI_CLASS* mWifi, ONBOARD_SENSORS* onBoardSensors ){    
    this->interface = interface;
    this->interface->mserial->printStr("\ninit measurements library ...");
    this->mWifi = mWifi;
    this->onBoardSensors =  onBoardSensors;

    this->settings_defaults();
    this->interface = interface;

  // Initialize the random number generator and set the APRS telemetry start sequence
  randomSeed(hwTemp + timeUNIX(false) + hwVcc + millis());
  aprsTlmSeq = random(1000);
  
  this->interface->mserial->printStrln( "Telemetry:" + String(aprsTlmSeq) );
  // Start the sensor timer
  snsNextTime = millis();

  this->interface->mserial->printStrln("done.");
}

// ****************************************************************************
void APRS_CWOP_CLASS::settings_defaults(){

    this->config.aprsServer                   = "cwop5.aprs.net";   // CWOP APRS-IS server address to connect to
    this->config.aprsPort                     = 14580;              // CWOP APRS-IS port

    this->config.aprsCallSign                 = "null";
    this->config.aprsPassCode                 = "-1";
    this->config.aprsLocation                 = "null";
}


// --------------------------------------------------------------------------

bool APRS_CWOP_CLASS::saveSettings(fs::FS &fs){
    this->interface->mserial->printStrln( this->interface->DeviceTranslation("save_daq_settings")  + "...");

    if (fs.exists("/aprs.cfg") )
        fs.remove("/aprs.cfg");

    File settingsFile = fs.open("/aprs.cfg", FILE_WRITE); 
    if ( !settingsFile ){
        this->interface->mserial->printStrln( this->interface->DeviceTranslation("err_create_daq_settings") + ".");
        settingsFile.close();
        return false;
    }

    settingsFile.print( String(this->config.MEASUREMENT_INTERVAL) + String(';'));

    settingsFile.close();
    return true;
}
// --------------------------------------------------------------------

bool APRS_CWOP_CLASS::readSettings(fs::FS &fs){    
    File settingsFile = fs.open("/aprs.cfg", FILE_READ);
    if (!settingsFile){
        this->interface->mserial->printStrln( this->interface->DeviceTranslation("err_notfound_daq_settings")  + ".");
        settingsFile.close();
        return false;
    }
    if (settingsFile.size() == 0){
        this->interface->mserial->printStrln( this->interface->DeviceTranslation("err_invalid_daq_settings") + ".");
        settingsFile.close();
        return false;    
    }

    String temp= settingsFile.readStringUntil(';');

    this->config.MEASUREMENT_INTERVAL = atol(settingsFile.readStringUntil( ';' ).c_str() ); 

    settingsFile.close();
    return true;
}

// **************************************
long APRS_CWOP_CLASS::altFeet(int altMeters){
  return (long)(altMeters * 3.28084);  // Altitude in feet
}

// **************************************
 float APRS_CWOP_CLASS::altCorr(int altMeters){
  return pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));  // Altitude correction for QNH
 }

 
/** ***************************************************
  Simple median filter: get the median
  2014-03-25: started by David Cary

  @param idx the index in round median array
  @return the median
*/
int APRS_CWOP_CLASS::rMedOut(int idx) {
  // Return the last value if the buffer is not full yet
  if (rMed[idx][0] < 3) return rMed[idx][3];
  else {
    // Get the maximum and the minimum
    int the_max = max(max(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    int the_min = min(min(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    // Clever code: XOR the max and min, remaining the middle
    return the_max ^ the_min ^ rMed[idx][1] ^ rMed[idx][2] ^ rMed[idx][3];
  }
}

/** *****************************************************
  Simple median filter: add value to array

  @param idx the index in round median array
  @param x the value to add
*/
void APRS_CWOP_CLASS::rMedIn(int idx, int x) {
  // At index 0 there is the number of values stored
  if (rMed[idx][0] < 3) rMed[idx][0]++;
  // Shift one position
  rMed[idx][1] = rMed[idx][2];
  rMed[idx][2] = rMed[idx][3];
  rMed[idx][3] = x;
}


/** ****************************************************
  Send an APRS packet and, eventuall, print it to serial line

  @param *pkt the packet to send
*/
void APRS_CWOP_CLASS::aprsSend(const char *pkt) {
#ifdef DEBUG
  Serial.print(pkt);
#endif
  ethClient.print(pkt);
}

/**
  Return time in zulu APRS format: HHMMSSh

  @param *buf the buffer to return the time to
  @param len the buffer length
*/
char aprsTime(char *buf, size_t len) {
  // Get the time, but do not open a connection to server
  unsigned long utm = timeUNIX(false);
  // Compute hour, minute and second
  int hh = (utm % 86400L) / 3600;
  int mm = (utm % 3600) / 60;
  int ss =  utm % 60;
  // Return the formatted time
  snprintf_P(buf, len, PSTR("%02d%02d%02dh"), hh, mm, ss);
}

/** **************************************
  Send APRS authentication data
  user FW0727 pass -1 vers WxUno 3.1"
*/
void APRS_CWOP_CLASS::aprsAuthenticate() {
  strcpy_P(aprsPkt, PSTR("user "));
  strcat_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, PSTR(" pass "));
  strcat_P(aprsPkt, this->config.aprsPassCode);
  strcat_P(aprsPkt, PSTR(" vers "));
  strcat_P(aprsPkt, this->interface->DEVICE_NAME);
  strcat_P(aprsPkt, PSTR(" "));
  strcat_P(aprsPkt, this->interface->firmware_version);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**  ******************************************************
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxUno

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
*/
void APRS_CWOP_CLASS::aprsSendWeather(int temp, int hmdt, int pres, int lux) {
  char buf[8];
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("@"));
  aprsTime(buf, sizeof(buf));
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, this->config.aprsLocation);
  // Wind (unavailable)
  strcat_P(aprsPkt, PSTR(".../...g..."));
  // Temperature
  if (temp >= -460) { // 0K in F
    sprintf_P(buf, PSTR("t%03d"), temp);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  else {
    strcat_P(aprsPkt, PSTR("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      strcat_P(aprsPkt, PSTR("h00"));
    }
    else {
      sprintf_P(buf, PSTR("h%02d"), hmdt);
      strncat(aprsPkt, buf, sizeof(buf));
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    sprintf_P(buf, PSTR("b%05d"), pres);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  // Illuminance, if valid
  if (lux >= 0 and lux <= 999) {
    sprintf_P(buf, PSTR("L%03d"), lux);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  // Comment (device name)
  strcat_P(aprsPkt, this->interface->config.DEVICE_NAME);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**  *****************************************************
  Send APRS telemetry and, periodically, send the telemetry setup
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

  @param a0 read analog A0
  @param a1 read analog A1
  @param rssi GSM RSSI level
  @param vcc voltage
  @param temp internal temperature
  @param bits digital inputs
*/
void APRS_CWOP_CLASS::aprsSendTelemetry(int a0, int a1, int rssi, int vcc, int temp, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup if the sequence number is 0
  if (aprsTlmSeq == 0) aprsSendTelemetrySetup();
  // Compose the APRS packet
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("T"));
  char buf[40];
  snprintf_P(buf, sizeof(buf), PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, a0, a1, rssi, vcc, temp);
  strncat(aprsPkt, buf, sizeof(buf));
  itoa(bits, buf, 2);
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**   ******************************
  Send APRS telemetry setup
*/
void APRS_CWOP_CLASS::aprsSendTelemetrySetup() {
  char padCallSign[10];
  strcpy_P(padCallSign, this->config.aprsCallSign);  // Workaround
  sprintf_P(padCallSign, PSTR("%-9s"), padCallSign);
  // Parameter names
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmPARM);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Equations
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmEQNS);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Units
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmUNIT);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Bit sense and project name
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmBITS);
  strcat_P(aprsPkt, this->interface->config.DEVICE_NAME);
  strcat_P(aprsPkt, PSTR("/"));
  strcat_P(aprsPkt, this->interface->firmware_version);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**  ****************************************
  Send APRS status
  FW0690>APRS,TCPIP*:>Fine weather

  @param message the status message to send
*/
void APRS_CWOP_CLASS::aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message[0] != '\0') {
    // Send the APRS packet
    strcpy_P(aprsPkt, this->config.aprsCallSign);
    strcat_P(aprsPkt, aprsPath);
    strcat_P(aprsPkt, PSTR(">"));
    strcat(aprsPkt, message);
    strcat_P(aprsPkt, eol);
    aprsSend(aprsPkt);
  }
}

/**   ***********************************************
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void APRS_CWOP_CLASS::aprsSendPosition(const char *comment = NULL) {
  // Compose the APRS packet
  strcpy_P(aprsPkt, this->config.aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("!"));
  strcat_P(aprsPkt, this->config.aprsLocation);
  strcat_P(aprsPkt, PSTR("/000/000/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), this->altFeet(this->altMeters));
  strncat(aprsPkt, buf, sizeof(buf));
  if (comment != NULL) strcat(aprsPkt, comment);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

// *********************************************************
// GBRL commands -------------------------------------------
 bool APRS_CWOP_CLASS::helpCommands(String $BLE_CMD, uint8_t sendTo){
    if($BLE_CMD != "$?" && $BLE_CMD !="$help" )
        return false;

    String dataStr="Measurements Commands:\n" \
                    "$ufid                  - "+ this->interface->DeviceTranslation("ufid") +" unique fingerprint ID\n" \ 
                    "$set mi [sec]          - "+ this->interface->DeviceTranslation("set_mi") +"\n\n";

    this->interface->sendBLEstring( dataStr, sendTo);
      
    return false;
 }

// ---------------------------------------------------------
bool APRS_CWOP_CLASS::gbrl_commands(String $BLE_CMD, uint8_t sendTo){
    String dataStr="";

    bool result =false;
    result = this->helpCommands( $BLE_CMD,  sendTo);
    
    bool result3 =false;
    result3 = this->cfg_commands($BLE_CMD,  sendTo);
    
    //this->gbrl_menu_selection();
   
   return ( result || result2 || result3 || result4 );

}