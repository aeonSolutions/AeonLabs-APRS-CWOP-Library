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
#include "Arduino.h"
#include "interface_class.h"
#include "m_wifi.h"
#include "m_display_lcd.h"

#include "esp32-hal-psram.h"
// #include "rom/cache.h"
extern "C" 
{
#include <esp_himem.h>
#include <esp_spiram.h>
}

#include <semphr.h>
#include "onboard_sensors.h"

#include "sensors/ds18b20.h"
#include "sensors/aht20.h"
#include "sensors/sht3x.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFi.h>

#ifndef MEASUREMENTS_COMMANDS  
  #define MEASUREMENTS_COMMANDS
  

  // **************************** == Measurements Class == ************************
class APRS_CWOP_CLASS {
    private:
        INTERFACE_CLASS* interface       = nullptr;
        M_WIFI_CLASS* mWifi              = nullptr ;
        ONBOARD_SENSORS* onBoardSensors  = nullptr;

        // GBRL commands  *********************************************
        bool helpCommands(String $BLE_CMD, uint8_t sendTo );


    public:
        // Reports and measurements
        const int aprsRprtHour   = 10; // Number of APRS reports per hour
        const int aprsMsrmMax    = 3;  // Number of measurements per report (keep even)
        int       aprsMsrmCount  = 0;  // Measurements counter
        int       aprsTlmSeq     = 0;  // Telemetry sequence mumber

        bool       PROBE              = true;                   // True if the station is being probed

        const char aprsPath[]      = ">APRS,TCPIP*:";
        const char aprsTlmPARM[]   = ":PARM.Light,Soil,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,RB,B8";
        const char aprsTlmEQNS[]   = ":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100";
        const char aprsTlmUNIT[]   = ":UNIT.mV,mV,dBm,V,C,prb,on,on,sat,low,err,N/A,N/A";
        const char aprsTlmBITS[]   = ":BITS.10011111, ";
        const char eol[]           = "\r\n";

        char       aprsPkt[100]           = "";     // The APRS packet buffer, largest packet is 82 for v2.1

        // The APRS connection client
        unsigned long   linkLastTime = 0UL;             // Last connection time

        // ...................................................     
        typedef struct{
            String aprsCallSign  = "FW0727";
            String aprsPassCode  = "-1";
            String aprsLocation  = "4455.29N/02527.08E_";

            // Telemetry bits
            char aprsTlmBits;

            // APRS parameters
            String  aprsServer;          // CWOP APRS-IS server address to connect to
            int   aprsPort;              // CWOP APRS-IS port

            int   altMeters;             // Altitude in Bucharest

            // configuration: PCB specific
            float    MCU_VDD = 3.38;
        } config_strut;

        config_strut config;
        

        // Sensors
        const unsigned long snsReadTime = 30UL * 1000UL;                          // Total time to read sensors, repeatedly, for aprsMsrmMax times
        const unsigned long snsDelayBfr = 3600000UL / aprsRprtHour - snsReadTime; // Delay before sensor readings
        const unsigned long snsDelayBtw = snsReadTime / aprsMsrmMax;              // Delay between sensor readings
        unsigned long       snsNextTime = 0UL;                                    // Next time to read the sensors

        // Time synchronization and keeping
        const char    timeServer[] PROGMEM  = "utcnist.colorado.edu";  // Time server address to connect to (RFC868)
        const int     timePort              = 37;                      // Time server port
        unsigned long timeNextSync          = 0UL;                     // Next time to syncronize
        unsigned long timeDelta             = 0UL;                     // Difference between real time and internal clock
        bool          timeOk                = false;                   // Flag to know the time is accurate
        const int     timeZone              = 0;                       // Time zone

        // Statistics (round median filter for the last 3 values)
        enum      rMedIdx {MD_TEMP, MD_PRES, MD_RSSI, MD_SRAD, MD_VCC, MD_A0, MD_A1, MD_ALL};
        int       rMed[MD_ALL][4];

        // ____________________________________________________
        APRS_CWOP_CLASS();
        
        void init(INTERFACE_CLASS* interface, DISPLAY_LCD_CLASS* display,M_WIFI_CLASS* mWifi, ONBOARD_SENSORS* onBoardSensors );
        
        void settings_defaults();

        // GBRL commands  *********************************************
        bool gbrl_commands(String $BLE_CMD, uint8_t sendTo);

        // Setup configuration and settings *******************************************
        bool readSettings( fs::FS &fs = LittleFS );
        bool saveSettings( fs::FS &fs = LittleFS  );

        // ****************************************************
        long  altFeet(int altMeters);
        float altCorr(int altMeters);

        int rMedOut(int idx);
        void rMedIn(int idx, int x);

        void aprsSend(const char *pkt);
        void aprsAuthenticate();
        void aprsSendWeather(int temp, int hmdt, int pres, int lux);
        void aprsSendTelemetry(int a0, int a1, int rssi, int vcc, int temp, byte bits);
        void aprsSendTelemetrySetup();
        void aprsSendStatus(const char *message);
        void aprsSendPosition(const char *comment = NULL);
};


#endif

