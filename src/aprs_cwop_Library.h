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
        INTERFACE_CLASS* interface;
        M_WIFI_CLASS* mWifi = NULL ;
        DISPLAY_LCD_CLASS* display = NULL;
        ONBOARD_SENSORS* onBoardSensors = NULL;

        unsigned long LAST_DATASET_UPLOAD = 0;
        unsigned long LAST_DATA_MEASUREMENTS = 0;
        unsigned long MAX_LATENCY_ALLOWED;
        
        long int lastMillisSensors;

        uint8_t NUMBER_OF_SENSORS_DATA_VALUES;
    
        float **measurements = NULL; //pointer to pointer
        String* measurementsOnBoard;
        int measureIndex[2];

        const float MCU_ADC_DIVIDER = 4096.0;
        uint8_t SELECTED_ADC_REF_RESISTANCE;

        float adc_ch_calcukated_e_resistance_avg;
        float adc_ch_measured_voltage_avg;


        // GBRL commands  *********************************************
        bool helpCommands(String $BLE_CMD, uint8_t sendTo );
        bool history(String $BLE_CMD, uint8_t sendTo);
        bool measurementInterval(String $BLE_CMD, uint8_t sendTo);
        bool cfg_commands(String $BLE_CMD, uint8_t sendTo);
        bool gbrl_summary_measurement_config( uint8_t sendTo);
        bool sw_commands(String $BLE_CMD, uint8_t sendTo);

    public:

        // external 3V3 power
        uint8_t ENABLE_3v3_PWR_PIN;
        // Voltage reference
        uint8_t VOLTAGE_REF_PIN;

        // Reports and measurements
        const int aprsRprtHour   = 10; // Number of APRS reports per hour
        const int aprsMsrmMax    = 3;  // Number of measurements per report (keep even)
        int       aprsMsrmCount  = 0;  // Measurements counter
        int       aprsTlmSeq     = 0;  // Telemetry sequence mumber

        bool       PROBE              = true;                   // True if the station is being probed

        const char aprsPath[]     PROGMEM = ">APRS,TCPIP*:";
        const char aprsTlmPARM[]  PROGMEM = ":PARM.Light,Soil,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,RB,B8";
        const char aprsTlmEQNS[]  PROGMEM = ":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100";
        const char aprsTlmUNIT[]  PROGMEM = ":UNIT.mV,mV,dBm,V,C,prb,on,on,sat,low,err,N/A,N/A";
        const char aprsTlmBITS[]  PROGMEM = ":BITS.10011111, ";
        const char eol[]          PROGMEM = "\r\n";

        char       aprsPkt[100]           = "";     // The APRS packet buffer, largest packet is 82 for v2.1


        bool hasNewMeasurementValues;
        float last_measured_probe_temp;
        float last_measured_time_delta;     
        int DATASET_NUM_SAMPLES;
        int DATASET_NUM_SAMPLES_TOTAL;
        
        bool Measurments_EN;
        bool Measurments_NEW;
        String measurement_Start_Time;

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

            String EXPERIMENTAL_DATA_FILENAME = "measurements.csv";

            
            // Measurements: RAM Storage of Live Data  ******************************
            // array size is the number of sensors to do data collection
            int NUM_SAMPLE_SAMPLING_READINGS;
            long int SAMPLING_INTERVAL;
            int MEASUREMENTS_BUFFER_SIZE;

            // Measurements: Planning / Schedule  **********************************
            unsigned long UPLOAD_DATASET_DELTA_TIME;
            unsigned long MEASUREMENT_INTERVAL;


            // configuration: PCB specific
            float    MCU_VDD = 3.38;


        } config_strut;

        config_strut config;
        
        bool scheduleWait;
        long int waitTimeSensors;

        SemaphoreHandle_t MemLockSemaphoreDatasetFileAccess = xSemaphoreCreateMutex();
        bool datasetFileIsBusySaveData = false;
        bool datasetFileIsBusyUploadData = false;

        // external sensors __________________________________________
        SHT3X_SENSOR*     sht3x;
        AHT20_SENSOR*     aht20;
        DS18B20_SENSOR*   ds18b20;
        String ch2_sensor_type;


        MEASUREMENTS();
        
        void init(INTERFACE_CLASS* interface, DISPLAY_LCD_CLASS* display,M_WIFI_CLASS* mWifi, ONBOARD_SENSORS* onBoardSensors );
        
        void settings_defaults();
        // **********************************
        void readSensorMeasurements();
        void readOnboardSensorData();
        void runExternalMeasurements();
        void readChannel2SensorMeasurements(int pos);
        void units();

        // ***********************************
        void initSaveDataset();
        bool saveDataMeasurements();

        bool initializeDataMeasurementsFile();
        bool initializeSensors();

        bool initializeDynamicVar( int size1D, int size2D);
        //Free Allocated memory
        void freeAllocatedMemory(int nRow, int nColumn);

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

