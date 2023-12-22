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
    // external 3V3 power
    this->ENABLE_3v3_PWR_PIN = 38;
    // Voltage reference
    this->VOLTAGE_REF_PIN = 7;
    // External PWM / Digital IO Pin
    this->EXT_IO_ANALOG_PIN = 6;
    this->SELECTED_ADC_REF_RESISTANCE = 0;

    this->Measurments_NEW=false;
    this->Measurments_EN=false;
    this->DATASET_NUM_SAMPLES=0;

    // Telemetry bits
    this->aprsTlmBits     = B00000000;
}


//****************************************************************
void APRS_CWOP_CLASS::init(INTERFACE_CLASS* interface, DISPLAY_LCD_CLASS* display,M_WIFI_CLASS* mWifi, ONBOARD_SENSORS* onBoardSensors ){    
    this->interface = interface;
    this->interface->mserial->printStr("\ninit measurements library ...");
    this->mWifi = mWifi;
    this->display = display;
    this->onBoardSensors =  onBoardSensors;

    // ADC 
    pinMode(this->EXT_IO_ANALOG_PIN, INPUT);
    // ADC Power
    pinMode(this->ENABLE_3v3_PWR_PIN, OUTPUT);

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

// **************************************************
bool APRS_CWOP_CLASS::initializeSensors(){
  this->NUMBER_OF_SENSORS_DATA_VALUES = 0;
  String dataStr = "";
  this->ch2_sensor_type ="disabled";

  if ( this->config.channel_2_switch_en == true ){
    this->sht3x = new SHT3X_SENSOR();
    this->sht3x->init(this->interface, 0x44);
    if ( this->sht3x->startSHT3X() ){
      dataStr += "The device is now setup for Temperature and Humidty readings using the SHT3x sensor\n";
      this->NUMBER_OF_SENSORS_DATA_VALUES =  this->NUMBER_OF_SENSORS_DATA_VALUES  + this->sht3x->numSensors;
      this->ch2_sensor_type ="sht3x";
    }else{
      this->sht3x->init(this->interface, 0x45);
      if ( this->sht3x->startSHT3X() ){
        this->NUMBER_OF_SENSORS_DATA_VALUES =  this->NUMBER_OF_SENSORS_DATA_VALUES  + this->sht3x->numSensors;
        dataStr += "The device is now setup for Temperature and Humidty readings using the SHT3x sensor.\n";
        this->ch2_sensor_type ="sht3x";
      }
    }
  }

  if (this->measurements != nullptr){
    this->interface->mserial->printStrln("Reinitializing Dynamic memory:");
    this->freeAllocatedMemory(this->measureIndex[0], this->measureIndex[1]);
    delete this->measurementsOnBoard;
    this->measurementsOnBoard = nullptr; 
  }else{
      this->interface->mserial->printStrln("Initializing Dynamic memory:");
  }

  this->interface->mserial->printStrln("Number of SAMPLING READINGS              :"+ String(this->config.NUM_SAMPLE_SAMPLING_READINGS));
  this->interface->mserial->printStrln("Number of Sensor Data values per reading :" + String(this->NUMBER_OF_SENSORS_DATA_VALUES));
  this->interface->mserial->printStrln("PSRAM buffer size                        :" + String(this->config.MEASUREMENTS_BUFFER_SIZE));
  
  this->measureIndex[0] =  this->NUMBER_OF_SENSORS_DATA_VALUES;
  this->measureIndex[1] = this->config.NUM_SAMPLE_SAMPLING_READINGS * this->config.MEASUREMENTS_BUFFER_SIZE;
  
  this->interface->mserial->printStrln( "Max idx is : " + String( this->measureIndex[1]) );

  // 1D number of sample readings ; 2D number of sensor data measuremtns; 3D RAM buffer size
  if ( false == this->initializeDynamicVar( this->measureIndex[0] ,  this->measureIndex[1] ) ){
    this->interface->mserial->printStrln("Error Initializing Measruments Buffer.");
    this->display->tftPrintText(0,160,"ERR Exp. data Buffer",2,"center", TFT_WHITE, true); 
    this->interface->onBoardLED->led[0] = this->interface->onBoardLED->LED_RED;
    this->interface->onBoardLED->statusLED(100, 5);
    // TODO : what to do when memeory alloc is NULL
  }else{
      this->measurementsOnBoard = new String[this->config.MEASUREMENTS_BUFFER_SIZE];

      float bufSize= sizeof(char)*(this->config.NUM_SAMPLE_SAMPLING_READINGS * this->NUMBER_OF_SENSORS_DATA_VALUES * this->config.MEASUREMENTS_BUFFER_SIZE); // bytes
      units=" B";
      if (bufSize>1024){
          bufSize=bufSize/1024;
          units=" Kb";
      }
      this->interface->mserial->printStrln("Buffer size:" + String(bufSize) + units);
      this->display->tftPrintText(0,160,(char*) String("Buf size:"+String(bufSize)+ units).c_str(),2,"center", TFT_WHITE, true); 
      delay(1000);
      this->display->tftPrintText(0,160,"Exp. Data RAM ready",2,"center", TFT_WHITE, true); 
      this->interface->mserial->printStrln("Exp. Data RAM ready");
      this->interface->onBoardLED->led[0] = this->interface->onBoardLED->LED_GREEN;
      this->interface->onBoardLED->statusLED(100, 2);
  }
  
  this->interface->sendBLEstring( dataStr + "\n" ); 
  return true;
}
// ****************************************************************************
void APRS_CWOP_CLASS::settings_defaults(){

    this->config.aprsServer                   = "cwop5.aprs.net";   // CWOP APRS-IS server address to connect to
    this->config.aprsPort                     = 14580;              // CWOP APRS-IS port

    this->config.aprsCallSign                 = "null";
    this->config.aprsPassCode                 = "-1";
    this->config.aprsLocation                 = "null";

    this->config.NUM_SAMPLE_SAMPLING_READINGS = 16;
    this->config.SAMPLING_INTERVAL            = 100; //ms
    this->config.MEASUREMENTS_BUFFER_SIZE     = 5;
    this->NUMBER_OF_SENSORS_DATA_VALUES       = 0;

    this->config.UPLOAD_DATASET_DELTA_TIME    = this->config.NUM_SAMPLE_SAMPLING_READINGS*this->config.SAMPLING_INTERVAL + 120000; // 10 min
    this->config.MEASUREMENT_INTERVAL         = this->config.NUM_SAMPLE_SAMPLING_READINGS*this->config.SAMPLING_INTERVAL + 1*60*1000; // 5min
    this->LAST_DATASET_UPLOAD                 = 0;
    this->LAST_DATA_MEASUREMENTS              = 0;
    this->MAX_LATENCY_ALLOWED                 = (unsigned long)(this->config.MEASUREMENT_INTERVAL/2);

    this->config.SPECIMEN_REF_NAME = "SPECIMEN_NO_REF";

    // Reference resistances : loaded from config file
    uint8_t SELECTED_ADC_REF_RESISTANCE=0;
    
    this->config.ADC_REF_RESISTANCE[0] = 1032;
    this->config.ADC_REF_RESISTANCE[1] = 19910;
    this->config.ADC_REF_RESISTANCE[2] = 200000;
    this->config.ADC_REF_RESISTANCE[3] = 2000000;

    this->config.channel_2_switch_en = false;
    this->config.channel_1_switch_en = false;
    this->config.channel_1_switch_on_pos = 0;
    this->ch2_sensor_type ="disabled";

    this->Measurments_NEW=false;
    this->Measurments_EN=false;
    this->DATASET_NUM_SAMPLES =0;
    this->DATASET_NUM_SAMPLES_TOTAL =0;
      // ADC Power
    pinMode( this->ENABLE_3v3_PWR_PIN, OUTPUT);
    digitalWrite( this->ENABLE_3v3_PWR_PIN,HIGH); // enabled 
}

// *************************************************************************
void APRS_CWOP_CLASS::readOnboardSensorData(){
    this->onBoardSensors->request_onBoard_Sensor_Measurements();
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES]  = String ( this->interface->rtc.getDateTime() ) + ";";
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardTHsensor->measurement[1] ) + ";" ;
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String(  this->onBoardSensors->onboardTHsensor->measurement[0] ) + ";" ;

    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[0] ) + ";" ;  
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[1] ) + ";" ;
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[2] ) + ";" ;

    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[3] ) + ";" ;
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[4] ) + ";" ;  
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[5] ) + ";" ;

    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->measurement[6] ) + ";" ;
    this->measurementsOnBoard[this->DATASET_NUM_SAMPLES] += String( this->onBoardSensors->onboardMotionSensor->sensor_n_errors ) + ";" ;
}

// ****************************************************************
void APRS_CWOP_CLASS::readChannel2SensorMeasurements ( int pos ){

 // SHT3x sensor
  if(this->ch2_sensor_type  == "sht3x"){
    this->sht3x->requestMeasurements(); 
    this->measurements [1 + n][pos] = ( this->sht3x->measurement[1] ); // Temp
    this->measurements [2 + n][pos] = ( this->sht3x->measurement[0] );  // Humidity
  }
   
  // AHT20 sensor
  if(this->ch2_sensor_type == "aht20"){
    this->aht20->requestMeasurements();
    this->measurements [1 +n ][pos] = ( this->aht20->measurement[1] ); // Temp
    this->measurements [2 + n ][pos] = ( this->aht20->measurement[0] );  // Humidity
  }

 // other sensors 
 // ...

}

// *****************************************************************************
void APRS_CWOP_CLASS::readSensorMeasurements() {  
   if (this->Measurments_EN == false )
      return;

  if (this->config.MEASUREMENT_INTERVAL < ( millis() -  this->LAST_DATA_MEASUREMENTS ) ){
    this->LAST_DATA_MEASUREMENTS=millis(); 
    this->interface->mserial->printStrln("");
    this->interface->mserial->printStrln("Requesting sensor values (" + String(this ->DATASET_NUM_SAMPLES) + ")...");

    this->readSensorMeasurements();
    
    this->scheduleWait=false;
  }

  this->readOnboardSensorData(); 

  this->ds18b20->requestMeasurements(); 
  this->measurements [0][ this->config.NUM_SAMPLE_SAMPLING_READINGS*this->DATASET_NUM_SAMPLES + i  ] =  ( this->ds18b20->measurement[0] );

  this ->DATASET_NUM_SAMPLES ++;
  this ->DATASET_NUM_SAMPLES_TOTAL ++;


    // Read the sensors
  if (millis() >= snsNextTime) {
    // Check the DHCP lease, if using DHCP
    if (ethDHCP) Ethernet.maintain();
    // Count to check if we need to send the APRS data
    if (++this->aprsMsrmCount >= this->aprsMsrmMax) {
      // Restart the counter
      this->aprsMsrmCount = 0;
      // Repeat sensor reading after the 'before' delay (long)
      snsNextTime += snsDelayBfr;
    }
    else {
      // Repeat sensor reading after the 'between' delay (short)
      snsNextTime += snsDelayBtw;
    }
    // Set the telemetry bit 7 if the station is being probed
    if (PROBE) aprsTlmBits = B10000000;

    // Check the time and set the telemetry bit 2 if time is not accurate
    unsigned long utm = timeUNIX();
    if (!timeOk) aprsTlmBits |= B00000100;

    // Set the telemetry bit 1 if the uptime is less than one day (recent reboot)
    if (millis() < 86400000UL) aprsTlmBits |= B00000010;

    // Read BMP280
    float temp, pres;
    if (atmo_ok) {
      // Set the bit 5 to show the sensor is present (reverse)
      aprsTlmBits |= B01000000;
      // Get the weather parameters
      temp = atmo.readTemperature();
      pres = atmo.readPressure();
      // Add to the round median filter
      rMedIn(MD_TEMP, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      rMedIn(MD_PRES, (int)(pres * this->altCorr(this->config.altMeters) / 10.0));  // Store directly sea level in dPa
    }

    // Read BH1750, illuminance value in lux
    uint16_t lux = light.readLightLevel();
    // Calculate the solar radiation in W/m^2
    // FIXME this is in cW/m^2
    int solRad = (int)(lux * 0.79);
    // Set the bit 5 to show the sensor is present (reverse) and there is any light
    if (solRad > 0) aprsTlmBits |= B00100000;
    // Set the bit 4 to show the sensor is saturated
    if (solRad > 999) aprsTlmBits |= B00010000;
    // Add to round median filter
    rMedIn(MD_SRAD, solRad);

    // Read Vcc (mV) and add to the round median filter
    int vcc = readVcc();
    rMedIn(MD_VCC, vcc);
    if (vcc < 4750 or vcc > 5250) {
      // Set the bit 3 to show the USB voltage is wrong (5V +/- 5%)
      aprsTlmBits |= B00001000;
    }

    // Various analog telemetry
    int a0 = readAnalog(A0);
    int a1 = readAnalog(A1);

    // Add to round median filter, mV (a / 1024 * Vcc)
    rMedIn(MD_A0, (vcc * (unsigned long)a0) / 1024);
    rMedIn(MD_A1, (vcc * (unsigned long)a1) / 1024);

    // Upper part
    // 500 / R(kO); R = R0(1024 / x - 1)
    // Lower part
    // Vout=RawADC0*0.0048828125;
    // lux=(2500/Vout-500)/10;
    //int lux = 51150L / a0 - 50;

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (this->aprsMsrmCount == 0) {
      // Reset the watchdog
      wdt_reset();
      // Get RSSI (will get FALSE (0) if the modem is not working)
      // FIXME
      int rssi = 0;
      if (rssi) rMedIn(MD_RSSI, -rssi);
      // Connect to APRS server
      char aprsServerBuf[strlen_P((char*)this->config.aprsServer) + 1];
      strncpy_P(aprsServerBuf, (char*) this->config.aprsServer, sizeof(aprsServerBuf));
      
      if (ethClient.connect(aprsServerBuf, aprsPort)) {
        // Reset the watchdog
        wdt_reset();
        // Authentication
        aprsAuthenticate();
        // Send the position, altitude and comment in firsts minutes after boot
        if (millis() < snsDelayBfr) aprsSendPosition();
        // Send weather data if the athmospheric sensor is present
        if (atmo_ok) aprsSendWeather(rMedOut(MD_TEMP), -1, rMedOut(MD_PRES), rMedOut(MD_SRAD));
        // Send the telemetry
        aprsSendTelemetry(rMedOut(MD_A0) / 20,
                          rMedOut(MD_A1) / 20,
                          rMedOut(MD_RSSI),
                          (rMedOut(MD_VCC) - 4500) / 4,
                          readMCUTemp() / 100 + 100,
                          aprsTlmBits);
        //aprsSendStatus("Fine weather");
        // Close the connection
        ethClient.stop();
        // Keep the millis the connection worked
        linkLastTime = millis();
      }
      else linkFailed();
    }
  }

  // Reset the watchdog
  wdt_reset();

}

// ******************************************************
bool APRS_CWOP_CLASS::initializeDynamicVar(  int size1D, int size2D ){    
    this->measurements = (float **)heap_caps_malloc(size1D * sizeof(float*), MALLOC_CAP_SPIRAM);
    if(this->measurements == NULL){
      //this->freeAllocatedMemory(measurements,i1D);
      this->interface->mserial->printStrln("FAIL 1D alloc");
      return false;
    }

    for (int i = 0; i < size1D; i++) {
      this->measurements[i] = (float *)heap_caps_malloc(size2D * sizeof(float), MALLOC_CAP_SPIRAM);
      if(this->measurements[i] == NULL){
        //this->freeAllocatedMemory(measurements,i1D);
        this->interface->mserial->printStrln("FAIL 2D alloc");
        return false;
      }
    }
 
    for (int i = 0; i < size1D; i++) {
      for (int j = 0; j < size2D; j++) {
          this->measurements[i][j] = 0.0f; 
      }
    }
    return true;   
  } // initializeDynamicVar

  //Free Allocated memory
  void MEASUREMENTS::freeAllocatedMemory(int nRow, int nColumn){
    for (int i = 0; i < nRow; i++) {
      delete[] this->measurements[i]; 
    }
    delete [] this->measurements; 
    this->measurements = NULL;
  }

// --------------------------------------------------------------------------

bool APRS_CWOP_CLASS::saveSettings(fs::FS &fs){
    this->interface->mserial->printStrln( this->interface->DeviceTranslation("save_daq_settings")  + "...");

    if (fs.exists("/measurements.cfg") )
        fs.remove("/measurements.cfg");

    File settingsFile = fs.open("/measurements.cfg", FILE_WRITE); 
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
    File settingsFile = fs.open("/measurements.cfg", FILE_READ);
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

// -------------------------------------------------------------------------------
// ********************************************************
bool APRS_CWOP_CLASS:: gbrl_summary_measurement_config( uint8_t sendTo){
    String dataStr = this->interface->DeviceTranslation("config_summary") +  ":\n";
    dataStr += this->interface->DeviceTranslation("mi_interval") +  ": " + String(this->config.MEASUREMENT_INTERVAL/1000) + " sec.\n\n";
    this->interface->sendBLEstring( dataStr , sendTo); 
    return true;
}


// *******************************************************
bool APRS_CWOP_CLASS::cfg_commands(String $BLE_CMD, uint8_t sendTo){
    String dataStr="";
    long int hourT; 
    long int minT; 
    long int secT; 
    long int daysT;
    long int $timedif;

    if($BLE_CMD.indexOf("$cfg mi ")>-1){
        String value= $BLE_CMD.substring(11, $BLE_CMD.length());
        if (isNumeric(value)){
            long int val= (long int) value.toInt();
            if(val>0){
                this->config.MEASUREMENT_INTERVAL=val*1000; // mili seconds 
                this->saveSettings(LittleFS);

                hourT = (long int) ( this->config.MEASUREMENT_INTERVAL/(3600*1000) );
                minT  = (long int) ( this->config.MEASUREMENT_INTERVAL/(60*1000) - (hourT*60));
                secT  = (long int) ( this->config.MEASUREMENT_INTERVAL/1000 - (hourT*3600) - (minT*60));
                daysT = (long int) (hourT/24);
                hourT = (long int) ( (this->config.MEASUREMENT_INTERVAL/(3600*1000) ) - (daysT*24));
                
                dataStr = this->interface->DeviceTranslation("new_mi_accepted") +  "\r\n\n";        
                dataStr += " ["+String(daysT)+"d "+ String(hourT)+"h "+ String(minT)+"m "+ String(secT)+"s "+ String("]\n\n");
                this->interface->sendBLEstring( dataStr, sendTo);
            }else{
                dataStr= this->interface->BaseTranslation("invalid_input") +  "\r\n";
                this->interface->sendBLEstring( dataStr, sendTo);
            }
            return true;
        }
    }
    return false;
}

// *******************************************************
bool APRS_CWOP_CLASS::measurementInterval(String $BLE_CMD, uint8_t sendTo){
    if($BLE_CMD !="$MEASURE INTERVAL" && $BLE_CMD !="$measure interval")
        return false;

    long int hourT; 
    long int minT; 
    long int secT; 
    long int daysT;
    long int $timedif;
    String dataStr="";

    hourT = (long int) (this->config.MEASUREMENT_INTERVAL/(3600*1000) );
    minT = (long int) (this->config.MEASUREMENT_INTERVAL/(60*1000) - (hourT*60));
    secT =  (long int) (this->config.MEASUREMENT_INTERVAL/1000 - (hourT*3600) - (minT*60));
    daysT = (long int) (hourT/24);
    hourT = (long int) ((this->config.MEASUREMENT_INTERVAL/(3600*1000) ) - (daysT*24));

    dataStr= this->interface->DeviceTranslation("mi_interval")  + String(char(10)) + String(daysT)+"d "+ String(hourT)+"h "+ String(minT)+"m "+ String(secT)+"s "+ String(char(10));
    this->interface->sendBLEstring( dataStr, sendTo);
    return true;
}


// ********************************************************
bool APRS_CWOP_CLASS::history(String $BLE_CMD, uint8_t sendTo){
    if($BLE_CMD != "$history"  )
        return false;

    long int hourT; 
    long int minT; 
    long int secT; 
    long int daysT;
    String dataStr="";
    long int $timedif;
    time_t timeNow;
    time(&timeNow);

    dataStr = this->interface->DeviceTranslation("calc_mi_st_val") +  "..."+ this->interface->BaseTranslation("wait_moment") +"." +String(char(10));
    this->interface->sendBLEstring( dataStr, sendTo);

    this->interface->setMCUclockFrequency(this->interface->MAX_FREQUENCY);

    dataStr = "\n"+ this->interface->DeviceTranslation("data_history") +String(char(10));

    File file = LittleFS.open("/" + this->interface->config.SENSOR_DATA_FILENAME, "r");
    int counter=0; 

    long int sumTimeDelta=0;

    while (file.available()) {
        this->interface->sendBLEstring( "#", sendTo);

        if (counter == 0 ) {
            // raed the header
            String bin2 = file.readStringUntil( char(10) );
            bin2 = file.readStringUntil( char(10) );
        }

        String bin = file.readStringUntil( ';' ); // RTC Date & Time

        long int timeStart = atol(file.readStringUntil( (char) ';' ).c_str() ); //start time of measure 

        long int timeDelta = atol(file.readStringUntil( (char) ';' ).c_str() ); // delta time since last measure
        sumTimeDelta+=timeDelta; //elapsed time since start time of measure

        float temp = (file.readStringUntil( (char) ';' ).toFloat()); // probe temp.

        String rest = file.readStringUntil( char(10) ); // remaider of data string line

        $timedif = (timeStart+sumTimeDelta) - timeStart;
        hourT = (long int) ($timedif / (3600*1000));
        minT = (long int) ($timedif/ (60*1000) - (hourT*60));
        secT =  (long int) ($timedif/1000 - (hourT*3600) - (minT*60));
        daysT = (long int) (hourT/24);
        hourT = (long int) (($timedif/(3600*1000) ) - (daysT*24));

        dataStr +=  ": ["+String(daysT)+"d "+ String(hourT)+"h "+ String(minT)+"m "+ String(secT)+"s "+ String("]  ");    
        dataStr +=  this->interface->DeviceTranslation("probe_temp") + ": ";
        dataStr += String(roundFloat(temp,2))+String(char(176))+String("C  ");

        counter++; 
    }

    file.close();
  
    dataStr += "--------------- \n";
    this->interface->sendBLEstring( dataStr, sendTo);
    this->interface->setMCUclockFrequency( this->interface->CURRENT_CLOCK_FREQUENCY);
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
// GBRL commands --------------------------------------------------------------
 bool APRS_CWOP_CLASS::helpCommands(String $BLE_CMD, uint8_t sendTo){
    if($BLE_CMD != "$?" && $BLE_CMD !="$help" )
        return false;

    String dataStr="Measurements Commands:\n" \
                    "$ufid                  - "+ this->interface->DeviceTranslation("ufid") +" unique fingerprint ID\n" \
                    "$me new                - "+ this->interface->DeviceTranslation("me_new") +"\n" \
                    "$me start              - "+ this->interface->DeviceTranslation("me_start") +"\n" \
                    "$me end                - "+ this->interface->DeviceTranslation("me_end") +"\n" \
                    "$me status             - "+ this->interface->DeviceTranslation("me_status") +"\n" \
                    "\n" \        
                    "$history               - "+ this->interface->DeviceTranslation("history") +"\n" \
                    "$ns                    - "+ this->interface->DeviceTranslation("ns") +"\n" \
                    "$mi                    - "+ this->interface->DeviceTranslation("mi") +"\n" \      
                    "$set mi [sec]          - "+ this->interface->DeviceTranslation("set_mi") +"\n\n";

    this->interface->sendBLEstring( dataStr, sendTo);
      
    return false;
 }

// -------------------------------------------------------------------------------


bool APRS_CWOP_CLASS::gbrl_commands(String $BLE_CMD, uint8_t sendTo){
    String dataStr="";

    if ($BLE_CMD == "$view ch2"){
      dataStr = "Channel 2 current configuration is : " + String(this->ch2_sensor_type) +"\n";
      this->interface->sendBLEstring( dataStr , sendTo); 
      return true;
    }

    if ($BLE_CMD == "$view ch1"){
      dataStr = "Current configuration on the channel 1 switch is :\n";
      if (this->config.channel_1_switch_en){
        dataStr += "Enabled, switch " + String(this->config.channel_1_switch_on_pos) + " is ON all other are set to OFF.\n";
      } else {
        dataStr += "Disabled. (all switches are set to OFF)\n";
      }
      this->interface->sendBLEstring( dataStr + "\n" , sendTo); 
      return true;
    }

    if( $BLE_CMD.equals( "$set ch2 on" ) || $BLE_CMD.equals( "$set ch2 off" ) ){
      return this->sw_commands( $BLE_CMD,  sendTo);
    }
    
    if($BLE_CMD.indexOf("$set sw")>-1){
      return this->sw_commands( $BLE_CMD,  sendTo);
    }

    if ($BLE_CMD == "$ufid"){
      if (this->DATASET_NUM_SAMPLES == 0){
          dataStr = this->interface->DeviceTranslation("no_data_entries") + "." +String(char(10));
          this->interface->sendBLEstring( dataStr, sendTo);
          return true;
      }
      dataStr =  this->interface->DeviceTranslation("calc_ufid") + "..." + this->interface->BaseTranslation("wait_moment")  + "." +String(char(10));
      this->interface->sendBLEstring( dataStr, sendTo);
      
      this->interface->setMCUclockFrequency(this->interface->MAX_FREQUENCY);
      
      dataStr += "Unique Data Fingerprint ID:"+String(char(10));
      dataStr += CryptoICserialNumber(this->interface)+"-"+macChallengeDataAuthenticity(this->interface, String(this->interface->rtc.getDateTime(true)) + String(roundFloat(this->last_measured_probe_temp,2)) );
      dataStr += String(char(10) + String(char(10)) );
      
      this->interface->setMCUclockFrequency(this->interface->CURRENT_CLOCK_FREQUENCY);
      this->interface->sendBLEstring( dataStr, sendTo);
      return true;
    }

    if($BLE_CMD == "$mi"){
      dataStr= this->interface->DeviceTranslation("curr_measure_interval") + " " + String(roundFloat(this->config.MEASUREMENT_INTERVAL/(60*1000) ,2)) + String(" min") + String(char(10));
      this->interface->sendBLEstring( dataStr, sendTo);
      return true;
    }

    if( $BLE_CMD == "$me status"){
      if( this->Measurments_EN == false){
          dataStr = this->interface->DeviceTranslation("measure_not_started") +  String("\n\n");
      } else{
          dataStr = this->interface->DeviceTranslation("measure_already_started") + String("\n");
          dataStr += this->interface->DeviceTranslation("measure_num_records") + " " + String(this->DATASET_NUM_SAMPLES) + "\n\n";
      }

      this->interface->sendBLEstring( dataStr, sendTo); 
      return true;
    }
    if( $BLE_CMD == "$me new"){
        this->Measurments_NEW=true;
        this->Measurments_EN=false;
        this->DATASET_NUM_SAMPLES=0;
        dataStr= this->interface->DeviceTranslation("new_started") +  String("\n\n");
        this->interface->sendBLEstring( dataStr, sendTo); 
        return true;
    }
    if($BLE_CMD == "$me start"){
        if (this->Measurments_EN){
            dataStr = this->interface->DeviceTranslation("measure_already_started_on") +  " " + String(this->measurement_Start_Time) + String("\n\n");
            this->interface->sendBLEstring( dataStr, sendTo); 
        }else{
            this->DATASET_NUM_SAMPLES = 0;
            this ->DATASET_NUM_SAMPLES_TOTAL = 0;
            this->Measurments_NEW=true;
            this->Measurments_EN=true;
            this->measurement_Start_Time = this->interface->rtc.getDateTime(true);
            this->initializeSensors();
            
            dataStr = this->interface->DeviceTranslation("measure_started_on") +  " " + String(this->measurement_Start_Time) + String("\n");
            this->interface->sendBLEstring( dataStr, sendTo); 
            
            this->gbrl_summary_measurement_config(sendTo);
        }
        return true;

    }
    if( $BLE_CMD=="$me end"){
        if(this->Measurments_EN==false){
            dataStr= this->interface->DeviceTranslation("measure_already_ended")  + String( char(10));
        }else{
            this->Measurments_EN=false;
            dataStr= this->interface->DeviceTranslation("measure_ended_on") +  " " + String(this->interface->rtc.getDateTime(true)) + String( char(10));
        }
        this->interface->sendBLEstring( dataStr, sendTo); 
        return true;
    }
    if($BLE_CMD=="$ns"){
        dataStr = this->interface->DeviceTranslation("num_data_measure") +  ": " + String(this->DATASET_NUM_SAMPLES+1) + String(char(10));
        this->interface->sendBLEstring( dataStr, sendTo);
        return true;
    }
    bool result =false;
    result = this->helpCommands( $BLE_CMD,  sendTo);
    
    bool result2 =false;
    result2 = this->history($BLE_CMD,  sendTo);
    
    bool result3 =false;
    result3 = this->cfg_commands($BLE_CMD,  sendTo);
    
    bool result4 =false;
    result4 = this->measurementInterval($BLE_CMD,  sendTo);    
    
    //this->gbrl_menu_selection();
   
   return ( result || result2 || result3 || result4 );

}