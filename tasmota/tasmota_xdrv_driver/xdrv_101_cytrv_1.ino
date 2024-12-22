/*
  xdrv_101_cytrv_.ino - My IoT device support for Tasmota
*/

#ifdef USE_I2C

#ifdef USE_CYTRV_1
/*********************************************************************************************\
 * My IoT Device with command support
 *
 *
\*********************************************************************************************/

#warning **** xdrv cyTRV_1 is included... ****

#define XDRV_101 101

#ifdef USE_INA219
  #undef USE_INA219
  #warning **** INA219 from tasmota was deactivated... ****
#endif

// INA219 part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <Wire.h>
#include "INA219_WE.h"
#define I2C_ADDRESS 0x40

/* There are several ways to create your INA219 object:
   INA219_WE ina219 = INA219_WE(); -> uses Wire / I2C Address = 0x40
   INA219_WE ina219 = INA219_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
   INA219_WE ina219 = INA219_WE(&Wire); -> you can pass any TwoWire object
   INA219_WE ina219 = INA219_WE(&Wire, I2C_ADDRESS); -> all together
*/
INA219_WE ina219 = INA219_WE(I2C_ADDRESS);


float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina219_overflow = false;

float gv_max_curr = 50;



/*********************************************************************************************\
 * My IoT Device Functions
\*********************************************************************************************/

// This variable will be set to true after initialization
bool initSuccess = false;

/* 
  Optional: if you need to pass any command for your device 
  Commands are issued in Console or Web Console
  Commands:
    Say_Hello  - Only prints some text. Can be made something more useful...
    SendMQTT   - Send a MQTT example message
    Help       - Prints a list of available commands  
*/


bool init_ina219() {
  //Wire.begin();
  //Wire.begin(/*SDA*/D2,/*SCL*/D1); //(D1mini)
  if (!ina219.init()) {
    //Serial.println("INA219 not connected!");
    return false;
  } else {
    // Serial.println("INA219 connected!");
  }

  /* Set ADC Mode for Bus and ShuntVoltage
    Mode *            * Res / Samples *       * Conversion Time
    BIT_MODE_9        9 Bit Resolution             84 µs
    BIT_MODE_10       10 Bit Resolution            148 µs
    BIT_MODE_11       11 Bit Resolution            276 µs
    BIT_MODE_12       12 Bit Resolution            532 µs  (DEFAULT)
    SAMPLE_MODE_2     Mean Value 2 samples         1.06 ms
    SAMPLE_MODE_4     Mean Value 4 samples         2.13 ms
    SAMPLE_MODE_8     Mean Value 8 samples         4.26 ms
    SAMPLE_MODE_16    Mean Value 16 samples        8.51 ms
    SAMPLE_MODE_32    Mean Value 32 samples        17.02 ms
    SAMPLE_MODE_64    Mean Value 64 samples        34.05 ms
    SAMPLE_MODE_128   Mean Value 128 samples       68.10 ms
  */
  //ina219.setADCMode(SAMPLE_MODE_128); // choose mode and uncomment for change of default
  ina219.setADCMode(SAMPLE_MODE_16);
  /* Set measure mode
    POWER_DOWN - INA219 switched off
    TRIGGERED  - measurement on demand
    ADC_OFF    - Analog/Digital Converter switched off
    CONTINUOUS  - Continuous measurements (DEFAULT)
  */
  // ina219.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default

  /* Set PGain
    Gain *  * Shunt Voltage Range *   * Max Current (if shunt is 0.1 ohms)
    PG_40       40 mV                    0.4 A
    PG_80       80 mV                    0.8 A
    PG_160      160 mV                   1.6 A
    PG_320      320 mV                   3.2 A (DEFAULT)
  */
  // ina219.setPGain(PG_320); // choose gain and uncomment for change of default

  /* Set Bus Voltage Range
    BRNG_16   -> 16 V
    BRNG_32   -> 32 V (DEFAULT)
  */
  // ina219.setBusRange(BRNG_32); // choose range and uncomment for change of default

  //Serial.println("INA219 Current Sensor Example Sketch - Continuous");

  /* If the current values delivered by the INA219 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA219
  */
  // ina219.setCorrectionFactor(0.98); // insert your correction factor if necessary

  /* If you experience a shunt voltage offset, that means you detect a shunt voltage which is not
     zero, although the current should be zero, you can apply a correction. For this, uncomment the
     following function and apply the offset you have detected.
  */
  // ina219.setShuntVoltOffset_mV(0.5); // insert the shunt voltage (millivolts) you detect at zero current

return true;

}

const char MyProjectCommands[] PROGMEM = "|"  // No Prefix
  "Say_Hello|" 
  "SendMQTT|"
  "Help";

void (* const MyProjectCommand[])(void) PROGMEM = {
  &CmdSay_Hello, &CmdSendMQTT, &CmdHelp };

void CmdSay_Hello(void) {
  //AddLog(LOG_LEVEL_INFO, PSTR("Say_Hello: Hello!"));
  ResponseCmndDone();
}

char payload[200];
char topic[100];

void CmdSendMQTT(void) {
  //AddLog(LOG_LEVEL_INFO, PSTR("Sending MQTT message."));

  snprintf_P(topic, sizeof(topic), PSTR("tasmota/myproject"));

  snprintf_P(payload, sizeof(payload), 
            PSTR("{\"" D_JSON_TIME "\":\"%s\",\"name\":\"My Project\"}"), 
            GetDateAndTime(DT_LOCAL).c_str()
  );

  // retain = true
  MqttPublishPayload(topic, payload, strlen(payload), false);

  ResponseCmndDone();
}

void CmdHelp(void) {
  //AddLog(LOG_LEVEL_INFO, PSTR("Help: Accepted commands - Say_Hello, SendMQTT, Help"));
  ResponseCmndDone();
}

/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/



void MyProjectInit()
{

  /*
    Here goes My Project setting.
    Usually this part is included into setup() function
  */


  //AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("My Project init..."));

  //Serial.begin(115200);
init_ina219();

  // Set initSuccess at the very end of the init process
  // Init is successful
  initSuccess = true;

}



void MyProjectProcessing(void)
{

  /*
    Here goes My Project code.
    Usually this part is included into loop() function
  */

}






/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdrv101(uint32_t function)
{


  bool result = false;

  if (FUNC_INIT == function) {
    MyProjectInit();
    //AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("My project init is done..."));
  }
  else if (initSuccess) {

    switch (function) {
      // Select suitable interval for polling your function
//    case FUNC_EVERY_SECOND:
      case FUNC_EVERY_250_MSECOND:
//    case FUNC_EVERY_200_MSECOND:
//    case FUNC_EVERY_100_MSECOND:
        MyProjectProcessing();
        break;

      // Command support
      case FUNC_COMMAND:
        //AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Calling My Project Command..."));
        result = DecodeCommand(MyProjectCommands, MyProjectCommand);
        break;

    }

  }

  return result;
}

#endif  // USE_CYTRV_1
#endif