/*
  XDRV_102_cytrv_.ino - My IoT device support for Tasmota
*/

#ifdef USE_I2C

#ifdef USE_CYTRV_2
/*********************************************************************************************\
 * TRV driver
 *
 *
\*********************************************************************************************/

#warning **** xdrv cyTRV_101 is included... ****

#define XDRV_102 102

#ifdef USE_INA219
#undef USE_INA219
#warning **** INA219 from tasmota was deactivated... ****
#endif

#ifndef USE_BH1750
#define USE_BH1750 // [I2cDriver11] Enable BH1750 sensor (I2C address 0x23 or 0x5C) (+0k5 code)
#warning **** BH1750 from tasmota was activated... ****
#endif

#ifndef USE_DS18x20
#define USE_DS18x20 // Add support for DS18x20 sensors with id sort, single scan and read retry (+2k6 code)
//  #define W1_PARASITE_POWER     // Optimize for parasite powered sensors
#warning **** DS18x20 from tasmota was activated... ****
#endif

#ifndef USE_MLX90614
#define USE_MLX90614 // [I2cDriver32] Enable MLX90614 ir temp sensor (I2C address 0x5a) (+0.6k code)
#warning **** MLX90614 from tasmota was activated... ****
#endif

#ifdef MODULE
#undef MODULE
#endif
#define MODULE USER_MODULE // [Module] Select default model (the list is kModuleNiceList() in file tasmota_template.h) USER_MODULE is the TEMPLATE

#ifdef FALLBACK_MODULE
#undef FALLBACK_MODULE
#endif
#define FALLBACK_MODULE USER_MODULE // to Select the default model as FALLBACK when the user does a RESET 1

#ifdef USER_TEMPLATE
#undef USER_TEMPLATE
#endif
#define USER_TEMPLATE "{\"NAME\":\"cyTRV\",\"GPIO\":[1,1,576,1,640,608,1,1,162,1312,161,1,1,1],\"FLAG\":0,\"BASE\":18}" // [Template] Set JSON template
#warning **** User_Template cyTRV was activated... ****

#define D_cyTRV "cyTRV"

//*********************************************************************************************/
// INA219 decl ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/
#include <Wire.h>
#include "INA219_WE.h"

#define XDRV_102_I2C_ADDRESS 0x40

/* There are several ways to create your INA219 object:
   INA219_WE ina219 = INA219_WE(); -> uses Wire / I2C Address = 0x40
   INA219_WE ina219 = INA219_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
   INA219_WE ina219 = INA219_WE(&Wire); -> you can pass any TwoWire object
   INA219_WE ina219 = INA219_WE(&Wire, I2C_ADDRESS); -> all together
*/
INA219_WE ina219 = INA219_WE(XDRV_102_I2C_ADDRESS);
const char *XDRV_102_INA219_TYPE[] = {"INA219", "ISL28022"};

struct XDRV_102_INA219
{
  bool active;
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0;
  bool ina219_overflow = false;

  float max_curr = 50;
} XDRV_102_ina219;

#ifdef USE_WEBSERVER
const char XDRV_102_HTTP_SNS_INA219_DATA[] PROGMEM =
    "{s}%s " D_VOLTAGE "{m}%s " D_UNIT_VOLT "{e}"
    "{s}%s " D_CURRENT "{m}%s " D_UNIT_AMPERE "{e}"
    "{s}%s " D_POWERUSAGE "{m}%s " D_UNIT_WATT "{e}";
#endif // USE_WEBSERVER

//*********************************************************************************************/
// MQTT decl  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

struct XDRV_102_MQTT
{
  boolean log_mqtt;
  boolean pub_sens;
  int dest_pos;
  boolean cal;
} XDRV_102_mqtt;

//*********************************************************************************************/
// State decl ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

struct XDRV_102_STATE
{
  unsigned long State_millis;

  unsigned long dest_millis;
  unsigned long old_millis;

  int state;
  int state_old;
  int max_time;
  int pos_time;

} XDRV_102_state;

//*********************************************************************************************/
// Motor decl ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

#include "WEMOS_Motor.h"
// Motor shield I2C Address: 0x30
// PWM frequency: 1000Hz(1kHz)
// Motor Pumpe(0x30, _MOTOR_A, 1000); //Motor A
Motor *XDRV_102_Ventil;

struct XDRV_102_MOTOR
{
  int pwm;

  boolean run = false;
  boolean dir = false;

  int act_pos;
  int dest_pos;
  int old_pos;

} XDRV_102_motor;

#ifdef USE_WEBSERVER
const char XDRV_102_HTTP_SNS_TRV_DATA[] PROGMEM =
    "{s}%s  Position {m}%s " D_UNIT_PERCENT " {e}"
    "{s}%s  State {m}%s  {e}"
    "{s}%s  max_time {m}%s " D_UNIT_SECOND " {e}";
#endif // USE_WEBSERVER

/*********************************************************************************************/
// INA219 part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

bool XDRV_102_init_ina219()
{
  XDRV_102_ina219.active = false;

  if (!ina219.init())
  {
    return XDRV_102_ina219.active;
  }

  I2cSetActiveFound(XDRV_102_I2C_ADDRESS, XDRV_102_INA219_TYPE[0]);

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
  // ina219.setADCMode(SAMPLE_MODE_128); // choose mode and uncomment for change of default
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

  // Serial.println("INA219 Current Sensor Example Sketch - Continuous");

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
  XDRV_102_ina219.active = true;
  return XDRV_102_ina219.active;
}

void XDRV_102_check_ina219()
{

  if (XDRV_102_ina219.active == false)
  {
    return;
  }

  XDRV_102_ina219.shuntVoltage_mV = ina219.getShuntVoltage_mV();
  XDRV_102_ina219.busVoltage_V = ina219.getBusVoltage_V();
  XDRV_102_ina219.current_mA = ina219.getCurrent_mA();
  XDRV_102_ina219.power_mW = ina219.getBusPower();
  XDRV_102_ina219.loadVoltage_V = XDRV_102_ina219.busVoltage_V + (XDRV_102_ina219.shuntVoltage_mV / 1000);
  XDRV_102_ina219.ina219_overflow = ina219.getOverflow();
}

void XDRV_102_show_INA219(bool json)
{
  if (!XDRV_102_ina219.active)
  {
    return;
  }

  char voltage[16];
  dtostrfd(XDRV_102_ina219.busVoltage_V, Settings->flag2.voltage_resolution, voltage);
  char current[16];
  dtostrfd(XDRV_102_ina219.current_mA / 1000, Settings->flag2.current_resolution, current);
  char power[16];
  dtostrfd(XDRV_102_ina219.power_mW / 1000, Settings->flag2.wattage_resolution, power);
  char name[16];
  snprintf_P(name, sizeof(name), PSTR("%s"), XDRV_102_INA219_TYPE[0]);

  if (json)
  {
    ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_VOLTAGE "\":%s,\"" D_JSON_CURRENT "\":%s,\"" D_JSON_POWERUSAGE "\":%s}"),
                     name, XDRV_102_I2C_ADDRESS, voltage, current, power);

#ifdef USE_WEBSERVER
  }
  else
  {
    WSContentSend_PD(XDRV_102_HTTP_SNS_INA219_DATA, name, voltage, name, current, name, power);
#endif // USE_WEBSERVER
  }
}
//*********************************************************************************************/
// MQTT part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/
void XDRV_102_show_TRV(bool json)
{
  char position[16];
  dtostrfd(XDRV_102_motor.act_pos, 0, position);
  // dtostrfd(XDRV_102_ina219.busVoltage_V, 0, position);
  char state[16];
  dtostrfd(XDRV_102_state.state, 0, state);
  char max_time[16];
  dtostrfd(XDRV_102_state.max_time, 0, max_time);
  char name[16];
  snprintf_P(name, sizeof(name), PSTR("%s"), D_cyTRV);

  if (json)
  {
    ResponseAppend_P(PSTR(",\"%s\":{\"Position\":%s,\"State\":%s,\"Max_Time\":%s}"),
                     name, position, state, max_time);

#ifdef USE_WEBSERVER
  }
  else
  {
    WSContentSend_PD(XDRV_102_HTTP_SNS_TRV_DATA, name, position, name, state, name, max_time);
#endif // USE_WEBSERVER
  }
}

//*********************************************************************************************/
// State part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

void XDRV_102_check_state(void)
{

  XDRV_102_state.state_old = XDRV_102_state.state;

  if (XDRV_102_state.state != 1)
  {
    XDRV_102_check_ina219();
  }

  switch (XDRV_102_state.state)
  {
  case 0: // do stop
    XDRV_102_state_motor_stop();
    break;
  case 1: // stopped
    if (XDRV_102_state.state == 1)
    {
      if (XDRV_102_mqtt.dest_pos != -1)
      {
        XDRV_102_motor.dest_pos = XDRV_102_mqtt.dest_pos;
        XDRV_102_state.state = 6;
        XDRV_102_mqtt.dest_pos = -1;
      }
    }

    if (XDRV_102_state.state == 1)
    {
      if (XDRV_102_mqtt.cal == true)
      {
        XDRV_102_state.state = 2; // calibration
        XDRV_102_mqtt.cal = false;
      }
    }
    break;
  case 2: // do calibration
    XDRV_102_motor.act_pos = -1;
    XDRV_102_Ventil->setmotor(_CW, 100);
    XDRV_102_motor.dir = true;
    XDRV_102_motor.run = true;
    XDRV_102_state.state = 3;
  case 3: // calibration opening
    if (XDRV_102_ina219.current_mA > XDRV_102_ina219.max_curr)
    {
      XDRV_102_Ventil->setmotor(_STOP);
      XDRV_102_motor.run = false;
      XDRV_102_state.state = 4;
    }
    break;
  case 4: // do calibration close
    XDRV_102_Ventil->setmotor(_CCW, 100);
    XDRV_102_motor.dir = false;
    XDRV_102_motor.run = true;
    XDRV_102_state.max_time = 0;
    XDRV_102_state.state = 5;

    break;

  case 5: // calibration closing
    if (XDRV_102_ina219.current_mA > XDRV_102_ina219.max_curr)
    {
      XDRV_102_state_motor_stop();
      XDRV_102_motor.act_pos = 0;
    }
    break;
  case 6: // do positioning
    if (XDRV_102_motor.act_pos == XDRV_102_motor.dest_pos)
    {
      XDRV_102_state.state = 1;
      break;
    }

    XDRV_102_motor.old_pos = XDRV_102_motor.act_pos;

    if (XDRV_102_motor.act_pos > XDRV_102_motor.dest_pos)
    {
      // millis für neue Position errechnen
      int lv_diff_pos = XDRV_102_motor.act_pos - XDRV_102_motor.dest_pos;
      int lv_diff_millis = ((XDRV_102_state.max_time * 1000) / 100) * lv_diff_pos;
      XDRV_102_state.old_millis = millis();
      XDRV_102_state.dest_millis = XDRV_102_state.old_millis + lv_diff_millis;

      XDRV_102_Ventil->setmotor(_CCW, 100);
      XDRV_102_motor.dir = false;
      XDRV_102_motor.run = true;
      XDRV_102_state.state = 7;
    }
    else
    {
      // millis für neue Position errechnen
      int lv_diff_pos = XDRV_102_motor.dest_pos - XDRV_102_motor.act_pos;
      int lv_diff_millis = ((XDRV_102_state.max_time * 1000) / 100) * lv_diff_pos;
      XDRV_102_state.old_millis = millis();
      XDRV_102_state.dest_millis = XDRV_102_state.old_millis + lv_diff_millis;

      XDRV_102_Ventil->setmotor(_CW, 100);
      XDRV_102_motor.dir = true;
      XDRV_102_motor.run = true;
      XDRV_102_state.state = 8;
    }
    // gv_pos_time = gv_max_time - map(gv_dest_pos, 0, 100, gv_max_time, 0);
    break;
  case 7: // go to position closing
    if (XDRV_102_ina219.current_mA > XDRV_102_ina219.max_curr)
    {
      XDRV_102_state_motor_stop();
      XDRV_102_motor.act_pos = 0;
      break;
    }

    // aktuelle Position berechnen
    XDRV_102_motor.act_pos = XDRV_102_motor.old_pos - ((100 * (millis() - XDRV_102_state.old_millis)) / (XDRV_102_state.max_time * 1000));

    if (XDRV_102_motor.act_pos <= XDRV_102_motor.dest_pos)
    {
      XDRV_102_state_motor_stop();
      break;
    }
    break;
  case 8: // go to position opening
    if (XDRV_102_ina219.current_mA > XDRV_102_ina219.max_curr)
    {
      XDRV_102_state_motor_stop();
      XDRV_102_motor.act_pos = 100;

      break;
    }

    // aktuelle Position berechnen
    XDRV_102_motor.act_pos = XDRV_102_motor.old_pos + ((100 * (millis() - XDRV_102_state.old_millis)) / (XDRV_102_state.max_time * 1000));

    if (XDRV_102_motor.act_pos >= XDRV_102_motor.dest_pos)
    {
      XDRV_102_state_motor_stop();
      break;
    }
    break;
  }

  if (XDRV_102_state.state_old != XDRV_102_state.state)
  {
    XDRV_102_mqtt.pub_sens = true;
  }
}

void XDRV_102_check_state_1s()
{

  switch (XDRV_102_state.state)
  {
  case 5: // calibration closing
    XDRV_102_state.max_time = XDRV_102_state.max_time + 1;
    break;
  case 7:
    // gv_act_pos = gv_act_pos - ( 100 / gv_max_time );

    break;
  case 8:
    // gv_act_pos = gv_act_pos + ( 100 / gv_max_time );

    break;
  }
}

void XDRV_102_state_motor_stop()
{
  XDRV_102_Ventil->setmotor(_STOP);
  XDRV_102_motor.run = false;
  XDRV_102_state.state = 1;
  XDRV_102_mqtt.pub_sens = true;
  XDRV_102_check_ina219();
}

void XDRV_102_init_state()
{
  XDRV_102_state.state = 0;
  XDRV_102_state.max_time = 46;
  // set_state_tact();
}

//*********************************************************************************************/
// Motor part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/
void XDRV_102_init_motor()
{
  XDRV_102_Ventil = new Motor(0x30, _MOTOR_B, 1000); // Motor B

  XDRV_102_motor_stop_all();
}

void XDRV_102_motor_stop_all()
{
  XDRV_102_Ventil->setmotor(_STOP);
  XDRV_102_motor.run = false;
}

/*********************************************************************************************\
 * My IoT Device Functions
\*********************************************************************************************/

// This variable will be set to true after initialization
bool XDRV_102_initSuccess = false;

/*
  Optional: if you need to pass any command for your device
  Commands are issued in Console or Web Console
  Commands:
    Say_Hello  - Only prints some text. Can be made something more useful...
    SendMQTT   - Send a MQTT example message
    Help       - Prints a list of available commands
*/

const char MyProjectCommands[] PROGMEM = D_cyTRV "|" // No Prefix
                                                 "Cal|"
                                                 "Pos|"
                                                 //                                         "Say_Hello|"
                                                 //                                         "SendMQTT|"
                                                 "HELP";

void (*const MyProjectCommand[])(void) PROGMEM = {
    &CmdTRV2Cal, &CmdTRV2Pos,
    //    &CmdSay_Hello, &CmdSendMQTT,
    &CmdTRV2Help};

/* void CmdSay_Hello(void)
{
  AddLog(LOG_LEVEL_INFO, PSTR("Say_Hello: Hello!"));
  ResponseCmndDone();
}

char payload[200];
char topic[100];

void CmdSendMQTT(void)
{
  // AddLog(LOG_LEVEL_INFO, PSTR("Sending MQTT message."));

  snprintf_P(topic, sizeof(topic), PSTR("tasmota/myproject"));

  snprintf_P(payload, sizeof(payload),
             PSTR("{\"" D_JSON_TIME "\":\"%s\",\"name\":\"My Project\"}"),
             GetDateAndTime(DT_LOCAL).c_str());

  // retain = true
  MqttPublishPayload(topic, payload, strlen(payload), false);

  ResponseCmndDone();
} */

void CmdTRV2Help(void)
{
  AddLog(LOG_LEVEL_INFO, PSTR("cyTRV Help: Accepted commands - cyTRVCal, cyTRVPos, cyTRVHelp"));
  ResponseCmndDone();
}

// Command Calibration
void CmdTRV2Cal(void)
{
  // AddLog(LOG_LEVEL_INFO, PSTR("Calling XDRV_102 Command calibrate..."));
  XDRV_102_mqtt.cal = true;
  ResponseCmndDone();
}

// Command Position
void CmdTRV2Pos()
{
  // AddLog(LOG_LEVEL_INFO, PSTR("Calling XDRV_102 Command position ..."));
  if (XdrvMailbox.payload >= 0)
  {
    XDRV_102_mqtt.dest_pos = XdrvMailbox.payload;
  }
  else
  {
    //    char position[16];
    //    dtostrfd(XDRV_102_mqtt.dest_pos, 0, position);
    //    AddLog(LOG_LEVEL_INFO, position);
    MqttPublishSensor();
  }

  ResponseCmndDone();
}

/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/

void XDRV_102_Init()
{
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_cyTRV " init..."));

  // Serial.begin(115200);
  XDRV_102_init_motor();

  // init_window();

  XDRV_102_init_ina219();

  XDRV_102_init_state();

  XDRV_102_motor.dest_pos = 0;
  XDRV_102_state.state = 6;

  // Set XDRV_102_initSuccess at the very end of the init process
  // Init is successful
  XDRV_102_initSuccess = true;
}

void XDRV_102_check_1s(void)
{
  XDRV_102_check_ina219();
  XDRV_102_check_state_1s();
  if (XDRV_102_mqtt.pub_sens)
  {
    MqttPublishSensor();
    XDRV_102_mqtt.pub_sens = false;
  }
}

void XDRV_102_show(bool json)
{
  XDRV_102_show_INA219(json);
  WSContentSeparator(0);
  XDRV_102_show_TRV(json);
  XDRV_102_mqtt.pub_sens = false;
}
/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdrv102(uint32_t function)
{

  bool result = false;

  if (FUNC_INIT == function)
  {
    XDRV_102_Init();
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_cyTRV " init is done..."));
  }
  else if (XDRV_102_initSuccess)
  {

    switch (function)
    {
      // Select suitable interval for polling your function
    case FUNC_EVERY_SECOND:
      XDRV_102_check_1s();

      //    case FUNC_EVERY_250_MSECOND:
      //    case FUNC_EVERY_200_MSECOND:
      //    case FUNC_EVERY_100_MSECOND:

    case FUNC_EVERY_50_MSECOND:
      XDRV_102_check_state();
      break;

    // Command support
    case FUNC_COMMAND:
      // AddLog(LOG_LEVEL_INFO, PSTR("Calling Command..."));
      result = DecodeCommand(MyProjectCommands, MyProjectCommand);
      break;

    case FUNC_ACTIVE:
      result = true;
      break;

    case FUNC_JSON_APPEND:
      XDRV_102_show(1);
      break;

#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      XDRV_102_show(0);
      break;
#endif // USE_WEBSERVER
    }
  }

  return result;
}

#endif // USE_CYTRV_2
#endif
