/*
  xdrv_101_cytrv_.ino - My IoT device support for Tasmota
*/

#ifdef USE_I2C

#ifdef USE_CYTRV

/*********************************************************************************************\
 * TRV driver
 *
 *
\*********************************************************************************************/

#warning **** xdrv cyTRV_101 is included... ****

#define XDRV_101 101

#ifdef USE_CYTRV_1
#ifdef USE_CYTRV_2
#error **** cyTRV: selct only one version! ****
#endif
#define xdrv_101_ina219
#endif

#ifdef USE_CYTRV_2
#define xdrv_101_ina219
#endif

#ifdef xdrv_101_ina219
#ifdef USE_INA219
#undef USE_INA219
#warning **** INA219 from tasmota was deactivated... ****
#endif
#endif

#ifdef USE_CYSHUTTER_1

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

#ifdef xdrv_101_ina219
//*********************************************************************************************/
// INA219 decl ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/
#include <Wire.h>
#include "INA219_WE.h"

#define XDRV_101_I2C_ADDRESS 0x40

/* There are several ways to create your INA219 object:
   INA219_WE ina219 = INA219_WE(); -> uses Wire / I2C Address = 0x40
   INA219_WE ina219 = INA219_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
   INA219_WE ina219 = INA219_WE(&Wire); -> you can pass any TwoWire object
   INA219_WE ina219 = INA219_WE(&Wire, I2C_ADDRESS); -> all together
*/
INA219_WE ina219 = INA219_WE(XDRV_101_I2C_ADDRESS);
const char *XDRV_101_INA219_TYPE[] = {"INA219", "ISL28022"};

struct XDRV_101_INA219
{
  bool active;
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0;
  bool overflow = false;
#ifdef USE_CYTRV_1
  float max_curr = 40;
#endif // USE_CYTRV_1
#ifdef USE_CYTRV_2
  float max_curr = 50;
#endif // USE_CYTRV_2
} XDRV_101_ina219;

#ifdef USE_WEBSERVER
const char XDRV_101_HTTP_SNS_INA219_DATA[] PROGMEM =
    "{s}%s " D_VOLTAGE "{m}%s " D_UNIT_VOLT "{e}"
    "{s}%s " D_CURRENT "{m}%s " D_UNIT_AMPERE "{e}"
    "{s}%s " D_POWERUSAGE "{m}%s " D_UNIT_WATT "{e}";
#endif // USE_WEBSERVER

#endif // xdrv_101_ina219

//*********************************************************************************************/
// MQTT decl  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

struct XDRV_101_MQTT
{
  boolean log_mqtt;
  boolean pub_sens;
  int dest_pos;
  boolean cal;
} XDRV_101_mqtt;

//*********************************************************************************************/
// State decl ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

struct XDRV_101_STATE
{
  unsigned long State_millis;

  unsigned long dest_millis;
  unsigned long old_millis;

  int state;
  int state_old;
  int max_time;
  int pos_time;

} XDRV_101_state;

const char cyTRVStates[] PROGMEM = "do stop" "stopped" ;

//*********************************************************************************************/
// Motor decl ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

#ifdef USE_CYTRV_1

#include "WEMOS_Motor.h"
Motor *XDRV_101_Ventil;

#else
#define _CCW 1
#define _CW 2
#define _STOP 3
#define _STANDBY 4
#endif // USE_CYTRV_1

struct XDRV_101_MOTOR
{
  int pwm;

  boolean init = false;
  boolean run = false;
  boolean dir = false;
  boolean starting = false;
  boolean started = false;

  int act_pos;
  int dest_pos;
  int old_pos;

} XDRV_101_motor;

#ifdef USE_WEBSERVER
const char XDRV_101_HTTP_SNS_TRV_DATA[] PROGMEM =
    "{s}%s  Position {m}%s " D_UNIT_PERCENT " {e}"
    "{s}%s  State {m}%s  {e}"
    "{s}%s  max_time {m}%s " D_UNIT_SECOND " {e}";
#endif // USE_WEBSERVER

#ifdef xdrv_101_ina219
/*********************************************************************************************/
// INA219 part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

bool Xdrv_101_init_ina219()
{
  XDRV_101_ina219.active = false;

  if (!ina219.init())
  {
    return XDRV_101_ina219.active;
  }

  I2cSetActiveFound(XDRV_101_I2C_ADDRESS, XDRV_101_INA219_TYPE[0]);

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
  XDRV_101_ina219.active = true;
  return XDRV_101_ina219.active;
}

void Xdrv_101_check_ina219()
{

  if (XDRV_101_ina219.active == false)
  {
    return;
  }

  XDRV_101_ina219.shuntVoltage_mV = ina219.getShuntVoltage_mV();
  XDRV_101_ina219.busVoltage_V = ina219.getBusVoltage_V();
  XDRV_101_ina219.current_mA = ina219.getCurrent_mA();
  XDRV_101_ina219.power_mW = ina219.getBusPower();
  XDRV_101_ina219.loadVoltage_V = XDRV_101_ina219.busVoltage_V + (XDRV_101_ina219.shuntVoltage_mV / 1000);
  XDRV_101_ina219.overflow = ina219.getOverflow();
}

void XDRV_101_show_INA219(bool json)
{
  if (!XDRV_101_ina219.active)
  {
    return;
  }

  char voltage[16];
  dtostrfd(XDRV_101_ina219.busVoltage_V, Settings->flag2.voltage_resolution, voltage);
  char current[16];
  dtostrfd(XDRV_101_ina219.current_mA / 1000, Settings->flag2.current_resolution, current);
  char power[16];
  dtostrfd(XDRV_101_ina219.power_mW / 1000, Settings->flag2.wattage_resolution, power);
  char name[16];
  snprintf_P(name, sizeof(name), PSTR("%s"), XDRV_101_INA219_TYPE[0]);

  if (json)
  {
    ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_VOLTAGE "\":%s,\"" D_JSON_CURRENT "\":%s,\"" D_JSON_POWERUSAGE "\":%s}"),
                     name, XDRV_101_I2C_ADDRESS, voltage, current, power);

#ifdef USE_WEBSERVER
  }
  else
  {
    WSContentSend_PD(XDRV_101_HTTP_SNS_INA219_DATA, name, voltage, name, current, name, power);
#endif // USE_WEBSERVER
  }
}

#endif // xdrv_101_ina219

//*********************************************************************************************/
// MQTT part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/
void XDRV_101_show_TRV(bool json)
{
  char position[16];
  if (XDRV_101_motor.init)
  {
    dtostrfd(XDRV_101_motor.act_pos, 0, position);
  }
  else
  {
    snprintf_P(position, sizeof(position), PSTR("%s"), "Define Relay Pins!");
  }

  char state[16];
  dtostrfd(XDRV_101_state.state, 0, state);

  char max_time[16];
  dtostrfd(XDRV_101_state.max_time, 0, max_time);

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
    WSContentSend_PD(XDRV_101_HTTP_SNS_TRV_DATA, name, position, name, state, name, max_time);
#endif // USE_WEBSERVER
  }
}

//*********************************************************************************************/
// Motor part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/
void XDRV_101_motor_start(uint8_t dir, float pwm_val)
{

  // read INA219 to get actual current before start of motor
  Xdrv_101_check_ina219();
  char current[16];
  dtostrfd(XDRV_101_ina219.current_mA, Settings->flag2.current_resolution, current);
  AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_LOG "Motor Start - current: %s"), current);

#ifdef USE_CYTRV_1
  XDRV_101_Ventil->setmotor(dir, pwm_val);
#endif // USE_CYTRV_1

#ifdef USE_CYTRV_2
  // void ExecuteCommandPower(uint32_t device, uint32_t state, uint32_t source)
  //  device  = Relay number 1 and up
  //  state 0 = POWER_OFF = Relay Off
  //  state 1 = POWER_ON = Relay On (turn off after Settings->pulse_timer * 100 mSec if enabled)
  //  state 2 = POWER_TOGGLE = Toggle relay
  //  state 3 = POWER_BLINK = Blink relay
  //  state 4 = POWER_BLINK_STOP = Stop blinking relay
  //  state 5 = POWER_OFF_FORCE = Relay off even if locked
  //  state 8 = POWER_OFF_NO_STATE = Relay Off and no publishPowerState
  //  state 9 = POWER_ON_NO_STATE = Relay On and no publishPowerState
  //  state 10 = POWER_TOGGLE_NO_STATE = Toggle relay and no publishPowerState
  //  state 16 = POWER_SHOW_STATE = Show power state

  //  ShowSource(source);
  if (dir == _CW)
  {
    ExecuteCommandPower(1, POWER_ON, SRC_IGNORE);
    ExecuteCommandPower(2, POWER_OFF, SRC_IGNORE);
  }
  else
  {
    ExecuteCommandPower(1, POWER_OFF, SRC_IGNORE);
    ExecuteCommandPower(2, POWER_ON, SRC_IGNORE);
  }
  // TasmotaGlobal.last_source = SRC_SHUTTER;

#endif // USE_CYTRV_2

  if (dir == _CW)
  {
    XDRV_101_motor.dir = true;
  }
  else
  {
    XDRV_101_motor.dir = false;
  }
  XDRV_101_motor.run = true;
  XDRV_101_motor.starting = false;
  XDRV_101_motor.started = false;
}

void Xdrv_101_motor_startup()
{
  if (!XDRV_101_motor.run)
  {
    return;
  }

  if (XDRV_101_motor.started == true)
  {
    return;
  }

  if (XDRV_101_motor.starting == false)
  {
    XDRV_101_motor.starting = true;
    return;
  }

  if (XDRV_101_motor.started == false)
  {
    XDRV_101_motor.started = true;
    return;
  }
}

void XDRV_101_motor_stop()
{
#ifdef USE_CYTRV_1
  XDRV_101_Ventil->setmotor(_STOP);
#endif // USE_CYTRV_1

#ifdef USE_CYTRV_2
  ExecuteCommandPower(1, POWER_OFF, SRC_IGNORE);
  ExecuteCommandPower(2, POWER_OFF, SRC_IGNORE);
#endif // USE_CYTRV_2

  XDRV_101_motor.run = false;
  XDRV_101_motor.started = false;
}

void XDRV_101_motor_stop_all()
{
  XDRV_101_motor_stop();
}
void XDRV_101_init_motor()
{
#ifdef USE_CYTRV_1
  // Motor shield I2C Address: 0x30
  // PWM frequency: 1000Hz(1kHz)
  // Motor Pumpe(0x30, _MOTOR_A, 1000); //Motor A
  XDRV_101_Ventil = new Motor(0x30, _MOTOR_B, 1000); // Motor B
  XDRV_101_motor.init = true;
#endif // USE_CYTRV_1

#ifdef USE_CYTRV_2
  // relais assigned
  if (PinUsed(GPIO_REL1, 0) && PinUsed(GPIO_REL1, 1))
  {
    XDRV_101_motor.init = true;
  }
#endif // USE_CYTRV_2

  XDRV_101_motor_stop_all();
}

//*********************************************************************************************/
// State part ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//*********************************************************************************************/

void Xdrv_101_check_state(void)
{

  XDRV_101_state.state_old = XDRV_101_state.state;

  //  if ((XDRV_101_state.state != 1) && XDRV_101_motor.started == true)
  if ((XDRV_101_motor.run == true) && (XDRV_101_motor.started == true))
  {
    Xdrv_101_check_ina219();
  }

  switch (XDRV_101_state.state)
  {
  case 0: // do stop
    Xdrv_101_state_motor_stop();
    break;
  case 1: // stopped
    if (XDRV_101_state.state == 1)
    {
      if (XDRV_101_mqtt.dest_pos != -1)
      {
        XDRV_101_motor.dest_pos = XDRV_101_mqtt.dest_pos;
        XDRV_101_state.state = 6;
        XDRV_101_mqtt.dest_pos = -1;
      }
    }

    if (XDRV_101_state.state == 1)
    {
      if (XDRV_101_mqtt.cal == true)
      {
        XDRV_101_state.state = 2; // calibration
        XDRV_101_mqtt.cal = false;
      }
    }
    break;
  case 2: // do calibration
    XDRV_101_motor.act_pos = -1;
    XDRV_101_motor_start(_CW, 100);
    XDRV_101_state.state = 3;
  case 3: // calibration opening
    if (XDRV_101_ina219.current_mA > XDRV_101_ina219.max_curr)
    {
      XDRV_101_motor_stop();
      XDRV_101_state.state = 4;
    }
    break;
  case 4: // do calibration close
    XDRV_101_motor_start(_CCW, 100);
    XDRV_101_state.max_time = 0;
    XDRV_101_state.state = 5;

    break;

  case 5: // calibration closing
    if (XDRV_101_ina219.current_mA > XDRV_101_ina219.max_curr)
    {
      Xdrv_101_state_motor_stop();
      XDRV_101_motor.act_pos = 0;
    }
    break;
  case 6: // do positioning
    if (XDRV_101_motor.act_pos == XDRV_101_motor.dest_pos)
    {
      XDRV_101_state.state = 1;
      break;
    }

    XDRV_101_motor.old_pos = XDRV_101_motor.act_pos;

    if (XDRV_101_motor.act_pos > XDRV_101_motor.dest_pos)
    {
      // millis für neue Position errechnen
      int lv_diff_pos = XDRV_101_motor.act_pos - XDRV_101_motor.dest_pos;
      int lv_diff_millis = ((XDRV_101_state.max_time * 1000) / 100) * lv_diff_pos;
      XDRV_101_state.old_millis = millis();
      XDRV_101_state.dest_millis = XDRV_101_state.old_millis + lv_diff_millis;

      XDRV_101_motor_start(_CCW, 100);
      XDRV_101_state.state = 7;
    }
    else
    {
      // millis für neue Position errechnen
      int lv_diff_pos = XDRV_101_motor.dest_pos - XDRV_101_motor.act_pos;
      int lv_diff_millis = ((XDRV_101_state.max_time * 1000) / 100) * lv_diff_pos;
      XDRV_101_state.old_millis = millis();
      XDRV_101_state.dest_millis = XDRV_101_state.old_millis + lv_diff_millis;

      XDRV_101_motor_start(_CW, 100);
      XDRV_101_state.state = 8;
    }
    // gv_pos_time = gv_max_time - map(gv_dest_pos, 0, 100, gv_max_time, 0);
    break;
  case 7: // go to position closing
    if (XDRV_101_state_check_overcurr())
    {
      XDRV_101_motor.act_pos = 0;
      break;
    };

    // aktuelle Position berechnen
    XDRV_101_motor.act_pos = XDRV_101_motor.old_pos - ((100 * (millis() - XDRV_101_state.old_millis)) / (XDRV_101_state.max_time * 1000));

    if (XDRV_101_motor.dest_pos != 0)
    {
      if (XDRV_101_motor.act_pos <= XDRV_101_motor.dest_pos)
      {
        Xdrv_101_state_motor_stop();
        break;
      }
    }
    break;
  case 8: // go to position opening
    if (XDRV_101_state_check_overcurr())
    {
      XDRV_101_motor.act_pos = 100;
      break;
    };

    // aktuelle Position berechnen
    XDRV_101_motor.act_pos = XDRV_101_motor.old_pos + ((100 * (millis() - XDRV_101_state.old_millis)) / (XDRV_101_state.max_time * 1000));

    if (XDRV_101_motor.dest_pos != 100)
    {
      if (XDRV_101_motor.act_pos >= XDRV_101_motor.dest_pos)
      {
        Xdrv_101_state_motor_stop();
        break;
      }
    }
    break;
  }

  if (XDRV_101_state.state_old != XDRV_101_state.state)
  {
    XDRV_101_mqtt.pub_sens = true;

    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_LOG "State changed from %d to %d"), XDRV_101_state.state_old, XDRV_101_state.state);
  }
}

boolean XDRV_101_state_check_overcurr()
{
  if (XDRV_101_ina219.current_mA <= XDRV_101_ina219.max_curr)
  {
    return false;
  }

  float curr = XDRV_101_ina219.current_mA;

  Xdrv_101_state_motor_stop();

  // AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_LOG "overcurrent (%.0f) -> stop"), curr);
  // AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_LOG "overcurrent -> stop %f"), curr);
  AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_LOG "overcurrent -> stop"));

  return true;
}

void Xdrv_101_check_state_1s()
{

  switch (XDRV_101_state.state)
  {
  case 5: // calibration closing
    XDRV_101_state.max_time = XDRV_101_state.max_time + 1;
    break;
  case 7:
    // gv_act_pos = gv_act_pos - ( 100 / gv_max_time );

    break;
  case 8:
    // gv_act_pos = gv_act_pos + ( 100 / gv_max_time );

    break;
  }
}

void Xdrv_101_state_motor_stop()
{
  XDRV_101_motor_stop();
  XDRV_101_state.state = 1;
  XDRV_101_mqtt.pub_sens = true;
  Xdrv_101_check_ina219();
}

void Xdrv_101_init_state()
{
  XDRV_101_state.state = 0;
#ifdef USE_CYTRV_1
  XDRV_101_state.max_time = 46;
#endif
#ifdef USE_CYTRV_2
  XDRV_101_state.max_time = 68;
#endif
  // set_state_tact();
}

/*********************************************************************************************\
 * My IoT Device Functions
\*********************************************************************************************/

// This variable will be set to true after initialization
bool XDRV_101_initSuccess = false;

/*
  Optional: if you need to pass any command for your device
  Commands are issued in Console or Web Console
  Commands:
    Say_Hello  - Only prints some text. Can be made something more useful...
    SendMQTT   - Send a MQTT example message
    Help       - Prints a list of available commands
*/

const char cyTRVCommands[] PROGMEM = D_cyTRV "|" // Prefix
                                             "Cal|"
                                             "Pos|"
                                             //  "Say_Hello|"
                                             //   "SendMQTT|"
                                             "HELP";

void (*const cyTRVCommand[])(void) PROGMEM = {
    &CmdTRVCal,
    &CmdTRVPos,
    // &CmdSay_Hello,
    // &CmdSendMQTT,
    &CmdHelp};

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

void CmdHelp(void)
{
  AddLog(LOG_LEVEL_INFO, PSTR("cyTRV Help: Accepted commands - cyTRVCal, cyTRVPos, cyTRVHelp"));
  ResponseCmndDone();
}

// Command Calibration
void CmdTRVCal(void)
{
  // AddLog(LOG_LEVEL_INFO, PSTR("Calling Xdrv_101 Command calibrate..."));
  XDRV_101_mqtt.cal = true;
  ResponseCmndDone();
}

// Command Position
void CmdTRVPos()
{
  AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_LOG "Calling Xdrv_101 Command position ... %d"), XdrvMailbox.payload);
  if (XdrvMailbox.payload >= 0)
  {
    XDRV_101_mqtt.dest_pos = XdrvMailbox.payload;
  }
  else
  {
    MqttPublishSensor();
  }

  ResponseCmndDone();
}

/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/

void XDRV_101_Init()
{
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_cyTRV " init..."));

  // Serial.begin(115200);
  XDRV_101_init_motor();

  // init_window();

  Xdrv_101_init_ina219();

  Xdrv_101_init_state();

  XDRV_101_motor.dest_pos = 0;
  XDRV_101_state.state = 6;

  // Set XDRV_101_initSuccess at the very end of the init process
  // Init is successful
  XDRV_101_initSuccess = true;
}

void Xdrv_101_check_1s(void)
{
  Xdrv_101_check_ina219();
  Xdrv_101_check_state_1s();
  if (XDRV_101_mqtt.pub_sens)
  {
    MqttPublishSensor();
    XDRV_101_mqtt.pub_sens = false;
  }
}

void XDRV_101_show(bool json)
{
  XDRV_101_show_INA219(json);
  WSContentSeparator(0);
  XDRV_101_show_TRV(json);
  XDRV_101_mqtt.pub_sens = false;
}
/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdrv101(uint32_t function)
{

  bool result = false;

  if (function == FUNC_INIT)
  {
    XDRV_101_Init();
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_cyTRV " init is done..."));
  }
  else if (XDRV_101_initSuccess)
  {

    switch (function)
    {
      // Select suitable interval for polling your function
    case FUNC_EVERY_SECOND:
      Xdrv_101_check_1s();

      //    case FUNC_EVERY_250_MSECOND:

    case FUNC_EVERY_200_MSECOND:
      Xdrv_101_motor_startup();
      break;
      // case FUNC_EVERY_100_MSECOND:

    case FUNC_EVERY_50_MSECOND:
      Xdrv_101_check_state();
      break;

    // Command support
    case FUNC_COMMAND:
      // AddLog(LOG_LEVEL_INFO, PSTR("Calling Command..."));
      result = DecodeCommand(cyTRVCommands, cyTRVCommand);
      break;

    case FUNC_ACTIVE:
      result = true;
      break;

    case FUNC_SET_POWER:
      AddLog(LOG_LEVEL_INFO, PSTR("Calling FUNC_SET_POWER ..."));
      break;

    case FUNC_JSON_APPEND:
      XDRV_101_show(1);
      break;

#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      XDRV_101_show(0);
      break;
#endif // USE_WEBSERVER
    }
  }

  return result;
}

#endif // USE_CYTRV_1
#endif
