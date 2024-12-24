// Conditional compilation of driver
#ifdef USE_I2C
#ifdef USE_CYTRV_1

#warning **** xsns cyTRV_130 is included... ****
 
// Define driver ID
#define XSNS_130  130

/**
 * The callback function Xsns<driver_ID>() interfaces Tasmota with the sensor driver.
 *
 * It provides the Tasmota callback IDs.
 *
 * @param   byte    callback_id  Tasmota function ID.
 * @return  boolean              Return value.
 * @pre     None.
 * @post    None.
 *
 */
boolean Xsns130(byte callback_id) {

  // Set return value to `false`
  boolean result = false;

  // Check if I2C interface mode
// if(i2c_flg) {

  // Check which callback ID is called by Tasmota
  switch (callback_id) {
    case FUNC_INIT:
      break;
    case FUNC_EVERY_50_MSECOND:
      break;
    case FUNC_EVERY_SECOND:
      break;
    case FUNC_JSON_APPEND:
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      break;
#endif // USE_WEBSERVER
    case FUNC_SAVE_BEFORE_RESTART:
      break;
    case FUNC_COMMAND:
      break;
  }
// } // if(i2c_flg)

  // Return boolean result
  return result;
}


#endif
#endif 