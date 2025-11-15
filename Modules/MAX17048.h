#include "main.h"

#ifndef MAX17048_H_
#define MAX17048_H_


#define MAX17048_I2CADDR_DEFAULT (0x36<<1) ///< MAX17048 default i2c address

#define MAX1704X_VCELL_REG 0x02   ///< Register that holds cell voltage
#define MAX1704X_SOC_REG 0x04     ///< Register that holds cell state of charge
#define MAX1704X_MODE_REG 0x06    ///< Register that manages mode
#define MAX1704X_VERSION_REG 0x08 ///< Register that has IC version
#define MAX1704X_HIBRT_REG 0x0A   ///< Register that manages hibernation
#define MAX1704X_CONFIG_REG 0x0C  ///< Register that manages configuration
#define MAX1704X_VALERT_REG 0x14  ///< Register that holds voltage alert values
#define MAX1704X_CRATE_REG 0x16   ///< Register that holds cell charge rate
#define MAX1704X_VRESET_REG 0x18  ///< Register that holds reset voltage setting
#define MAX1704X_CHIPID_REG 0x19  ///< Register that holds semi-unique chip ID
#define MAX1704X_STATUS_REG 0x1A  ///< Register that holds current alert/status
#define MAX1704X_CMD_REG 0xFE ///< Register that can be written for special commands

#define MAX1704X_ALERTFLAG_SOC_CHANGE                                          \
  0x20 ///< Alert flag for state-of-charge change
#define MAX1704X_ALERTFLAG_SOC_LOW 0x10 ///< Alert flag for state-of-charge low
#define MAX1704X_ALERTFLAG_VOLTAGE_RESET                                       \
  0x08 ///< Alert flag for voltage reset dip
#define MAX1704X_ALERTFLAG_VOLTAGE_LOW 0x04 ///< Alert flag for cell voltage low
#define MAX1704X_ALERTFLAG_VOLTAGE_HIGH                                        \
  0x02 ///< Alert flag for cell voltage high
#define MAX1704X_ALERTFLAG_RESET_INDICATOR                                     \
  0x01 ///< Alert flag for IC reset notification


uint8_t MAX17048_connection(I2C_HandleTypeDef* hi2c);
float_t MAX17048_getVoltage(I2C_HandleTypeDef* hi2c);
float_t MAX17048_getPercentage(I2C_HandleTypeDef* hi2c);
float_t MAX17048_getDisChargeRate(I2C_HandleTypeDef* hi2c);

#endif
