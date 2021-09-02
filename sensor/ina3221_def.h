/*
 * ina3221_def.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef INA3221_DEF_H_
#define INA3221_DEF_H_
#include "driver_comm.h"

#define INA3221_ADDR    0x40
#define INA3221_MID_REG 0xFE
#define INA3221_CFG_REG 0x00

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define INA3221_ADDRESS (0x40)  // 1000000 (A0+A1=GND)
#define INA3221_READ    (0x01)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
#define INA3221_REG_CONFIG (0x00)
/*---------------------------------------------------------------------*/
#define INA3221_CONFIG_RESET (0x8000)  // Reset Bit

#define INA3221_CONFIG_ENABLE_CHAN1 (0x4000)  // Enable Channel 1
#define INA3221_CONFIG_ENABLE_CHAN2 (0x2000)  // Enable Channel 2
#define INA3221_CONFIG_ENABLE_CHAN3 (0x1000)  // Enable Channel 3

#define INA3221_CONFIG_AVG2 (0x0800)  // AVG Samples Bit 2 - See table 3 spec
#define INA3221_CONFIG_AVG1 (0x0400)  // AVG Samples Bit 1 - See table 3 spec
#define INA3221_CONFIG_AVG0 (0x0200)  // AVG Samples Bit 0 - See table 3 spec

#define INA3221_CONFIG_VBUS_CT2 (0x0100)  // VBUS bit 2 Conversion time - See table 4 spec
#define INA3221_CONFIG_VBUS_CT1 (0x0080)  // VBUS bit 1 Conversion time - See table 4 spec
#define INA3221_CONFIG_VBUS_CT0 (0x0040)  // VBUS bit 0 Conversion time - See table 4 spec

#define INA3221_CONFIG_VSH_CT2 (0x0020)  // Vshunt bit 2 Conversion time - See table 5 spec
#define INA3221_CONFIG_VSH_CT1 (0x0010)  // Vshunt bit 1 Conversion time - See table 5 spec
#define INA3221_CONFIG_VSH_CT0 (0x0008)  // Vshunt bit 0 Conversion time - See table 5 spec

#define INA3221_CONFIG_MODE_2 (0x0004)  // Operating Mode bit 2 - See table 6 spec
#define INA3221_CONFIG_MODE_1 (0x0002)  // Operating Mode bit 1 - See table 6 spec
#define INA3221_CONFIG_MODE_0 (0x0001)  // Operating Mode bit 0 - See table 6 spec

/*=========================================================================*/

/*=========================================================================
    SHUNT VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA3221_REG_SHUNTVOLTAGE_1 (0x01)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA3221_REG_BUSVOLTAGE_1 (0x02)
/*=========================================================================*/

#define SHUNT_RESISTOR_VALUE (0.1)  // default shunt resistor value of 0.1 Ohm


#endif /* INA3221_DEF_H_ */
