/*
 * Copyright (c) 2019 MS-RTOS Team.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_xx24xx.c xx24xx EEPROM device driver.
 *
 * Author: Jiao.jinxing <jiaojixing@acoinfo.com>
 *
 */

#ifndef MS_DRV_XX24XX_H
#define MS_DRV_XX24XX_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /*
     * Microchip geometries
     */
    EEPROM_24XX00,
    EEPROM_24XX01,
    EEPROM_24XX02,
    EEPROM_24XX04,
    EEPROM_24XX08,
    EEPROM_24XX16,
    EEPROM_24XX32,
    EEPROM_24XX64,
    EEPROM_24XX128,
    EEPROM_24XX256,
    EEPROM_24XX512,
    EEPROM_24XX1025,
    EEPROM_24XX1026,
    EEPROM_24CM02,

    /*
     * Atmel geometries - none...
     */

    /*
     * STM geometries
     */
    EEPROM_M24C01,
    EEPROM_M24C02,
    EEPROM_M24M02,

    /*
     * Aliases (devices similar to previously defined ones)
     */
    EEPROM_AT24C01   = EEPROM_24XX01,
    EEPROM_AT24C02   = EEPROM_24XX02,
    EEPROM_AT24C04   = EEPROM_24XX04,
    EEPROM_AT24C08   = EEPROM_24XX08,
    EEPROM_AT24C16   = EEPROM_24XX16,
    EEPROM_AT24C32   = EEPROM_24XX32,
    EEPROM_AT24C64   = EEPROM_24XX64,
    EEPROM_AT24C128  = EEPROM_24XX128,
    EEPROM_AT24C256  = EEPROM_24XX256,
    EEPROM_AT24C512  = EEPROM_24XX512,
    EEPROM_AT24C1024 = EEPROM_24XX1026,
    EEPROM_AT24CM02  = EEPROM_24CM02,

    EEPROM_M24C04    = EEPROM_24XX04,
    EEPROM_M24C08    = EEPROM_24XX08,
    EEPROM_M24C16    = EEPROM_24XX16,
    EEPROM_M24C32    = EEPROM_24XX32,
    EEPROM_M24C64    = EEPROM_24XX64,
    EEPROM_M24128    = EEPROM_24XX128,
    EEPROM_M24256    = EEPROM_24XX256,
    EEPROM_M24512    = EEPROM_24XX512,
    EEPROM_M24M01    = EEPROM_24XX1026,
} ms_xx24xx_type_t;

ms_err_t ms_xx24xx_drv_register(void);
ms_err_t ms_xx24xx_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr, ms_xx24xx_type_t dev_type);

#ifdef __cplusplus
}
#endif

#endif /* MS_DRV_XX24XX_H */
