/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_ap3216c.c AP3216C device driver.
 *
 * Author: Yu.Kangzhi <yukangzhi@acoinfo.com>
 *
 */

#ifndef MS_DRV_AP3216C_H
#define MS_DRV_AP3216C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ms_drv_sensor.h"

ms_err_t ms_ap3216c_drv_register(void);
ms_err_t ms_ap3216c_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr);

#ifdef __cplusplus
}
#endif

#endif /* MS_DRV_AP3216C_H */
