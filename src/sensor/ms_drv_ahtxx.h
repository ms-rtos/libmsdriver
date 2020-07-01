/*
 * Copyright (c) 2020 MS-RTOS Team.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_ahtxx.c AHTXX device driver.
 *
 * Author: Yu.Kangzhi <yukangzhi@acoinfo.com>
 *
 */

#ifndef MS_DRV_AHTXX_H
#define MS_DRV_AHTXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ms_drv_sensor.h"

typedef enum {
    MS_AHTXX_TYPE_AHT10,
    MS_AHTXX_TYPE_AHT20,
    MS_AHTXX_TYPE_MAX,
} ms_ahtxx_type_t;

typedef struct {
#define MS_AHTXX_MODE_TRIG_ON_READ      (0U)
#define MS_AHTXX_MODE_TRIG_ON_TIME      MS_BIT(0U)
#define MS_AHTXX_MODE_TRIG_MODE_MASK    MS_BIT(0U)

#define MS_AHTXX_MODE_MEASURE_ONCE      (0U)
#define MS_AHTXX_MODE_MEASURE_PERIOD    MS_BIT(1U)
#define MS_AHTXX_MODE_MEASURE_MODE_MASK MS_BIT(1U)

#define MS_AHTXX_MODE_FILTER_DISABLE    (0U)
#define MS_AHTXX_MODE_FILTER_ENABLE     MS_BIT(2U)
#define MS_AHTXX_MODE_FILTER_MODE_MASK  MS_BIT(2U)

#define MS_AHTXX_MODE_DATA_IN_RANGE     (0U)
#define MS_AHTXX_MODE_DATA_OUT_RANGE    MS_BIT(3U)
#define MS_AHTXX_MODE_DATA_MODE_MASK    MS_BIT(3U)
    ms_uint16_t       mode;
    ms_tick_t         interval;

    ms_sensor_data_t *overflow_line;
    ms_uint16_t       overflow_line_nr;

    ms_sensor_data_t *underflow_line;
    ms_uint16_t       underflow_line_nr;
} ms_ahtxx_param_t;

#define MS_SENSOR_AHTXX_CMD_SOFT_RESET      _MS_IO('a', 'r')

#define MS_SENSOR_AHTXX_CMD_GET_CAP         _MS_IO('a', 'c')

#define MS_SENSOR_AHTXX_CMD_SET_PARAM       _MS_IOW('a', 'p', ms_ahtxx_param_t)
#define MS_SENSOR_AHTXX_CMD_GET_PARAM       _MS_IOR('a', 'p', ms_ahtxx_param_t)

#define MS_SENSOR_AHTXX_CMD_HANG_MEASURE    _MS_IO('a', 'h')
#define MS_SENSOR_AHTXX_CMD_RESUME_MEASURE  _MS_IO('a', 's')

ms_err_t ms_ahtxx_drv_register(void);
ms_err_t ms_ahtxx_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr, ms_ahtxx_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* MS_DRV_AHTXX_H */
