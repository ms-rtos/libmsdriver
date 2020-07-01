/*
 * Copyright (c) 2019 MS-RTOS Team.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_gt9xx.h GT9XX touch screen device driver.
 *
 * Author: Yu.kangzhi <yukangzhi@acoinfo.com>
 *
 */

#ifndef MS_DRV_GT9XX_H
#define MS_DRV_GT9XX_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum  {
    MS_GTXXX_TYPE_GT9157,
    MS_GTXXX_TYPE_GT911,
    MS_GTXXX_TYPE_GT615,
    MS_GTXXX_TYPE_MAX,
} ms_gtxxx_type_t;

typedef struct {
    ms_gtxxx_type_t   type;
    ms_uint8_t        product_id[4];
    const ms_uint8_t *cfg_tbl_buf;
    ms_uint16_t       cfg_tbl_len;
    double            inch_size;
    ms_uint16_t       abs_x_max;
    ms_uint16_t       abs_y_max;
    ms_uint8_t        int_mode;
} const ms_gtxxx_port_t;

ms_err_t ms_gt9xx_drv_register(void);
ms_err_t ms_gt9xx_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr, ms_gtxxx_port_t *port);

#ifdef __cplusplus
}
#endif

#endif /* MS_DRV_GT9XX_H */
