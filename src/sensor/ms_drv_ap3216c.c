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

#define __MS_IO
#include "ms_kern.h"
#include "ms_io_core.h"
#include "ms_driver.h"
#include <string.h>

#include "ms_drv_ap3216c.h"

/**
 * @brief AP3216C device driver.
 */

#if MS_CFG_IO_MODULE_EN > 0

#define MS_AP3216C_DRV_NAME     "ap3216c_sensor"

/*
 * Private info
 */
typedef struct {
    ms_i2c_device_t i2c_dev;
    ms_uint16_t     ir;
    ms_uint16_t     ps;
    ms_uint16_t     als;
} privinfo_t;

/*
 * AP3216C device
 */
typedef struct {
    privinfo_t      priv;
    ms_io_device_t  dev;
} ms_ap3216c_dev_t;

/*
 * Read AP3216C register.
 */
static inline ms_err_t __ap3216c_read_reg(privinfo_t *priv, ms_uint8_t reg, ms_uint8_t *data)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;

    return ms_i2c_device_writeread(i2c_dev, &reg, 1, 0, data, 1, 0);
}

/*
 * Write AP3216C register.
 */
static inline ms_err_t __ap3216c_write_reg(privinfo_t *priv, ms_uint8_t reg, ms_uint8_t data)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;
    ms_uint8_t       tx_data[2];

    tx_data[0] = reg;
    tx_data[1] = data;

    return ms_i2c_device_write(i2c_dev, tx_data, 2, 0);
}

/*
 * Initialize AP3216C.
 */
static ms_err_t __ap3216c_init(privinfo_t *priv)
{
    ms_uint8_t  temp = 0;
    ms_err_t    err;

    /*
     * Reset AP3216C
     */
    __ap3216c_write_reg(priv, 0x00, 0x04);
    ms_thread_sleep_ms(100);

    /*
     * Enable ALS, PS+IR
     */
    __ap3216c_write_reg(priv, 0x00, 0x03);
    ms_thread_sleep_ms(100);

    /*
     * AP3216C test
     */
    err = __ap3216c_read_reg(priv, 0x00, &temp);
    if ((err != MS_ERR_NONE) || (temp != 0x03)) {
        err = MS_ERR;
    }

    return err;
}

/*
 * Fetch AP3216C data.
 */
static ms_err_t __ap3216c_fetch_data(privinfo_t *priv)
{
    ms_uint8_t   buf[6];
    ms_uint8_t   i;
    ms_uint16_t  ir;
    ms_uint16_t  ps;
    ms_uint16_t  als;
    ms_err_t     err;

    /*
     * Delay for continuous reading
     */
    ms_thread_sleep_ms(120);

    /*
     * Read all sensor data
     */
    for (i = 0; i < 6; i++) {
        err = __ap3216c_read_reg(priv, 0x0A + i, &buf[i]);
        if (err != MS_ERR_NONE) {
            return err;
        }
    }

    /*
     * Check the data is valid
     */
    if (buf[0] & 0x80) {
        ir = 0;
    } else {
        /*
         * Read the valid IR data
         */
        ir = ((ms_uint16_t)buf[1] << 2) | (buf[0] & 0x03);
    }

    /*
     * Read ALS sensor data
     */
    als = ((ms_uint16_t)buf[3] << 8) | buf[2];

    /*
     * Check the data is valid
     */
    if (buf[4] & 0x40) {
        ps = 0;
    } else {
        /*
         * Read the valid PS data
         */
        ps = ((ms_uint16_t)(buf[5] & 0x3F) << 4) | (buf[4] & 0x0F);
    }

    priv->als = als;
    priv->ir  = ir;
    priv->ps  = ps;

    return MS_ERR_NONE;
}

/*
 * Open device
 */
static int __ap3216c_open(ms_ptr_t ctx, ms_io_file_t *file, int oflag, ms_mode_t mode)
{
    privinfo_t *priv = ctx;
    int         ret;

    if (ms_atomic_inc(MS_IO_DEV_REF(file)) == 2) {
        if (__ap3216c_init(priv) == MS_ERR_NONE) {
            ret = 0;
        } else {
            ms_atomic_dec(MS_IO_DEV_REF(file));
            ms_thread_set_errno(EIO);
            ret = -1;
        }
    } else {
        ms_atomic_dec(MS_IO_DEV_REF(file));
        ms_thread_set_errno(EBUSY);
        ret = -1;
    }

    return ret;
}

/*
 * Close device
 */
static int __ap3216c_close(ms_ptr_t ctx, ms_io_file_t *file)
{
    ms_atomic_dec(MS_IO_DEV_REF(file));

    return 0;
}

/*
 * Read device
 */
static ms_ssize_t __ap3216c_read(ms_ptr_t ctx, ms_io_file_t *file, ms_ptr_t buf, ms_size_t len)
{
    privinfo_t *priv = ctx;
    ms_uint8_t  data_count;
    ms_ssize_t  ret;

    data_count = len / sizeof(ms_sensor_data_t);
    if ((data_count >= 1) && (len % sizeof(ms_sensor_data_t) == 0)) {
        if (__ap3216c_fetch_data(priv) == MS_ERR_NONE) {
            ms_sensor_data_t *sensor_data = (ms_sensor_data_t *)buf;
            int i;

            for (i = 0; i < data_count; i++) {
                if (MS_SENSOR_TYPE_IR == sensor_data->type) {
                    sensor_data->u.ir = priv->ir;

                } else if (MS_SENSOR_TYPE_LIGHT == sensor_data->type) {
                    sensor_data->u.light = priv->als;

                } else if (MS_SENSOR_TYPE_PROXIMITY == sensor_data->type) {
                    sensor_data->u.proximity = priv->ps;

                } else {
                    ms_thread_set_errno(EINVAL);
                    break;
                }

                sensor_data++;
            }
            ret = i * sizeof(ms_sensor_data_t);

        } else {
            ms_thread_set_errno(EIO);
            ret = -1;
        }
    } else {
        ms_thread_set_errno(EINVAL);
        ret = -1;
    }

    return ret;
}

/*
 * Device operating function set
 */
static ms_io_driver_ops_t ms_ap3216c_drv_ops = {
        .type   = MS_IO_DRV_TYPE_CHR,
        .open   = __ap3216c_open,
        .close  = __ap3216c_close,
        .read   = __ap3216c_read,
};

/*
 * Device driver
 */
static ms_io_driver_t ms_ap3216c_drv = {
        .nnode = {
            .name = MS_AP3216C_DRV_NAME,
        },
        .ops = &ms_ap3216c_drv_ops,
};

/*
 * Register AP3216C device driver
 */
ms_err_t ms_ap3216c_drv_register(void)
{
    return ms_io_driver_register(&ms_ap3216c_drv);
}

/*
 * Create AP3216C device file
 */
ms_err_t ms_ap3216c_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr)
{
    ms_ap3216c_dev_t *dev;
    ms_err_t err;

    if ((path != MS_NULL) && (i2c_bus_name != MS_NULL)) {

        dev = ms_kmalloc(sizeof(ms_ap3216c_dev_t));
        if (dev != MS_NULL) {
            privinfo_t *priv = &dev->priv;

            /*
             * Attach i2c device to i2c bus
             */
            priv->i2c_dev.nnode.name = MS_AP3216C_DRV_NAME;
            priv->i2c_dev.clk_speed  = MS_I2C_CLK_SPEED_STANDARD;
            priv->i2c_dev.addr       = dev_addr;
            priv->i2c_dev.addrlen    = 7U;
            priv->i2c_dev.ctx        = MS_NULL;

            err = ms_i2c_device_attach(&priv->i2c_dev, i2c_bus_name);
            if (err == MS_ERR_NONE) {
                err = ms_io_device_register(&dev->dev, path, MS_AP3216C_DRV_NAME, &dev->priv);

                if (err != MS_ERR_NONE) {
                    ms_i2c_device_detach(&priv->i2c_dev, i2c_bus_name);
                }
            }

            if (err != MS_ERR_NONE) {
                (void)ms_kfree(dev);
            }

        } else {
            err = MS_ERR_KERN_HEAP_NO_MEM;
        }
    } else {
        err = MS_ERR_ARG_NULL_PTR;
    }

    return err;
}

#endif
