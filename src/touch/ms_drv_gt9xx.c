/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_gt9xx.c GT9XX touch screen device driver.
 *
 * Author: Yu.kangzhi <yukangzhi@acoinfo.com>
 *
 */

#define __MS_IO
#include "ms_kern.h"
#include "ms_io_core.h"
#include "ms_driver.h"
#include <string.h>

#include "ms_drv_gt9xx.h"

/**
 * @brief GT9XX touch screen device driver.
 */

#if MS_CFG_IO_MODULE_EN > 0

#define MS_GT9XX_DRV_NAME           "gt9xx_touch"

#define GT9XX_DEBUG_EN              (0U)

#define GT9XX_MAX_RETRY_TIMES       (5U)
#define GT9XX_MAX_TOUCH_NUM         (5U)

#define GT9XX_CONFIG_MIN_LENGTH     (186U)
#define GT9XX_CONFIG_MAX_LENGTH     (240U)

#define GT9XX_ADDR_LENGTH           (2U)
#define GT9XX_REG_CTRL              (0x8040)
#define GT9XX_REG_CFGS              (0x8047)
#define GT9XX_REG_CHECK             (0x80FF)
#define GT9XX_REG_PID               (0x8140)
#define GT9XX_REG_STATUS            (0x814E)
#define GT9XX_REG_POINTS            (0x814F)

#define GT9XX_TP_COOR_ADDR          (0x814E)

typedef struct {
    ms_uint16_t     coordinate_x;
    ms_uint16_t     coordinate_y;
} point_record_t;

/*
 * Private info
 */
typedef struct {
    ms_i2c_device_t  i2c_dev;
    ms_gtxxx_port_t *port;
    ms_uint8_t      *tp_buf;
    ms_uint8_t       tp_buf_len;
    ms_uint8_t       last_points_num;
    point_record_t   last_record[GT9XX_MAX_TOUCH_NUM];
} privinfo_t;

/*
 * GT9XX device object
 */
typedef struct {
    privinfo_t      priv;
    ms_io_device_t  dev;
} ms_gt9xx_dev_t;

/*
 * Check GT9XX porting info
 */
static ms_err_t __gt9xx_port_check(ms_gtxxx_port_t *port)
{
    if (port->type >= MS_GTXXX_TYPE_MAX) {
        return MS_ERR_ARG_INVALID;
    }

    if ((port->cfg_tbl_buf == MS_NULL) ||
        (port->cfg_tbl_len < GT9XX_CONFIG_MIN_LENGTH)) {
        return MS_ERR_ARG_INVALID;
    }

    return MS_ERR_NONE;
}

/*
 * Read GT9XX register
 */
static inline ms_err_t __gt9xx_read_reg(privinfo_t *priv, ms_uint16_t reg, ms_uint8_t *buf, ms_uint16_t len)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;
    ms_uint8_t       maddr[2U];

    maddr[0U] = (reg) >> 8U;
    maddr[1U] = (reg) &  0xFFU;

    return ms_i2c_device_writeread(i2c_dev, maddr, 2, 0, buf, len, 0);
}

/*
 * Write data to GT9XX
 */
static inline ms_err_t __gt9xx_write(privinfo_t *priv, ms_uint8_t *buf, ms_uint16_t len)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;

    return ms_i2c_device_write(i2c_dev, buf, len, 0);
}

/*
 * Check product id of GT9XX.
 */
static ms_err_t __gt9xx_pid_check(privinfo_t *priv)
{
    ms_gtxxx_port_t *port = priv->port;
    ms_uint8_t       product_id[4];
    ms_err_t         err = MS_ERR_IO;

    if (__gt9xx_read_reg(priv, GT9XX_REG_PID, product_id, sizeof(product_id)) == MS_ERR_NONE) {
        if (memcmp(port->product_id, product_id, sizeof(product_id)) == 0) {
            err = MS_ERR_NONE;
        }
    }

    return err;
}

/*
 * Reset GT9XX.
 */
static ms_err_t __gt9xx_reset(privinfo_t *priv)
{
    ms_uint8_t data[3];

    data[0] = GT9XX_REG_CTRL >> 8;
    data[1] = GT9XX_REG_CTRL & 0xFF;
    data[2] = 0x02;

    return __gt9xx_write(priv, data, 3);
}

/*
 * Start GT9XX.
 */
static ms_err_t __gt9xx_start(privinfo_t *priv)
{
    ms_uint8_t data[3];

    data[0] = GT9XX_REG_CTRL >> 8;
    data[1] = GT9XX_REG_CTRL & 0xFF;
    data[2] = 0x00;

    return __gt9xx_write(priv, data, 3);
}

/*
 * Build GT9XX configure table.
 */
static void __gt9xx_build_cfgs(privinfo_t *priv, ms_uint8_t *cfg_buf, ms_uint8_t *cfg_data_len)
{
    ms_gtxxx_port_t *port = priv->port;
    ms_uint8_t       cfg_num;
    ms_uint8_t       check_sum;
    int              i;

    /*
     * Get configure register number (Most time equals to port->cfg_tbl_len)
     */
    if (port->type == MS_GTXXX_TYPE_GT615) {
        cfg_num = 0x8128 - 0x8047 + 1;
    } else {
        cfg_num = 0x80FE - 0x8047 + 1;
    }

    /*
     * Copy configure data
     */
    memset(cfg_buf, 0, GT9XX_CONFIG_MAX_LENGTH);
    memcpy(cfg_buf, port->cfg_tbl_buf, port->cfg_tbl_len);

    /*
     * Calculate checksum
     */
    check_sum = 0;
    for (i = 0; i < cfg_num; i++) {
        check_sum += cfg_buf[i];
    }

    cfg_buf[cfg_num]     = (~check_sum) + 1;
    cfg_buf[cfg_num + 1] = 1;

    /*
     * Return real configure table length
     */
    *cfg_data_len = cfg_num + 2;
}

/*
 * Configure GT9XX.
 */
static int __gt9xx_config(privinfo_t *priv)
{
    ms_uint8_t *buf;
    int         ret;

    buf = ms_kmalloc(GT9XX_ADDR_LENGTH + GT9XX_CONFIG_MAX_LENGTH);
    if (buf != MS_NULL) {
        ms_uint8_t  cfg_len;
        ms_int32_t  retry;

        buf[0] = GT9XX_REG_CFGS >> 8;
        buf[1] = GT9XX_REG_CFGS & 0xFF;

        /*
         * Build GT9XX configure table
         */
        __gt9xx_build_cfgs(priv, &buf[GT9XX_ADDR_LENGTH], &cfg_len);

        /*
         * Write configure data to register
         */
        for (retry = 0; retry < GT9XX_MAX_RETRY_TIMES; retry++) {
            if (__gt9xx_write(priv, buf, GT9XX_ADDR_LENGTH + cfg_len) == MS_ERR_NONE) {
                break;
            }
        }

#if GT9XX_DEBUG_EN > 0
        {
            int i;

            memset(&buf[2], 0, cfg_len);

            __gt9xx_read_reg(priv, GT9XX_REG_CFGS, &buf[2], cfg_len);
            for (i = 2; i < (2 + cfg_len); i++) {
                ms_printk(MS_PK_DEBUG, "0x%02X ", buf[i]);
            }
        }
#endif

        /*
         * Free configure table buffer
         */
        ms_kfree(buf);

        if (retry != GT9XX_MAX_RETRY_TIMES) {
            ret = 0;
        } else {
            ms_thread_set_errno(EIO);
            ret = -1;
        }
    } else {
        ms_thread_set_errno(ENOMEM);
        ret = -1;
    }

    return ret;
}

/*
 * Clear GT9XX state.
 */
static ms_err_t __gt9xx_clear_state(privinfo_t *priv)
{
    ms_uint8_t data[3];

    data[0] = GT9XX_TP_COOR_ADDR >> 8;
    data[1] = GT9XX_TP_COOR_ADDR & 0xFF;
    data[2] = 0;

    return __gt9xx_write(priv, data, 3);
}

/*
 * Get GT9XX state.
 */
static int __gt9xx_get_state(privinfo_t *priv, ms_touch_event_t *touch_event)
{
    ms_uint8_t *point_data = priv->tp_buf;
    int         ret;

    if (__gt9xx_read_reg(priv, GT9XX_TP_COOR_ADDR, point_data, 9) == MS_ERR_NONE) {
        ms_uint8_t  finger;
        ms_uint8_t  touch_num;

        finger    = point_data[0];
        touch_num = finger & 0x0F;

        if ((finger & 0x80) && (touch_num <= GT9XX_MAX_TOUCH_NUM)) {
            int i;

            if (touch_num > 1) {
                ms_err_t err = __gt9xx_read_reg(priv, (GT9XX_TP_COOR_ADDR + 9),
                                                &point_data[9], (8 * (touch_num - 1) + 1));
                if (err != MS_ERR_NONE) {
                    touch_num = 1;
                }
            }

            touch_event->touch_detected = touch_num;

            /*
             * Get all point date.
             */
            if (touch_num > 0) {
                ms_uint8_t *coor_data;

                for (i = 0; i < touch_num; i++) {
                    coor_data = &point_data[i * 8 + 1];

                    touch_event->touch_x[i] = coor_data[1] | (coor_data[2] << 8);
                    touch_event->touch_y[i] = coor_data[3] | (coor_data[4] << 8);
                    touch_event->touch_weight[i] = coor_data[5] | (coor_data[6] << 8);
                    touch_event->touch_event_id[i] = MS_TOUCH_EVENT_ID_PRESS_DOWN;
                }
            }

            /*
             * Check last points record.
             */
            int disappeared_num = 0;
            if (priv->last_points_num > 0) {

                /*
                 * Iterate all last points.
                 */
                for (i = 0; i < priv->last_points_num; i++) {
                    int j;
                    int found = 0;

                    for (j = 0; j < touch_num; j++) {
                        /*
                         * Last point still exist.
                         */
                        if ((priv->last_record[i].coordinate_x == touch_event->touch_x[j]) &&
                            (priv->last_record[i].coordinate_y == touch_event->touch_y[j])) {
                            found = 1;
                            touch_event->touch_event_id[j] = MS_TOUCH_EVENT_ID_CONTACT;
                            break;
                        }
                    }

                    /*
                     * Last point disappear.
                     */
                    if (found == 0) {
                        int index;

                        ++disappeared_num;
                        index = touch_num + disappeared_num - 1;

                        if (index < MS_TOUCH_MAX_POINT) {
                            touch_event->touch_x[index] = priv->last_record[i].coordinate_x;
                            touch_event->touch_y[index] = priv->last_record[i].coordinate_y;
                            touch_event->touch_event_id[index] = MS_TOUCH_EVENT_ID_LIFT_UP;
                        }
                    }
                }

                /*
                 * Clear last points record.
                 */
                priv->last_points_num = 0;
            }

            /*
             * Record last points.
             */
            if (priv->last_points_num == 0) {
                priv->last_points_num = touch_num;

                for (i = 0; i < touch_num; i++) {
                    priv->last_record[i].coordinate_x = touch_event->touch_x[i];
                    priv->last_record[i].coordinate_y = touch_event->touch_y[i];
                }
            }

            /*
             * Report disappeared points.
             */
            if (disappeared_num > 0) {
                touch_event->touch_detected = (touch_num + disappeared_num) > MS_TOUCH_MAX_POINT ? 
                                               MS_TOUCH_MAX_POINT : (touch_num + disappeared_num);
            }

            /*
             * Clear point buffer state.
             */
            for (i = 0; i < GT9XX_MAX_RETRY_TIMES; i++) {
                if (__gt9xx_clear_state(priv) == MS_ERR_NONE) {
                    break;
                }
            }

            if (i != GT9XX_MAX_RETRY_TIMES) {
                ret = 0;
            } else {
                ms_thread_set_errno(EIO);
                ret = -1;
            }
        } else {
            /*
             * No invalid point detected.
             */
            touch_event->touch_detected = 0;
            ret = 0;
        }
    } else {
        ms_thread_set_errno(EIO);
        ret = -1;
    }

    return ret;
}

/*
 * Open device
 */
static int __gt9xx_open(ms_ptr_t ctx, ms_io_file_t *file, int oflag, ms_mode_t mode)
{
    privinfo_t *priv = ctx;
    int         ret;

    if (ms_atomic_inc(MS_IO_DEV_REF(file)) == 1) {
        /*
         * Allocate points data buffer
         */
        priv->tp_buf_len = 1 + 8 * GT9XX_MAX_TOUCH_NUM + 1;
        priv->tp_buf = ms_kmalloc(priv->tp_buf_len);
        if (priv->tp_buf != MS_NULL) {
            ms_err_t err;

            /*
             * Configure touch panel chip.
             */
            err = __gt9xx_reset(priv);
            if (err == MS_ERR_NONE) {
                err = __gt9xx_config(priv);
                if (err == MS_ERR_NONE) {
                    err = __gt9xx_start(priv);
                }
            }

            if (err == MS_ERR_NONE) {
                ret = 0;

            } else {
                ms_kfree(priv->tp_buf);
                priv->tp_buf = MS_NULL;

                ms_atomic_dec(MS_IO_DEV_REF(file));
                ms_thread_set_errno(EIO);
                ret = -1;
            }
        } else {
            ms_atomic_dec(MS_IO_DEV_REF(file));
            ms_thread_set_errno(ENOMEM);
            ret = -1;
        }
    } else {
        ms_thread_set_errno(EBUSY);
        ret = -1;
    }

    return ret;
}

/*
 * Close device
 */
static int __gt9xx_close(ms_ptr_t ctx, ms_io_file_t *file)
{
    privinfo_t *priv = ctx;

    if (ms_atomic_dec(MS_IO_DEV_REF(file)) == 0) {
        ms_kfree(priv->tp_buf);
    }

    return 0;
}

/*
 * Read device
 */
static ms_ssize_t __gt9xx_read(ms_ptr_t ctx, ms_io_file_t *file, ms_ptr_t buf, ms_size_t len)
{
    privinfo_t       *priv = ctx;
    ms_touch_event_t *event = (ms_touch_event_t *)buf;
    ssize_t           ret;

    if (len == sizeof(ms_touch_event_t)) {
        if (__gt9xx_get_state(priv, event) == MS_ERR_NONE) {
            ret = sizeof(ms_touch_event_t);
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
 * Get device status
 */
static int __gt9xx_fstat(ms_ptr_t ctx, ms_io_file_t *file, ms_stat_t *buf)
{
    privinfo_t *priv = ctx;

    buf->st_size = priv->tp_buf_len;

    return 0;
}

/*
 * Control device
 */
static int __gt9xx_ioctl(ms_ptr_t ctx, ms_io_file_t *file, int cmd, void *arg)
{
    privinfo_t      *priv = ctx;
    ms_gtxxx_port_t *port = priv->port;
    int              ret;

    switch (cmd) {
    case MS_TOUCH_CMD_GET_INFO:
        if (ms_access_ok(arg, sizeof(ms_touch_info_t), MS_ACCESS_W)) {
            ms_touch_info_t *info = (ms_touch_info_t *)arg;
            info->caps      = MS_TOUCH_CAP_MULTI_TOUCH;
            info->max_touch = GT9XX_MAX_TOUCH_NUM;
            info->max_x     = port->abs_x_max;
            info->max_y     = port->abs_y_max;
            ret = 0;
        } else {
            ms_thread_set_errno(EFAULT);
            ret = -1;
        }
        break;

    default:
        ms_thread_set_errno(EINVAL);
        ret = -1;
        break;
    }

    return ret;
}

/*
 * Device operating function set
 */
static ms_io_driver_ops_t ms_gt9xx_drv_ops = {
        .type   = MS_IO_DRV_TYPE_CHR,
        .open   = __gt9xx_open,
        .close  = __gt9xx_close,
        .read   = __gt9xx_read,
        .ioctl  = __gt9xx_ioctl,
        .fstat  = __gt9xx_fstat,
};

/*
 * Device driver
 */
static ms_io_driver_t ms_gt9xx_drv = {
        .nnode = {
            .name = MS_GT9XX_DRV_NAME,
        },
        .ops = &ms_gt9xx_drv_ops,
};

/*
 * Register gt9xx device driver
 */
ms_err_t ms_gt9xx_drv_register(void)
{
    return ms_io_driver_register(&ms_gt9xx_drv);
}

/*
 * Create gt9xx device file
 */
ms_err_t ms_gt9xx_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr, ms_gtxxx_port_t *port)
{
    ms_gt9xx_dev_t *dev;
    ms_err_t        err;

    if ((path != MS_NULL) && (i2c_bus_name != MS_NULL) && (port != MS_NULL)) {

        dev = ms_kmalloc(sizeof(ms_gt9xx_dev_t));
        if (dev != MS_NULL) {
            privinfo_t *priv = &dev->priv;

            /*
             * Private info init
             */
            priv->tp_buf = MS_NULL;
            priv->tp_buf_len = 0;

            /*
             * Check touch screen device info
             */
            err = __gt9xx_port_check(port);
            if (err == MS_ERR_NONE) {

                /*
                 * Get touch screen device info
                 */
                priv->port = port;

                /*
                 * Attach i2c device to i2c bus
                 */
                priv->i2c_dev.nnode.name = MS_GT9XX_DRV_NAME;
                priv->i2c_dev.clk_speed  = MS_I2C_CLK_SPEED_STANDARD;
                priv->i2c_dev.addr       = dev_addr;
                priv->i2c_dev.addrlen    = 7U;
                priv->i2c_dev.ctx        = MS_NULL;
                err = ms_i2c_device_attach(&priv->i2c_dev, i2c_bus_name);
                if (err == MS_ERR_NONE) {
                    /*
                     * Check touch screen product id
                     */
                    err = __gt9xx_pid_check(priv);
                    if (err == MS_ERR_NONE) {
                        err = ms_io_device_register(&dev->dev, path, MS_GT9XX_DRV_NAME, &dev->priv);
                    }

                    if (err != MS_ERR_NONE) {
                        ms_i2c_device_detach(&priv->i2c_dev, i2c_bus_name);
                    }
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
