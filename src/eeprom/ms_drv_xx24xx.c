/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_xx24xx.c xx24xx EEPROM device driver.
 *
 * Author: Jiao.jinxing <jiaojinxing@acoinfo.com>
 *
 */

#define __MS_IO
#include "ms_kern.h"
#include "ms_io_core.h"
#include "ms_driver.h"
#include "ms_drv_xx24xx.h"

#include <string.h>

/**
 * @brief xx24xx EEPROM device driver.
 */

#if MS_CFG_IO_MODULE_EN > 0

#define MS_XX24XX_DRV_NAME      "xx24xx_eeprom"

/*
 * Device geometry description, compact form (2 bytes per entry)
 */
typedef struct {
    ms_uint8_t bytes    : 4; /* Power of two of 128 bytes (0:128 ... 11:262144) */
    ms_uint8_t pagesize : 4; /* Power of two of   8 bytes (0:8 1:16 2:32 3:64 etc) */
    ms_uint8_t addrlen  : 4; /* Nr of bytes in command address field */
    ms_uint8_t abits    : 3; /* Nr of Address MSBits in the i2c device address LSBs */
    ms_uint8_t special  : 1; /* Special device: uchip 24xx00 (total 16 bytes)
                              * or 24xx1025 (shifted P bits) */
} ms_ee24xx_geom_t;

/*
 * Supported device geometries.
 * One geometry can fit more than one device.
 * The user will use an enum'ed index from include/eeprom/i2c_xx24xx.h
 */
static const ms_ee24xx_geom_t ms_ee24xx_devices[] = {
    /* Microchip devices */

    /* by pg al ab sp  device bytes page  alen */

    {
      0, 1, 1, 0, 1
    }, /* 24xx00        16    1     1 Ridiculously small device */
    {
      0, 0, 1, 0, 0
    }, /* 24xx01       128    8     1 */
    {
      1, 0, 1, 0, 0
    }, /* 24xx02       256    8     1 */
    {
      2, 1, 1, 1, 0
    }, /* 24xx04       512   16     1 */
    {
      3, 1, 1, 2, 0
    }, /* 24xx08      1024   16     1 */
    {
      4, 1, 1, 3, 0
    }, /* 24xx16      2048   16     1 */
    {
      5, 2, 2, 0, 0
    }, /* 24xx32      4096   32     2 */
    {
      6, 2, 2, 0, 0
    }, /* 24xx64      8192   32     2 */
    {
      7, 3, 2, 0, 0
    }, /* 24xx128    16384   64     2 */
    {
      8, 3, 2, 0, 0
    }, /* 24xx256    32768   64     2 */
    {
      9, 4, 2, 0, 0
    }, /* 24xx512    65536  128     2 */
    {
      10, 4, 2, 1, 1
    }, /* 24xx1025  131072  128     2 Shifted address, todo */
    {
      10, 4, 2, 1, 0
    }, /* 24xx1026  131072  128     2 */
    {
      11, 5, 2, 2, 0
    }, /* AT24CM02  262144  256     2 */

    /* STM devices */

    {
      0, 1, 1, 0, 0
    }, /* M24C01       128   16     1 */
    {
      1, 1, 1, 0, 0
    }, /* M24C02       256   16     1 */
    {
      11, 5, 2, 2, 0
    }, /* M24M02    262144  256     2 */
};

/*
 * Private info
 */
typedef struct {
    ms_i2c_device_t i2c_dev;
    ms_handle_t     lock;

    /*
     * Expanded from geometry
     */
    ms_uint32_t     size;       /* total bytes in device */
    ms_uint16_t     pgsize;     /* write block size, in bytes */
    ms_uint16_t     addrlen;    /* number of bytes in data addresses */
    ms_uint16_t     haddrbits;  /* Number of bits in high address part */
    ms_uint16_t     haddrshift; /* bit-shift of high address part */
} privinfo_t;

/*
 * xx24xx device
 */
typedef struct {
    privinfo_t      priv;
    ms_io_device_t  dev;
} ms_xx24xx_dev_t;

/*
 * Open device
 */
static int __xx24xx_open(ms_ptr_t ctx, ms_io_file_t *file, int oflag, ms_mode_t mode)
{
    ms_atomic_inc(MS_IO_DEV_REF(file));

    return 0;
}

/*
 * Close device
 */
static int __xx24xx_close(ms_ptr_t ctx, ms_io_file_t *file)
{
    ms_atomic_dec(MS_IO_DEV_REF(file));

    return 0;
}

/*
 * Read device
 */
static ms_ssize_t __xx24xx_read(ms_ptr_t ctx, ms_io_file_t *file, ms_ptr_t buf, ms_size_t len)
{
    ms_ssize_t ret;

    if (len == sizeof(ms_eeprom_msg_t)) {
        ms_eeprom_msg_t *eeprom_msg = buf;
        ms_ptr_t  rbuf;
        ms_size_t rlen;

        rbuf = eeprom_msg->buf;
        rlen = eeprom_msg->len;

        if (ms_access_ok(rbuf, rlen, MS_ACCESS_W)) {
            privinfo_t  *priv = ctx;
            ms_i2c_msg_t msgs[2U];
            ms_uint8_t   maddr[2U];
            ms_uint32_t  addr_hi;

            addr_hi = (eeprom_msg->memaddr >> (priv->addrlen << 3U));

            maddr[0U] = (eeprom_msg->memaddr) >> 8U;
            maddr[1U] = (eeprom_msg->memaddr) &  0xffU;

            msgs[0U].clk_speed = priv->i2c_dev.clk_speed;
            msgs[0U].addr      = priv->i2c_dev.addr | (addr_hi & ((1U << priv->haddrbits) - 1U));
            msgs[0U].flags     = MS_I2C_M_NOSTOP;
            msgs[0U].buf       = (priv->addrlen == 2U) ? &maddr[0U] : &maddr[1U];
            msgs[0U].len       = priv->addrlen;

            /*
             * Read data
             */
            msgs[1U].clk_speed = msgs[0U].clk_speed;
            msgs[1U].addr      = msgs[0U].addr;
            msgs[1U].flags     = MS_I2C_M_READ;
            msgs[1U].buf       = rbuf;
            msgs[1U].len       = rlen;

            (void)ms_mutex_lock(priv->lock, MS_TIMEOUT_FOREVER);

            if (ms_i2c_device_trans(&priv->i2c_dev, msgs, 2U) == 2U) {
                ret = len;
            } else {
                ms_thread_set_errno(EIO);
                ret = -1;
            }

            (void)ms_mutex_unlock(priv->lock);

        } else {
            ms_thread_set_errno(EFAULT);
            ret = -1;
        }
    } else {
        ms_thread_set_errno(EINVAL);
        ret = -1;
    }

    return ret;
}

/*
 * Wait write complete
 */
static ms_err_t __xx24xx_waitwritecomplete(privinfo_t *priv, ms_uint32_t memaddr)
{
    ms_i2c_msg_t msgs[1U];
    ms_uint8_t   adr;
    ms_uint32_t  addr_hi = (memaddr >> (priv->addrlen << 3U));
    ms_uint32_t  retries = 10U;
    ms_err_t     err = MS_ERR_IO;

    msgs[0U].clk_speed = priv->i2c_dev.clk_speed;
    msgs[0U].addr      = priv->i2c_dev.addr | (addr_hi & ((1U << priv->haddrbits) - 1U));
    msgs[0U].flags     = MS_I2C_M_READ;
    msgs[0U].buf       = &adr;
    msgs[0U].len       = 1U;

    do {
        if (ms_i2c_device_trans(&priv->i2c_dev, msgs, 1U) == 1) {
            err = MS_ERR_NONE;
            break;
        }
        retries--;
        ms_thread_sleep_ms(1U);
    } while (retries > 0U);

    return err;
}

/*
 * Write page
 */
static ms_err_t __xx24xx_writepage(privinfo_t *priv, ms_uint32_t memaddr, ms_ptr_t buf, ms_size_t len)
{
    ms_i2c_msg_t msgs[2U];
    ms_uint8_t   maddr[2U];
    ms_uint32_t  addr_hi = (memaddr >> (priv->addrlen << 3U));

    /*
     * Write data address
     */
    maddr[0U] = (memaddr) >> 8U;
    maddr[1U] = (memaddr) &  0xffU;

    msgs[0U].clk_speed = priv->i2c_dev.clk_speed;
    msgs[0U].addr      = priv->i2c_dev.addr | (addr_hi & ((1U << priv->haddrbits) - 1U));
    msgs[0U].flags     = MS_I2C_M_NOSTOP;
    msgs[0U].buf       = (priv->addrlen == 2U) ? &maddr[0U] : &maddr[1U];
    msgs[0U].len       = priv->addrlen;

    /*
     * Write data without a restart nor a control byte
     */
    msgs[1U].clk_speed = msgs[0U].clk_speed;
    msgs[1U].addr      = msgs[0U].addr;
    msgs[1U].flags     = MS_I2C_M_NOSTART;
    msgs[1U].buf       = buf;
    msgs[1U].len       = len;

    return (ms_i2c_device_trans(&priv->i2c_dev, msgs, 2U) == 2U) ? MS_ERR_NONE : MS_ERR;
}

/*
 * Write device
 */
static ms_ssize_t __xx24xx_write(ms_ptr_t ctx, ms_io_file_t *file, ms_const_ptr_t buf, ms_size_t len)
{
    ms_ssize_t ret;

    if (len == sizeof(ms_eeprom_msg_t)) {
        ms_eeprom_msg_t *eeprom_msg = (ms_eeprom_msg_t *)buf;

        if (ms_eeprom_write_ok(eeprom_msg->memaddr, eeprom_msg->len)) {
            ms_ptr_t  wbuf;
            ms_size_t wlen;

            wbuf = eeprom_msg->buf;
            wlen = eeprom_msg->len;

            if (ms_access_ok(wbuf, wlen, MS_ACCESS_R)) {
                privinfo_t *priv = ctx;
                ms_uint32_t pos = eeprom_msg->memaddr;
                ms_uint32_t pageoff;
                ms_size_t   cnt;

                /*
                 * First, write some page-unaligned data
                 */
                pageoff = pos & (priv->pgsize - 1U);
                cnt     = priv->pgsize - pageoff;
                if (cnt > wlen) {
                    cnt = wlen;
                }

                (void)ms_mutex_lock(priv->lock, MS_TIMEOUT_FOREVER);

                do {
                    if (pageoff > 0U) {
                        ret = __xx24xx_writepage(priv, pos, wbuf, cnt);
                        if (ret < 0) {
                            break;
                        }

                        ret = __xx24xx_waitwritecomplete(priv, pos);
                        if (ret < 0) {
                            break;
                        }

                        wlen -= cnt;
                        wbuf += cnt;
                        pos  += cnt;
                    } else {
                        ret = 0;
                    }

                    if (ret == 0) {
                        /*
                         * Then, write remaining bytes at page-aligned addresses
                         */
                        while (wlen > 0U) {
                            cnt = wlen;
                            if (cnt > priv->pgsize) {
                                cnt = priv->pgsize;
                            }

                            ret = __xx24xx_writepage(priv, pos, wbuf, cnt);
                            if (ret < 0) {
                                break;
                            }

                            ret = __xx24xx_waitwritecomplete(priv, pos);
                            if (ret < 0) {
                                break;
                            }

                            wlen -= cnt;
                            wbuf += cnt;
                            pos  += cnt;
                        }
                    }
                } while (MS_FALSE);

                (void)ms_mutex_unlock(priv->lock);

                if (ret == 0) {
                    ret = len;
                } else {
                    ms_thread_set_errno(EIO);
                    ret = -1;
                }
            } else {
                ms_thread_set_errno(EFAULT);
                ret = -1;
            }
        } else {
            ms_thread_set_errno(EACCES);
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
static int __xx24xx_fstat(ms_ptr_t ctx, ms_io_file_t *file, ms_stat_t *buf)
{
    privinfo_t *priv = ctx;

    buf->st_size = priv->size;

    return 0;
}

/*
 * Control device
 */
static int __xx24xx_ioctl(ms_ptr_t ctx, ms_io_file_t *file, int cmd, ms_ptr_t arg)
{
    privinfo_t *priv = ctx;
    int ret;

    switch (cmd) {
    case MS_EEPROM_CMD_GET_GEOMETRY:
        if (ms_access_ok(arg, sizeof(ms_eeprom_geometry_t), MS_ACCESS_W)) {
            ms_eeprom_geometry_t *geometry = (ms_eeprom_geometry_t *)arg;
            geometry->size = priv->size;
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
static ms_io_driver_ops_t ms_xx24xx_drv_ops = {
        .type   = MS_IO_DRV_TYPE_CHR,
        .open   = __xx24xx_open,
        .close  = __xx24xx_close,
        .read   = __xx24xx_read,
        .write  = __xx24xx_write,
        .fstat  = __xx24xx_fstat,
        .ioctl  = __xx24xx_ioctl,
};

/*
 * Device driver
 */
static ms_io_driver_t ms_xx24xx_drv = {
        .nnode = {
            .name = MS_XX24XX_DRV_NAME,
        },
        .ops = &ms_xx24xx_drv_ops,
};

/*
 * Register xx24xx device driver
 */
ms_err_t ms_xx24xx_drv_register(void)
{
    return ms_io_driver_register(&ms_xx24xx_drv);
}

/*
 * Create xx24xx device file
 */
ms_err_t ms_xx24xx_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr, ms_xx24xx_type_t dev_type)
{
    ms_xx24xx_dev_t *dev;
    const char *name;
    ms_err_t err;

    if ((path != MS_NULL) && (i2c_bus_name != MS_NULL)) {

        if (dev_type < MS_ARRAY_SIZE(ms_ee24xx_devices)) {

            dev = ms_kmalloc(sizeof(ms_xx24xx_dev_t));
            if (dev != MS_NULL) {
                privinfo_t *priv = &dev->priv;

                bzero(priv, sizeof(privinfo_t));

                err = ms_mutex_create("xx24xx_lock", MS_WAIT_TYPE_PRIO, &priv->lock);
                if (err == MS_ERR_NONE) {

                    /*
                     * Find Device Name
                     */
                    name = path + strlen(path) - 1;
                    while (name > path && *name != '/') {
                        name--;
                    }
                    priv->i2c_dev.nnode.name = name;
                    priv->i2c_dev.clk_speed  = MS_I2C_CLK_SPEED_STANDARD;
                    priv->i2c_dev.addr       = dev_addr;
                    priv->i2c_dev.addrlen    = 7U;
                    priv->i2c_dev.ctx        = MS_NULL;

                    err = ms_i2c_device_attach(&priv->i2c_dev, i2c_bus_name);
                    if (err == MS_ERR_NONE) {
                        /*
                         * Expand device geometry from compacted info
                         */
                        priv->size       = 128U << ms_ee24xx_devices[dev_type].bytes;
                        priv->pgsize     =   8U << ms_ee24xx_devices[dev_type].pagesize;
                        priv->addrlen    =         ms_ee24xx_devices[dev_type].addrlen;
                        priv->haddrbits  =         ms_ee24xx_devices[dev_type].abits;
                        priv->haddrshift = 0U;

                        /*
                         * Apply special properties
                         */
                        if (ms_ee24xx_devices[dev_type].special) {
                            if (dev_type == EEPROM_24XX00) {
                                /*
                                 * Ultra small 16-byte EEPROM
                                 */
                                priv->size = 16U;

                                /*
                                 * The device only has BYTE write,
                                 * which is emulated with 1-byte pages
                                 */
                                priv->pgsize = 1U;

                            } else if (dev_type == EEPROM_24XX1025) {
                                /*
                                 * Microchip alien part where the address MSB is << 2 bits
                                 */
                                ms_printk(MS_PK_ERR, "Device 24xx1025 is not supported for the moment, TODO.\n");
                                err = MS_ERR;
                            }
                        }

                        if (err == MS_ERR_NONE) {
                            err = ms_io_device_register(&dev->dev, path, MS_XX24XX_DRV_NAME, &dev->priv);
                        }

                        if (err != MS_ERR_NONE) {
                            ms_i2c_device_detach(&priv->i2c_dev, i2c_bus_name);
                        }
                    }

                    if (err != MS_ERR_NONE) {
                        (void)ms_mutex_destroy(priv->lock);
                    }
                }

                if (err != MS_ERR_NONE) {
                    (void)ms_kfree(dev);
                }
            } else {
                err = MS_ERR_KERN_HEAP_NO_MEM;
            }
        } else {
            err = MS_ERR_ARG_INVALID;
        }
    } else {
        err = MS_ERR_ARG_NULL_PTR;
    }

    return err;
}

#endif
