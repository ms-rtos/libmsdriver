/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_ahtxx.c AHTXX device driver.
 *
 * Author: Yu.Kangzhi <yukangzhi@acoinfo.com>
 *
 */

#define __MS_IO
#include "ms_kern.h"
#include "ms_io_core.h"
#include "ms_driver.h"
#include <string.h>

#include "ms_drv_ahtxx.h"

/**
 * @brief AHTXX device driver.
 */

#if MS_CFG_IO_MODULE_EN > 0

#define MS_AHTXX_DRV_NAME           "ahtxx_sensor"

#define AHT10_CMD_SOFT_INIT         (0xE1)

#define AHT20_CMD_SOFT_INIT         (0xBE)
#define AHT20_CMD_SOFT_RESET        (0xBA)

#define AHTXX_CMD_MEAS_TRIG         (0xAC)
#define AHTXX_CMD_MEAS_READ         (0x71)
#define AHTXX_CMD_NORMAL_MODE       (0xA8)

#define AHTXX_STATUS_BUSY           (1 << 7)
#define AHTXX_STATUS_CAL_EN         (1 << 3)

#define AHTXX_INIT_PARAM1           (0x08)
#define AHTXX_INIT_PARAM2           (0x00)

#define AHTXX_TRIG_PARAM1           (0x33)
#define AHTXX_TRIG_PARAM2           (0x00)

#define AHTXX_MODE_PARAM1           (0x00)
#define AHTXX_MODE_PARAM2           (0x00)

#define AHTXX_MEASURE_TIME          (75)
#define AHTXX_SENSOR_DATA_LEN       (6)

#define AHTXX_OVERFLOW_LINE         (0U)
#define AHTXX_UNDERFLOW_LINE        (1U)

#define AHTXX_DEFAULT_CHACHE_NUM    (6U)

/*
 * Sensor data cache entry
 */
typedef struct {
    ms_tick_t       timestamp;
    ms_uint8_t      type;
    union {
        ms_int32_t  humidity;
        ms_int32_t  temperature;
    } u;
} ms_ahtxx_cache_t;

/*
 * Private info
 */
typedef struct {
    ms_pollfd_t    *slots[1U];
    ms_i2c_device_t i2c_dev;
    ms_ahtxx_type_t type;

    ms_tick_t       timestamp;
    double          humidity;
    double          temperature;

    ms_io_job_t     measure_job;
    ms_fifo_t       data_fifo;
    ms_uint16_t     mode;
    ms_tick_t       interval;
    ms_bool_t       temp_line_enable[2];
    ms_int32_t      temp_line[2];
    ms_bool_t       humi_line_enable[2];
    ms_int32_t      humi_line[2];
} privinfo_t;

/*
 * AHTXX device
 */
typedef struct {
    privinfo_t     priv;
    ms_io_device_t dev;
} ms_ahtxx_dev_t;

/*
 * Read AHTXX register.
 */
static inline ms_err_t __ahtxx_read_reg(privinfo_t *priv, ms_uint8_t reg, ms_uint8_t *buf, ms_uint16_t len)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;

    return ms_i2c_device_writeread(i2c_dev, &reg, 1, 0, buf, len, 0);
}

/*
 * Write AHTXX register.
 */
static inline ms_err_t __ahtxx_write_reg(privinfo_t *priv, ms_uint8_t reg, ms_uint8_t *buf, ms_uint16_t len)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;
    ms_i2c_msg_t     msgs[2U];

    /*
     * Write register address
     */
    msgs[0U].clk_speed = i2c_dev->clk_speed;
    msgs[0U].addr      = i2c_dev->addr;
    msgs[0U].flags     = MS_I2C_M_NOSTOP;
    msgs[0U].buf       = &reg;
    msgs[0U].len       = 1;

    /*
     * Write data without a restart nor a control byte
     */
    msgs[1U].clk_speed = msgs[0U].clk_speed;
    msgs[1U].addr      = msgs[0U].addr;
    msgs[1U].flags     = MS_I2C_M_NOSTART;
    msgs[1U].buf       = buf;
    msgs[1U].len       = len;

    return (ms_i2c_device_trans(i2c_dev, msgs, 2U) == 2U) ? MS_ERR_NONE : MS_ERR;
}

/*
 * Transmit data to AHTXX.
 */
static inline ms_err_t __ahtxx_write(privinfo_t *priv, ms_uint8_t *data, ms_uint16_t len)
{
    ms_i2c_device_t *i2c_dev = &priv->i2c_dev;

    return ms_i2c_device_write(i2c_dev, data, len, 0);
}

/*
 * Decode AHTXX raw data.
 */
static ms_err_t __ahtxx_decode(ms_uint8_t *data, double *hum, double *temp)
{
    ms_uint8_t  byte_2th  = 0;
    ms_uint8_t  byte_3th  = 0;
    ms_uint8_t  byte_4th  = 0;
    ms_uint8_t  byte_5th  = 0;
    ms_uint8_t  byte_6th  = 0;
    ms_uint32_t retu_data = 0;
    ms_uint32_t ct_data[2];
    int         c1, t1;

    if (!data || !hum || !temp) {
        return MS_ERR;
    }

    byte_2th = data[0];
    byte_3th = data[1];
    byte_4th = data[2];
    byte_5th = data[3];
    byte_6th = data[4];

    retu_data  = (retu_data | byte_2th) << 8;
    retu_data  = (retu_data | byte_3th) << 8;
    retu_data  = (retu_data | byte_4th);
    retu_data  = retu_data >> 4;
    ct_data[0] = retu_data;

    retu_data  = 0;
    retu_data  = (retu_data | byte_4th) << 8;
    retu_data  = (retu_data | byte_5th) << 8;
    retu_data  = (retu_data | byte_6th);
    retu_data  = retu_data & 0xfffff;
    ct_data[1] = retu_data;

    c1 = ct_data[0] * 100 * 10 / 1024 / 1024;
    t1 = ct_data[1] * 200 * 10 / 1024 / 1024 - 500;

    *hum  = c1 * 1.0 / 10;
    *temp = t1 * 1.0 / 10;

    return MS_ERR_NONE;
}

/*
 * Initialize AHTXX.
 */
static ms_err_t __ahtxx_init(privinfo_t *priv)
{
    ms_uint8_t data[3];
    ms_err_t   err;

    ms_thread_sleep_ms(40);

    data[0] = AHTXX_CMD_NORMAL_MODE;
    data[1] = AHTXX_MODE_PARAM1;
    data[2] = AHTXX_MODE_PARAM2;
    err = __ahtxx_write(priv, data, 3);
    if (err != MS_ERR_NONE) {
        return err;
    }
    ms_thread_sleep_ms(11);

    data[0] = (priv->type == MS_AHTXX_TYPE_AHT20) ? AHT20_CMD_SOFT_INIT : AHT10_CMD_SOFT_INIT;
    data[1] = AHTXX_INIT_PARAM1;
    data[2] = AHTXX_INIT_PARAM2;
    err = __ahtxx_write(priv, data, 2);
    if (err != MS_ERR_NONE) {
        return err;
    }
    ms_thread_sleep_ms(40);

    return MS_ERR_NONE;
}

/*
 * Fetch AHTXX data.
 */
static ms_err_t __ahtxx_fetch_data(privinfo_t *priv)
{
    ms_uint8_t data[AHTXX_SENSOR_DATA_LEN];
    ms_uint8_t status;
    ms_uint8_t try_cnt = 10;
    ms_err_t   err;

    /*
     * Trigger measure
     */
    data[0] = AHTXX_CMD_MEAS_TRIG;
    data[1] = AHTXX_TRIG_PARAM1;
    data[2] = AHTXX_TRIG_PARAM2;
    err = __ahtxx_write(priv, data, 3);
    if (err != MS_ERR_NONE) {
        return err;
    }
    ms_thread_sleep_ms(80);

    /*
     * Check sensor status
     */
    do {
        err = __ahtxx_read_reg(priv, AHTXX_CMD_MEAS_READ, data, 1);
        if ((err != MS_ERR_NONE) || (try_cnt == 0)) {
            break;
        }
        status = data[0];
        --try_cnt;
    } while ((status & AHTXX_STATUS_BUSY));

    /*
     * Read sensor data
     */
    if ((err == MS_ERR_NONE) && (try_cnt != 0)) {
        err = __ahtxx_read_reg(priv, AHTXX_CMD_MEAS_READ, data, AHTXX_SENSOR_DATA_LEN);

        if (err == MS_ERR_NONE) {
            /*
             * Calculate humidity and temperature
             */
            priv->timestamp = (ms_tick_t)ms_time_get();
            err = __ahtxx_decode(&data[1], &(priv->humidity), &(priv->temperature));
        }
    } else {
        err = MS_ERR;
    }

    return err;
}

/*
 * Open device
 */
static int __ahtxx_open(ms_ptr_t ctx, ms_io_file_t *file, int oflag, ms_mode_t mode)
{
    privinfo_t *priv = ctx;
    int ret;

    if (ms_atomic_inc(MS_IO_DEV_REF(file)) == 2) {
        ms_err_t err;

        err = __ahtxx_init(priv);
        if (err == MS_ERR_NONE) {
            ms_uint8_t data;

            err = __ahtxx_read_reg(priv, AHTXX_CMD_MEAS_READ, &data, 1);
            if ((err == MS_ERR_NONE) && (data & AHTXX_STATUS_CAL_EN)) {
                ret = 0;
            } else {
                ms_atomic_dec(MS_IO_DEV_REF(file));
                ms_thread_set_errno(EBUSY);
                ret = -1;
            }
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
static int __ahtxx_close(ms_ptr_t ctx, ms_io_file_t *file)
{
    ms_atomic_dec(MS_IO_DEV_REF(file));

    return 0;
}

/*
 * Fill the user sensor data buffer.
 */
static ms_size_t __ahtxx_fill_sensor_data(privinfo_t *priv, ms_sensor_data_t *sensor_data, ms_uint8_t data_count)
{
    int i;
    ms_size_t fifo_size;
    ms_size_t cache_data_count;
    ms_ahtxx_cache_t cache_data;

    fifo_size = ms_fifo_size(&priv->data_fifo);
    cache_data_count = fifo_size / sizeof(ms_ahtxx_cache_t);
    cache_data_count = (cache_data_count < data_count) ? cache_data_count: data_count;

    for (i = 0; i < cache_data_count; i++) {
        ms_fifo_get(&priv->data_fifo, &cache_data, sizeof(ms_ahtxx_cache_t));

        sensor_data->type = cache_data.type;
        if (MS_SENSOR_TYPE_TEMP == cache_data.type) {
            sensor_data->u.temp = (ms_int32_t)cache_data.u.temperature;
        } else if (MS_SENSOR_TYPE_HUMI == cache_data.type) {
            sensor_data->u.humi = (ms_int32_t)cache_data.u.humidity;
        }

        sensor_data++;
    }

    return cache_data_count;
}

/*
 * Read device
 */
static ms_ssize_t __ahtxx_read(ms_ptr_t ctx, ms_io_file_t *file, ms_ptr_t buf, ms_size_t len)
{
    privinfo_t *priv = ctx;
    ms_uint8_t  data_count;
    ms_ssize_t  ret;

    data_count = len / sizeof(ms_sensor_data_t);
    if ((data_count >= 1) && (len % sizeof(ms_sensor_data_t) == 0)) {

        if ((priv->mode & MS_AHTXX_MODE_TRIG_MODE_MASK) == MS_AHTXX_MODE_TRIG_ON_READ) {
            /*
             * Get sensor data immediately
             */
            if (__ahtxx_fetch_data(priv) == MS_ERR_NONE) {
                ms_sensor_data_t *sensor_data = (ms_sensor_data_t *)buf;
                int i;

                for (i = 0; i < data_count; i++) {
                    if (MS_SENSOR_TYPE_TEMP == sensor_data->type) {
                        sensor_data->u.temp = (ms_int32_t)priv->temperature;

                    } else if (MS_SENSOR_TYPE_HUMI == sensor_data->type) {
                        sensor_data->u.humi = (ms_int32_t)priv->humidity;

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
            ms_size_t num;
            ms_sensor_data_t *sensor_data = (ms_sensor_data_t *)buf;
            num = __ahtxx_fill_sensor_data(priv, sensor_data, data_count);
            ret = num * sizeof(ms_sensor_data_t);
        }

    } else {
        ms_thread_set_errno(EINVAL);
        ret = -1;
    }

    return ret;
}

/*
 * Check device readable
 */
static ms_bool_t __ahtxx_readable_check(ms_ptr_t ctx)
{
    privinfo_t *priv = ctx;

    return ms_fifo_is_empty(&priv->data_fifo) ? MS_FALSE : MS_TRUE;
}

/*
 * Check device exception
 */
static ms_bool_t __ahtxx_except_check(ms_ptr_t ctx)
{
    privinfo_t *priv = ctx;

    (void)priv;

    /*
     * Note: report exception to user, then user reset the device.
     */
    return MS_TRUE;
}

/*
 * Device notify
 */
static int __ahtxx_poll_notify(privinfo_t *priv, ms_pollevent_t event)
{
    return ms_io_poll_notify_heaper(priv->slots, MS_ARRAY_SIZE(priv->slots), event);
}

/*
 * Poll device
 */
static int __ahtxx_poll(ms_ptr_t ctx, ms_io_file_t *file, ms_pollfd_t *fds, ms_bool_t setup)
{
    privinfo_t *priv = ctx;

    return ms_io_poll_heaper(fds, priv->slots, MS_ARRAY_SIZE(priv->slots), setup, ctx,
                             __ahtxx_readable_check, MS_NULL, __ahtxx_except_check);
}

/*
 * Get device status
 */
static int __ahtxx_fstat(ms_ptr_t ctx, ms_io_file_t *file, ms_stat_t *buf)
{
    privinfo_t *priv = ctx;

    (void)priv;

    /*
     * Note: No dynamic memory.
     */
    return 0;
}

/*
 * Resume measure work after param setting.
 * Configure hardware param or io_job_thread for different work mode.
 */
static ms_err_t __ahtxx_start_work(privinfo_t *priv)
{
    ms_err_t err;

    if ((priv->mode & MS_AHTXX_MODE_TRIG_MODE_MASK) == MS_AHTXX_MODE_TRIG_ON_READ) {
        err = MS_ERR_NONE;
    } else {
        if ((priv->mode & MS_AHTXX_MODE_MEASURE_MODE_MASK) == MS_AHTXX_MODE_MEASURE_ONCE) {
            err = ms_io_job_start(&priv->measure_job, 0, priv->interval, MS_IO_JOB_OPT_ONE_SHOT);
        } else {
            err = ms_io_job_start(&priv->measure_job, 0, priv->interval, MS_IO_JOB_OPT_PERIODIC);
        }
    }

    return err;
}

/*
 * Hung up measure work.
 * Stop hardware or io_job_thread for different work mode.
 */
static ms_err_t __ahtxx_stop_work(privinfo_t *priv)
{
    ms_err_t err;

    if ((priv->mode & MS_AHTXX_MODE_TRIG_MODE_MASK) == MS_AHTXX_MODE_TRIG_ON_READ) {
        err = MS_ERR_NONE;
    } else {
        err = ms_io_job_stop(&priv->measure_job);
    }

    return err;
}

/*
 * Set AHTXX work param.
 */
static ms_err_t __ahtxx_set_work_param(privinfo_t *priv, ms_ahtxx_param_t *ahtxx_param)
{
    int i;
    ms_sensor_data_t *sensor_data;

    if ((ahtxx_param->mode & MS_AHTXX_MODE_FILTER_MODE_MASK) == MS_AHTXX_MODE_FILTER_ENABLE) {
        if (ahtxx_param->overflow_line_nr > 0) {
            if (!ms_access_ok(ahtxx_param->overflow_line, \
                 sizeof(ms_sensor_data_t) * ahtxx_param->overflow_line_nr, MS_ACCESS_R)) {
                return MS_ERR;
            }
        }

        if (ahtxx_param->underflow_line_nr > 0) {
            if (!ms_access_ok(ahtxx_param->underflow_line, \
                 sizeof(ms_sensor_data_t) * ahtxx_param->underflow_line_nr, MS_ACCESS_R)) {
                return MS_ERR;
            }
        }

        sensor_data = ahtxx_param->overflow_line;
        for (i = 0; i < ahtxx_param->overflow_line_nr; i++) {
            if (MS_SENSOR_TYPE_TEMP == sensor_data->type) {
                priv->temp_line_enable[AHTXX_OVERFLOW_LINE] = MS_TRUE;
                priv->temp_line[AHTXX_OVERFLOW_LINE] = sensor_data->u.temp;
            } else if (MS_SENSOR_TYPE_HUMI == sensor_data->type) {
                priv->humi_line_enable[AHTXX_OVERFLOW_LINE] = MS_TRUE;
                priv->humi_line[AHTXX_OVERFLOW_LINE] = sensor_data->u.humi;
            } else {
                return MS_ERR;
            }
            sensor_data++;
        }

        sensor_data = ahtxx_param->underflow_line;
        for (i = 0; i < ahtxx_param->underflow_line_nr; i++) {
            if (MS_SENSOR_TYPE_TEMP == sensor_data->type) {
                priv->temp_line_enable[AHTXX_UNDERFLOW_LINE] = MS_TRUE;
                priv->temp_line[AHTXX_UNDERFLOW_LINE] = sensor_data->u.temp;
            } else if (MS_SENSOR_TYPE_HUMI == sensor_data->type) {
                priv->humi_line_enable[AHTXX_UNDERFLOW_LINE] = MS_TRUE;
                priv->humi_line[AHTXX_UNDERFLOW_LINE] = sensor_data->u.humi;
            } else {
                return MS_ERR;
            }
            sensor_data++;
        }
    }

    priv->mode = ahtxx_param->mode;
    priv->interval = ahtxx_param->interval;

    return MS_ERR_NONE;
}

/*
 * Get AHTXX work param.
 */
static ms_err_t __ahtxx_get_work_param(privinfo_t *priv, ms_ahtxx_param_t *ahtxx_param)
{
    int i;
    ms_sensor_data_t *sensor_data;

    if ((priv->mode & MS_AHTXX_MODE_FILTER_MODE_MASK) == MS_AHTXX_MODE_FILTER_ENABLE) {
        if (ahtxx_param->overflow_line_nr > 0) {
            if (!ms_access_ok(ahtxx_param->overflow_line, \
                 sizeof(ms_sensor_data_t) * ahtxx_param->overflow_line_nr, MS_ACCESS_W)) {
                return MS_ERR;
            }
        }

        if (ahtxx_param->underflow_line_nr > 0) {
            if (!ms_access_ok(ahtxx_param->underflow_line, \
                 sizeof(ms_sensor_data_t) * ahtxx_param->underflow_line_nr, MS_ACCESS_W)) {
                return MS_ERR;
            }
        }

        sensor_data = ahtxx_param->overflow_line;
        for (i = 0; i < ahtxx_param->overflow_line_nr; i++) {
            if (MS_SENSOR_TYPE_TEMP == sensor_data->type && priv->temp_line_enable[AHTXX_OVERFLOW_LINE]) {
                sensor_data->u.temp = priv->temp_line[AHTXX_OVERFLOW_LINE];
            } else if (MS_SENSOR_TYPE_HUMI == sensor_data->type && priv->humi_line_enable[AHTXX_OVERFLOW_LINE]) {
                sensor_data->u.humi = priv->humi_line[AHTXX_OVERFLOW_LINE];
            } else {
                return MS_ERR;
            }
            sensor_data++;
        }

        sensor_data = ahtxx_param->underflow_line;
        for (i = 0; i < ahtxx_param->underflow_line_nr; i++) {
            if (MS_SENSOR_TYPE_TEMP == sensor_data->type && priv->temp_line_enable[AHTXX_UNDERFLOW_LINE]) {
                sensor_data->u.temp = priv->temp_line[AHTXX_UNDERFLOW_LINE];
            } else if (MS_SENSOR_TYPE_HUMI == sensor_data->type && priv->humi_line_enable[AHTXX_UNDERFLOW_LINE]) {
                sensor_data->u.humi = priv->humi_line[AHTXX_UNDERFLOW_LINE];
            } else {
                return MS_ERR;
            }
            sensor_data++;
        }
    }

    ahtxx_param->mode = priv->mode;
    ahtxx_param->interval = priv->interval;

    return MS_ERR_NONE;
}

/*
 * Do measure Job.
 * Notify event in io_job_thread or hardware isr.
 */
static void __ahtxx_measure_job(ms_ptr_t arg)
{
    privinfo_t *priv = (privinfo_t *)arg;
    ms_ahtxx_cache_t cache_data[2];

    if (__ahtxx_fetch_data(priv) == MS_ERR_NONE) {
        /*
         * Init cache data
         */
        cache_data[0].timestamp = priv->timestamp;
        cache_data[0].type = MS_SENSOR_TYPE_TEMP;
        cache_data[0].u.temperature = (ms_int32_t)priv->temperature;
        cache_data[1].timestamp = priv->timestamp;
        cache_data[1].type = MS_SENSOR_TYPE_HUMI;
        cache_data[1].u.humidity = (ms_int32_t)priv->humidity;

        if ((priv->mode & MS_AHTXX_MODE_FILTER_MODE_MASK) == MS_AHTXX_MODE_FILTER_DISABLE) {
            /*
             * Put cache data to fifo and notify input event
             */
            ms_fifo_put(&priv->data_fifo, cache_data, sizeof(cache_data));
            __ahtxx_poll_notify(priv, POLLIN);

        } else {
            int checked_ok = MS_TRUE;

            /*
             * Filter sensor data then put it to fifo.
             */
            if ((priv->mode & MS_AHTXX_MODE_DATA_MODE_MASK) == MS_AHTXX_MODE_DATA_IN_RANGE) {
                if (priv->temp_line_enable[AHTXX_UNDERFLOW_LINE] &&
                    (priv->temp_line[AHTXX_UNDERFLOW_LINE] >= priv->temperature)) {
                    checked_ok = MS_FALSE;
                }

                if (priv->temp_line_enable[AHTXX_OVERFLOW_LINE] &&
                    (priv->temp_line[AHTXX_OVERFLOW_LINE] <= priv->temperature)) {
                    checked_ok = MS_FALSE;
                }

                if (checked_ok) {
                    ms_fifo_put(&priv->data_fifo, &cache_data[0], sizeof(ms_ahtxx_cache_t));
                    __ahtxx_poll_notify(priv, POLLIN);
                }

                if (priv->humi_line_enable[AHTXX_UNDERFLOW_LINE] &&
                    (priv->humi_line[AHTXX_UNDERFLOW_LINE] >= priv->humidity)) {
                    checked_ok = MS_FALSE;
                }

                if (priv->humi_line_enable[AHTXX_OVERFLOW_LINE] &&
                    (priv->humi_line[AHTXX_OVERFLOW_LINE] <= priv->humidity)) {
                    checked_ok = MS_FALSE;
                }

                if (checked_ok) {
                    ms_fifo_put(&priv->data_fifo, &cache_data[1], sizeof(ms_ahtxx_cache_t));
                    __ahtxx_poll_notify(priv, POLLIN);
                }

            } else {
                if (priv->temp_line_enable[AHTXX_UNDERFLOW_LINE] &&
                    (priv->temp_line[AHTXX_UNDERFLOW_LINE] < priv->temperature)) {
                    checked_ok = MS_FALSE;
                }

                if (priv->temp_line_enable[AHTXX_OVERFLOW_LINE] &&
                    (priv->temp_line[AHTXX_OVERFLOW_LINE] > priv->temperature)) {
                    checked_ok = MS_FALSE;
                }

                if (checked_ok) {
                    ms_fifo_put(&priv->data_fifo, &cache_data[0], sizeof(ms_ahtxx_cache_t));
                    __ahtxx_poll_notify(priv, POLLIN);
                }

                if (priv->humi_line_enable[AHTXX_UNDERFLOW_LINE] &&
                    (priv->humi_line[AHTXX_UNDERFLOW_LINE] < priv->humidity)) {
                    checked_ok = MS_FALSE;
                }

                if (priv->humi_line_enable[AHTXX_OVERFLOW_LINE] &&
                    (priv->humi_line[AHTXX_OVERFLOW_LINE] > priv->humidity)) {
                    checked_ok = MS_FALSE;
                }

                if (checked_ok) {
                    ms_fifo_put(&priv->data_fifo, &cache_data[1], sizeof(ms_ahtxx_cache_t));
                    __ahtxx_poll_notify(priv, POLLIN);
                }
            }
        }

    } else {
        __ahtxx_poll_notify(priv, POLLERR);
    }
}

/*
 * Control device
 */
static int __ahtxx_ioctl(ms_ptr_t ctx, ms_io_file_t *file, int cmd, void *arg)
{
    privinfo_t *priv = ctx;
    int ret;

    switch (cmd) {
    case MS_SENSOR_AHTXX_CMD_SOFT_RESET: {
        ms_err_t err;

        if (priv->type == MS_AHTXX_TYPE_AHT20) {
            ms_uint8_t data;

            data = AHT20_CMD_SOFT_RESET;
            err = __ahtxx_write(priv, &data, 1);
        } else {
            err = __ahtxx_init(priv);
        }

        if (err == MS_ERR_NONE) {
            ret = 0;
        } else {
            ms_thread_set_errno(EIO);
            ret = -1;
        }
    }
        break;

    case MS_SENSOR_AHTXX_CMD_SET_PARAM:
        if (ms_access_ok(arg, sizeof(ms_ahtxx_param_t), MS_ACCESS_R)) {
            ms_ahtxx_param_t *ahtxx_param = (ms_ahtxx_param_t *)arg;

            if (__ahtxx_set_work_param(priv, ahtxx_param) == MS_ERR_NONE) {
                ret = 0;
            } else {
                ms_thread_set_errno(EINVAL);
                ret = -1;
            }

        } else {
            ms_thread_set_errno(EFAULT);
            ret = -1;
        }
        break;

    case MS_SENSOR_AHTXX_CMD_GET_PARAM:
        if (ms_access_ok(arg, sizeof(ms_ahtxx_param_t), MS_ACCESS_W)) {
            ms_ahtxx_param_t *ahtxx_param = (ms_ahtxx_param_t *)arg;

            if (__ahtxx_get_work_param(priv, ahtxx_param) == MS_ERR_NONE) {
                ret = 0;
            } else {
                ms_thread_set_errno(EINVAL);
                ret = -1;
            }

        } else {
            ms_thread_set_errno(EFAULT);
            ret = -1;
        }
        break;

    case MS_SENSOR_AHTXX_CMD_HANG_MEASURE:
        /*
         * Stop sensor or measure_job
         */
        if (__ahtxx_stop_work(priv) == MS_ERR_NONE) {
            ret = 0;
        } else {
            ms_thread_set_errno(EIO);
            ret = -1;
        }
        break;

    case MS_SENSOR_AHTXX_CMD_RESUME_MEASURE:
        /*
         * Reset priv->data_fifo and start sensor.
         */
        ms_fifo_reset(&priv->data_fifo);
        if (__ahtxx_start_work(priv) == MS_ERR_NONE) {
            ret = 0;
        } else {
            ms_thread_set_errno(EIO);
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
static ms_io_driver_ops_t ms_ahtxx_drv_ops = {
        .type   = MS_IO_DRV_TYPE_CHR,
        .open   = __ahtxx_open,
        .close  = __ahtxx_close,
        .read   = __ahtxx_read,
        .ioctl  = __ahtxx_ioctl,
        .poll   = __ahtxx_poll,
        .fstat  = __ahtxx_fstat,
};

/*
 * Device driver
 */
static ms_io_driver_t ms_ahtxx_drv = {
        .nnode = {
            .name = MS_AHTXX_DRV_NAME,
        },
        .ops = &ms_ahtxx_drv_ops,
};

/*
 * Register AHTXX device driver
 */
ms_err_t ms_ahtxx_drv_register(void)
{
    return ms_io_driver_register(&ms_ahtxx_drv);
}

/*
 * Create AHTXX device file
 */
ms_err_t ms_ahtxx_dev_create(const char *path, const char *i2c_bus_name, ms_uint16_t dev_addr, ms_ahtxx_type_t type)
{
    ms_ahtxx_dev_t *dev;
    ms_uint32_t mem_size;
    ms_uint32_t buf_size;
    ms_ptr_t mem_ptr;
    ms_err_t err;

    if ((path != MS_NULL) && (i2c_bus_name != MS_NULL) && (type < MS_AHTXX_TYPE_MAX)) {

        buf_size = sizeof(ms_ahtxx_cache_t) * AHTXX_DEFAULT_CHACHE_NUM;
        buf_size = ms_roundup_pow2_size(buf_size);
        mem_size = sizeof(ms_ahtxx_dev_t) + buf_size;

        mem_ptr = ms_kmalloc(mem_size);
        dev = (ms_ahtxx_dev_t *)mem_ptr;
        if (dev != MS_NULL) {
            privinfo_t *priv = &dev->priv;

            bzero(priv, sizeof(privinfo_t));
            priv->type = type;

            mem_ptr = mem_ptr + sizeof(ms_ahtxx_dev_t);
            ms_fifo_init(&priv->data_fifo, (ms_uint8_t *)mem_ptr, buf_size);

            err = ms_io_job_init(&priv->measure_job, "ahtxx_measure", __ahtxx_measure_job, (ms_ptr_t)priv);
            if (err == MS_ERR_NONE) {
                /*
                 * Attach i2c device to i2c bus
                 */
                priv->i2c_dev.nnode.name = MS_AHTXX_DRV_NAME;
                priv->i2c_dev.clk_speed  = MS_I2C_CLK_SPEED_STANDARD;
                priv->i2c_dev.addr       = dev_addr;
                priv->i2c_dev.addrlen    = 7U;
                priv->i2c_dev.ctx        = MS_NULL;

                err = ms_i2c_device_attach(&priv->i2c_dev, i2c_bus_name);
                if (err == MS_ERR_NONE) {
                    err = ms_io_device_register(&dev->dev, path, MS_AHTXX_DRV_NAME, &dev->priv);

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
        err = MS_ERR_ARG_INVALID;
    }

    return err;
}

#endif
