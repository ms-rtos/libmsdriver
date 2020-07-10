/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_drv_sensor.h Sensor device driver.
 *
 * Author: Jiao.jinxing <jiaojixing@acoinfo.com>
 *
 */

#ifndef MS_DRV_SENSOR_H
#define MS_DRV_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Sensor types
 */
#define NS_SENSOR_TYPE_NONE             (0)
#define NS_SENSOR_TYPE_ACCE             (1)  /* Accelerometer     */
#define NS_SENSOR_TYPE_GYRO             (2)  /* Gyroscope         */
#define NS_SENSOR_TYPE_MAG              (3)  /* Magnetometer      */
#define NS_SENSOR_TYPE_TEMP             (4)  /* Temperature       */
#define NS_SENSOR_TYPE_HUMI             (5)  /* Relative Humidity */
#define NS_SENSOR_TYPE_BARO             (6)  /* Barometer         */
#define NS_SENSOR_TYPE_LIGHT            (7)  /* Ambient light     */
#define NS_SENSOR_TYPE_PROXIMITY        (8)  /* Proximity         */
#define NS_SENSOR_TYPE_HR               (9)  /* Heart Rate        */
#define NS_SENSOR_TYPE_TVOC             (10) /* TVOC Level        */
#define NS_SENSOR_TYPE_NOISE            (11) /* Noise Loudness    */
#define NS_SENSOR_TYPE_STEP             (12) /* Step sensor       */
#define NS_SENSOR_TYPE_FORCE            (13) /* Force sensor      */
#define NS_SENSOR_TYPE_DUST             (14) /* Dust sensor       */
#define NS_SENSOR_TYPE_ECO2             (15) /* eCO2 sensor       */
#define NS_SENSOR_TYPE_IR               (16) /* IR sensor         */

/*
 * Sensor unit types
 */
#define MS_SENSOR_UNIT_NONE             (0)
#define MS_SENSOR_UNIT_MG               (1)  /* Accelerometer           unit: mG         */
#define MS_SENSOR_UNIT_MDPS             (2)  /* Gyroscope               unit: mdps       */
#define MS_SENSOR_UNIT_MGAUSS           (3)  /* Magnetometer            unit: mGauss     */
#define MS_SENSOR_UNIT_LUX              (4)  /* Ambient light           unit: lux        */
#define MS_SENSOR_UNIT_CM               (5)  /* Distance                unit: cm         */
#define MS_SENSOR_UNIT_PA               (6)  /* Barometer               unit: pa         */
#define MS_SENSOR_UNIT_PERMILLAGE       (7)  /* Relative Humidity       unit: permillage */
#define MS_SENSOR_UNIT_DCELSIUS         (8)  /* Temperature             unit: dCelsius   */
#define MS_SENSOR_UNIT_HZ               (9)  /* Frequency               unit: HZ         */
#define MS_SENSOR_UNIT_ONE              (10) /* Dimensionless quantity  unit: 1          */
#define MS_SENSOR_UNIT_BPM              (11) /* Heart rate              unit: bpm        */
#define MS_SENSOR_UNIT_MM               (12) /* Distance                unit: mm         */
#define MS_SENSOR_UNIT_MN               (13) /* Force                   unit: mN         */

/*
 * 3-axis data type
 */
typedef struct {
    ms_int32_t  x;
    ms_int32_t  y;
    ms_int32_t  z;
} ms_sensor_3axis_t;

/*
 * Sensor data type
 */
typedef struct {
    ms_tick_t               timestamp;      /* The timestamp when the data was received */
    /*
     * NS_SENSOR_TYPE_XXX
     */
    ms_uint8_t              type;           /* The sensor type of the data            */
    ms_uint8_t              channel;        /* The sensor channel of the data         */
    union {
        ms_sensor_3axis_t   acce;           /* Accelerometer.       unit: mG          */
        ms_sensor_3axis_t   gyro;           /* Gyroscope.           unit: mdps        */
        ms_sensor_3axis_t   mag;            /* Magnetometer.        unit: mGauss      */
        ms_int32_t          temp;           /* Temperature.         unit: dCelsius    */
        ms_int32_t          humi;           /* Relative humidity.   unit: permillage  */
        ms_int32_t          baro;           /* Pressure.            unit: pascal (Pa) */
        ms_int32_t          light;          /* Light.               unit: lux         */
        ms_int32_t          proximity;      /* Distance.            unit: centimeters */
        ms_int32_t          hr;             /* Heart rate.          unit: bpm         */
        ms_int32_t          tvoc;           /* TVOC.                unit: permillage  */
        ms_int32_t          noise;          /* Noise Loudness.      unit: HZ          */
        ms_uint32_t         step;           /* Step sensor.         unit: 1           */
        ms_int32_t          force;          /* Force sensor.        unit: mN          */
        ms_uint32_t         dust;           /* Dust sensor.         unit: ug/m3       */
        ms_uint32_t         eco2;           /* eCO2 sensor.         unit: ppm         */
        ms_int32_t          ir;             /* IR light sensor.     unit: ADC value   */
        ms_uint32_t         reserve[8U];
    } u;
} ms_sensor_data_t;

#ifdef __cplusplus
}
#endif

#endif /* MS_DRV_SENSOR_H */
