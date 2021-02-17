#ifndef APP_GYRO_H_
#define APP_GYRO_H_

#include "Devices/Drivers/Icm20789-DMP/Icm20789.h"
#include "Devices/SensorTypes.h"
#include "spi.h"

/******************************************************************************/
/* Example configuration                                                      */
/******************************************************************************/

#define DATA_ACCURACY_MASK    ((uint32_t)0x7)

#define DEFAULT_ODR_US    20000    // 20 ms sample spacing = 50 HZ
#define MIN_ODR_US        5000		// 5 ms sample spacing = 200 HZ
#define MAX_ODR_US        1000000	// 1000 ms sample spacing = 1 HZ

extern int sensor_control(int enable);
extern int sensor_configure_odr(uint32_t odr_us);

int Icm20789_data_poll(void);
int icm20789_run_selftest(void);

extern int icm20789_sensor_setup(void);
extern int icm20789_sensor_configuration(void);

extern enum inv_icm20789_sensor idd_sensortype_conversion(int sensor);

extern uint32_t period_us;

extern inv_icm20789_t icm_device;

extern uint32_t user_enabled_sensor_mask;
/*
 * Sensor identifier for control function
 */
enum sensor {
	SENSOR_RAW_ACC,
	SENSOR_RAW_GYR,
	SENSOR_ACC,
	SENSOR_GYR,
	SENSOR_UGYR,
	SENSOR_GRV,
	SENSOR_GRA,
	SENSOR_LINACC,
	SENSOR_MAX
};

//typedef struct {
//	uint8_t sensor;
//	uint8_t status;
//	union {
//		struct {
//			float        vect[3];          /**< x,y,z vector data */
//			float        bias[3];          /**< x,y,z bias vector data */
//			uint8_t      accuracy_flag;    /**< accuracy flag */
//		} acc;                             /**< 3d accelerometer data in g */
//		struct {
//			float        vect[3];          /**< x,y,z vector data */
//			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
//			uint8_t      accuracy_flag;    /**< accuracy flag */
//		} gyr;                             /**< 3d gyroscope data in deg/s */
//		struct {
//			float        quat[4];          /**< w,x,y,z quaternion data */
//			float        accuracy;         /**< heading accuracy in deg */
//			uint8_t      accuracy_flag;    /**< accuracy flag specific for GRV*/
//		} quaternion;                      /**< quaternion data */
//		struct {
//			int32_t      vect[3];          /**< x,y,z vector data */
//			uint32_t     fsr;              /**< full scale range */
//		} raw3d;						   /**< 3d raw acc, mag or gyr*/
//	} data;
//} xSensorEvent;

typedef struct {
	int32_t grv_quat_q30[4];
	int32_t gravity_q16[3];
	int32_t linearacc_q16[3];
	int32_t acc_cal_q16[3];
} sGravity;

#define GPIO_INPUT_GYROINT     27
#define GPIO_INPUT_PIN_SEL  1 << GPIO_INPUT_GYROINT

extern void gpio_isr_handler(void* arg);

void gyroInit(spi_device_handle_t *handle);
int Icm20789_data_poll(void);

#endif
