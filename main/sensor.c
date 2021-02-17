#include "sensor.h"
#include "spi.h"
#include "esp_log.h"
#include "mouseAlgo.h"

#include <rom/ets_sys.h>
#include <stdbool.h>

#include "Devices/Drivers/Icm20789-DMP/Icm20789.h"
#include "Devices/Drivers/Icm20789-DMP/Icm20789Defs.h"
#include "Devices/Drivers/Icm20789-DMP/Icm20789MPUFifoControl.h"
#include "Devices/SensorTypes.h"
#include "Devices/SensorConfig.h"
#include "DataConverter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define INV_MSG_TAG "INVMSG"

static const uint8_t dmp3_image[] = {
	#include "icm20789_img.dmp3.h"
};

static const uint8_t EXPECTED_WHOAMI[] = {0x98};  /* WHOAMI value for ICM20689 */
sGravity sGRV;

/* FSR configurations */
int32_t cfg_acc_fsr = 4000; // Accel FSR must be set as +/- 4g and cannot be changed.
int32_t cfg_gyr_fsr = 2000; // Gyro FSR must be set at +/- 2000dps and cannot be changed.

inv_icm20789_t icm_device;

/*
 * Mask to keep track of enabled sensors
 */
uint32_t user_enabled_sensor_mask = 0;

/* Forward declaration */
static void icm20789_apply_mounting_matrix(void);
static const char * inv_error_str(int error);
static void check_rc(int rc, const char * context_str);
static void notify_event(void * context, enum inv_icm20789_sensor sensor, uint64_t timestamp, const void * data, const void *arg);
static void calc_gravity(sGravity *sGRV);
static void store_offsets(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3]);

void sensor_event(const inv_sensor_event_t * event, void * arg);

/*
 * Variable to keep track of the expected period common for all sensors
 */
uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

/*
 * Variable to drop the first timestamp(s) after a sensor start cached by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
//static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
static uint8_t chip_info[3];

/* Keep a copy of the estimated biases loaded from FLASH in RAM */
//static int32_t last_loaded_acc_bias[3] = {0};
//static int32_t last_loaded_gyr_bias[3] = {0};
//static int32_t last_loaded_mag_bias[3] = {0};

/*
 * Mounting matrix configuration applied for both Accel and Gyro
 * The coefficient values are coded in integer q30
 */
static float cfg_mounting_matrix[9]= {1.f,   0,   0,
										0, 1.f,   0,
										0,   0, 1.f };

static uint8_t convert_to_generic_ids[INV_ICM20789_SENSOR_MAX] = {
	INV_SENSOR_TYPE_ACCELEROMETER,
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_CUSTOM_PRESSURE };

/******************************************************************************************/
/*
 * Sleep implementation for ICM20789
 */
void inv_icm20789_sleep(int ms)
{
	vTaskDelay(ms / portTICK_PERIOD_MS);
	//delay_ms(ms);
}

void inv_icm20789_sleep_us(int us)
{
	int v = us / portTICK_PERIOD_MS / 1000;
	if(v > 1)
		vTaskDelay(v);
	else
		ets_delay_us(us);
	//delay_us(us);
}

uint64_t inv_icm20789_get_time_us(void) {
	return xTaskGetTickCount() * (1000 / configTICK_RATE_HZ);
}

/******************************************************************************************/

int icm20789_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;

	ESP_LOGI(INV_MSG_TAG, "Reseting device...");
	rc = inv_icm20789_soft_reset(&icm_device);
	check_rc(rc, "Error reseting device");
	if(rc < 0)
		return -1;
	/*
	 * Just get the whoami
	 */
	rc = inv_icm20789_get_whoami(&icm_device, &whoami);

	//INV_MSG(INV_MSG_LEVEL_INFO, "Device WHOAMI=0x%02x", whoami);
	ESP_LOGI(INV_MSG_TAG, "Device WHOAMI=0x%02x", whoami);
	check_rc(rc, "Error reading WHOAMI");
	if(rc < 0)
		return -1;

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		//INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x.", whoami);
		ESP_LOGE(INV_MSG_TAG, "Bad WHOAMI value. Got 0x%02x.", whoami);
		check_rc(-1, "");
		return -1;
	}
	rc = inv_icm20789_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");
	if(rc < 0)
		return -1;

	/*
	 * Configure and initialize the ICM20789 for normal use
	 */
	//INV_MSG(INV_MSG_LEVEL_INFO, "Booting up device...");
	ESP_LOGI(INV_MSG_TAG, "Booting up device...");

	/* set default power mode */
//	if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE) &&
//		!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER)) {
	if(inv_icm20789_all_sensors_off(&icm_device)) {
		//INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting device in sleep mode...");
		ESP_LOGV(INV_MSG_TAG, "Putting device in sleep mode...");
		inv_icm20789_init_matrix(&icm_device);
		icm20789_apply_mounting_matrix();
		rc = inv_icm20789_initialize(&icm_device, dmp3_image);
		check_rc(rc, "Error %d while setting-up device");
		if(rc < 0)
			return -1;
	}


	/* set default ODR = 50Hz */
	rc = inv_icm20789_set_sensor_period(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up device");
	if(rc < 0)
		return -1;

	rc = inv_icm20789_set_sensor_period(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up device");
	if(rc < 0)
		return -1;

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	//INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");
	ESP_LOGV(INV_MSG_TAG, "We're good to go !");

	return 0;
}

int icm20789_sensor_configuration(void)
{
	int rc;

	//INV_MSG(INV_MSG_LEVEL_INFO, "Configuring accelerometer FSR");
	ESP_LOGI(INV_MSG_TAG, "Configuring accelerometer FSR");
	rc = inv_icm20789_set_accel_fullscale(&icm_device, inv_icm20789_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");
	if(rc < 0)
		return -1;

	//INV_MSG(INV_MSG_LEVEL_INFO, "Configuring gyroscope FSR");
	ESP_LOGI(INV_MSG_TAG, "Configuring gyroscope FSR");
	rc = inv_icm20789_set_gyro_fullscale(&icm_device, inv_icm20789_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");
	if(rc < 0)
		return -1;

	return rc;
}

static const char * inv_error_str(int error)
{
	switch(error) {
	case INV_ERROR_SUCCESS:      return "Success";
	case INV_ERROR:              return "Unspecified error";
	case INV_ERROR_NIMPL:        return "Not implemented";
	case INV_ERROR_TRANSPORT:    return "Transport error";
	case INV_ERROR_TIMEOUT:      return "Timeout, action did not complete in time";
	case INV_ERROR_SIZE:         return "Wrong size error";
	case INV_ERROR_OS:           return "Operating system failure";
	case INV_ERROR_IO:           return "Input/Output error";
	case INV_ERROR_MEM: 		 return "Bad allocation";
	case INV_ERROR_HW:           return "Hardware error";
	case INV_ERROR_BAD_ARG:      return "Invalid arguments";
	case INV_ERROR_UNEXPECTED:   return "Unexpected error";
	case INV_ERROR_FILE:         return "Invalid file format";
	case INV_ERROR_PATH:         return "Invalid file path";
	case INV_ERROR_IMAGE_TYPE:   return "Unknown image type";
	case INV_ERROR_WATCHDOG:     return "Watchdog error";
	default:                     return "Unknown error";
	}
}

/*
 * Helper function to check RC value
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		//INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		ESP_LOGE(INV_MSG_TAG, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		//while(1);
	}
}

int sensor_configure_odr(uint32_t odr_us)
{
	int rc = 0;

	/* All sensors running at the same rate */

	/* Do not reconfigure the rate if it's already applied */
	if(odr_us == period_us)
		return rc;

	if(odr_us < MIN_ODR_US)
		odr_us = MIN_ODR_US;

	if(odr_us > MAX_ODR_US)
		odr_us = MAX_ODR_US;

	/* FIFO has been reset by ODR change */
	if (rc == 0) {
//		pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);

		/* Clear any remaining interrupts */
		//remove isr handler for gpio number.
//		gpio_isr_handler_remove(GPIO_INPUT_GYROINT);
//		__disable_irq();
		//irq_from_device = 0;
//		__enable_irq();
		//hook isr handler for specific gpio pin again
//        gpio_isr_handler_add(GPIO_INPUT_GYROINT, gpio_isr_handler, (void*) GPIO_INPUT_GYROINT);
	}

	/* Keep track in static variable of the odr value for further algorihtm use */
	period_us = odr_us;

	return rc;
}

int sensor_control(int enable)
{
	int rc = 0;
	static uint8_t sensors_on = 0;

	/* Keep track of the sensors state */
	if(enable && sensors_on)
		return rc;

	if(enable)
		sensors_on = 1;
	else
		sensors_on = 0;

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	if (enable) {
		gpio_isr_handler_add(GPIO_INPUT_GYROINT, gpio_isr_handler, (void*) GPIO_INPUT_GYROINT);
		/* Clock is more accurate when gyro is enabled, so let's enable it first to prevent side effect at startup */
//		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE))
//			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE, 1);

		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE, 1);
//		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED))
//			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED, 1);

//		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER))
//			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER, 1);
		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER, 1);

		if (!inv_icm20789_is_sensor_enabled(&icm_device, INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR))
			rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR, 1);

		/*
		 * There is a situation where two samples need to be dropped: if
		 * accelerometer is enable before gyroscope first interrupt triggers,
		 * both interrupts are raised causing the odr to be wrong if only one
		 * sample is dropped.
		 * We are in this exact situation since both sensors are enabled one after
		 * the other.
		 */
		//timestamp_to_drop = 2; //todo: is this needed? -- Qing
	} else {
		//rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_ACCELEROMETER, 0);
		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_ACCELEROMETER, 0);

		//rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_RAW_GYROSCOPE, 0);
		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE, 0);
		//rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GYROSCOPE_UNCALIBRATED, 0);

		rc += inv_icm20789_enable_sensor(&icm_device, INV_ICM20789_SENSOR_GAME_ROTATION_VECTOR, 0);
	}

	/* Clear the remaining items in the IRQ timestamp buffer when stopping all sensors */
	if(inv_icm20789_all_sensors_off(&icm_device))
		gpio_isr_handler_remove(GPIO_INPUT_GYROINT);
//		pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);

	return rc;
}

/*
* Poll devices for data
*/
int Icm20789_data_poll(void) {
	int rc = 0;
	int16_t int_read_back = 0;

	/*
	 *  Ensure data ready status
	 */
	inv_icm20789_identify_interrupt(&icm_device, &int_read_back);

	if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_2 | BIT_MSG_DMP_INT_3)) {
		inv_icm20789_poll_sensor(&icm_device, (void *)0, notify_event);
	}

	//remove isr handler for gpio number.
//	gpio_isr_handler_remove(GPIO_INPUT_GYROINT);
//		__disable_irq();
//	irq_from_device = 0;
//		__enable_irq();
	//hook isr handler for specific gpio pin again
//    gpio_isr_handler_add(GPIO_INPUT_GYROINT, gpio_isr_handler, (void*) GPIO_INPUT_GYROINT);

	return rc;
}

static void icm20789_apply_mounting_matrix(void)
{
	int ii;

	for (ii = 0; ii < INV_ICM20789_SENSOR_MAX; ii++) {
		inv_icm20789_set_matrix(&icm_device, cfg_mounting_matrix, ii);
	}
}

void notify_event(void * context, enum inv_icm20789_sensor sensor, uint64_t timestamp, const void * data, const void *arg)
{
	float raw_bias_data[6];
	inv_sensor_event_t event;
	uint8_t sensor_id = convert_to_generic_ids[sensor];

	memset(&event, 0, sizeof(event));

	event.sensor = sensor_id;
	event.timestamp = timestamp;
	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;

	switch(sensor_id) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
			//INV_MSG(INV_MSG_LEVEL_DEBUG, "%d %d %d\r\n", /*(long)data[0], (long)data[1], (long)data[2],*/
			//event.data.raw3d.vect[0],event.data.raw3d.vect[1],event.data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
			memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
			memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			memcpy(raw_bias_data, data, sizeof(raw_bias_data));
			memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
			memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
			memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
			memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
			event.data.quaternion.accuracy = 0;
			memcpy(&(event.data.quaternion.accuracy_flag), arg, sizeof(event.data.quaternion.accuracy_flag));
			break;

		default:
			return;
	}
	sensor_event(&event, NULL);
}

void sensor_event(const inv_sensor_event_t * event, void * arg)
{
	(void)arg;
	static bool acc_val = false, grv_val = false;
	int i=0;
	switch(event->sensor) {
//		case INV_SENSOR_TYPE_GYROSCOPE:
//			ESP_LOGW("SENSOR_GYRO", "%f,%f,%f", event->data.gyr.vect[0], event->data.gyr.vect[1]
//											 , event->data.gyr.vect[2]);
//		break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
			acc_val = true;
			for(;i < 3;i++)
				sGRV.acc_cal_q16[i] = event->data.acc.vect[i] * (1 << 16);
			if(acc_val && grv_val) {
				calc_gravity(&sGRV);
		//		for(;i < 3;i++)
		//			event->data.acc.vect[i] = (float)sGRV.linearacc_q16[i] / (1 << 16);
		//		event->sensor = INV_SENSOR_TYPE_LINEAR_ACCELERATION;

		//		ESP_LOGW("SENSOR_GRAV", "%f,%f,%f", (float)sGRV.gravity_q16[0] / (1 << 16), (float)sGRV.gravity_q16[1] / (1 << 16)
		//										  , (float)sGRV.gravity_q16[2] / (1 << 16));
		//		ESP_LOGW("SENSOR_LINE", "%f,%f,%f", (float)sGRV.linearacc_q16[0] / (1 << 16), (float)sGRV.linearacc_q16[1] / (1 << 16)
		//										  , (float)sGRV.linearacc_q16[2] / (1 << 16));
				proc_lineacc(sGRV);
			}
//			ESP_LOGW("SENSOR_ACCE", "%f,%f,%f", event->data.acc.vect[0], event->data.acc.vect[1]
//											 , event->data.acc.vect[2]);
		break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
			grv_val = true;
			for(;i < 4;i++)
				sGRV.grv_quat_q30[i] = event->data.quaternion.quat[i] * (1 << 30);
			proc_quat(event);
//			ESP_LOGW("SENSOR_QUAT", "%f,%f,%f,%f", event->data.quaternion.quat[0], event->data.quaternion.quat[1]
//											     , event->data.quaternion.quat[2], event->data.quaternion.quat[3]);
		break;
	}
}

void store_offsets(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3])
{
	uint8_t i, idx = 0;
	int raw_bias[12] = {0};
	uint8_t sensor_bias[84] = {0};

	/* Store offsets in NV memory */
	inv_icm20789_get_st_bias(&icm_device, raw_bias);
	/* Store ST biases: 3(axis) * 4(AccLP, AccLN, GyrLP, GyrLN) * 4(uint32_t) = 48 B [total=48 B] */
	for(i = 0; i < 12; i++)
		inv_dc_int32_to_little8(raw_bias[i], &sensor_bias[i * sizeof(uint32_t)]);
	idx += sizeof(raw_bias);

	/* Store Calib Accel biases: 3(axis) * 4(uint32_t) = 12 B [total=60 B] */
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += (sizeof(acc_bias_q16[0]) * 3);

	/* Store Calib Gyro biases: 3(axis) * 4(uint32_t) = 12 B [total=72 B] */
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(gyr_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += (sizeof(gyr_bias_q16[0]) * 3);
}

int icm20789_run_selftest(void)
{
	int raw_bias[12];
	int rc = 0;

	if (icm_device.selftest_done == 1)
//		INV_MSG(INV_MSG_LEVEL_INFO, "Self-test has already run. Skipping.");
		ESP_LOGI(INV_MSG_TAG, "Self-test has already run. Skipping.");
	else {
		/*
		 * Perform self-test
		 * For ICM20789 self-test is performed for both RAW_ACC/RAW_GYR
		 */
		//INV_MSG(INV_MSG_LEVEL_INFO, "Running self-test...");
		ESP_LOGI(INV_MSG_TAG, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm20789_run_selftest(&icm_device);
		/* Check transport errors */
		check_rc(rc, "Self-test failure");
		if (rc != 0x3) {
			/*
			 * Check for GYR success (1 << 0) and ACC success (1 << 1),
			 * but don't block as these are 'usage' failures.
			 */
			//INV_MSG(INV_MSG_LEVEL_ERROR, "Self-test failure");
			ESP_LOGE(INV_MSG_TAG, "Self-test failure");
			/* 0 would be considered OK, we want KO */
			return INV_ERROR;
		} else
			/* On success, offset will be kept until reset */
			icm_device.selftest_done = 1;

		/* It's advised to re-init the icm20789 device after self-test for normal use */
		rc = icm20789_sensor_setup();
	}

	/*
	 * Get Low Noise / Low Power bias computed by self-tests scaled by 2^16
	 */
	//INV_MSG(INV_MSG_LEVEL_INFO, "Getting LP/LN bias");
	ESP_LOGI(INV_MSG_TAG, "Getting LP/LN bias");
	inv_icm20789_get_st_bias(&icm_device, raw_bias);
	//INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f",
	ESP_LOGI(INV_MSG_TAG, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f",
			(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	//INV_MSG(INV_MSG_LEVEL_INFO, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f",
	ESP_LOGI(INV_MSG_TAG, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f",
			(float)(raw_bias[3] / (float)(1 << 16)), (float)(raw_bias[4] / (float)(1 << 16)), (float)(raw_bias[5] / (float)(1 << 16)));
	//INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f",
	ESP_LOGI(INV_MSG_TAG, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f",
			(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
	//INV_MSG(INV_MSG_LEVEL_INFO, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f",
	ESP_LOGI(INV_MSG_TAG, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f",
			(float)(raw_bias[3 + 6] / (float)(1 << 16)), (float)(raw_bias[4 + 6] / (float)(1 << 16)), (float)(raw_bias[5 + 6] / (float)(1 << 16)));

	return rc;
}

void gyroInit(spi_device_handle_t *handle)
{
	int rc = 0;
	/*
	 * Initialize icm20789 serif structure
	 */
	struct inv_icm20789_serif icm20789_serif;
	icm20789_serif.context   = handle;
	icm20789_serif.read_reg  = idd_io_hal_read_reg;
	icm20789_serif.write_reg = idd_io_hal_write_reg;
	icm20789_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
	icm20789_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */
	/*
	 * Init SPI communication
	 */
	icm20789_serif.is_spi    = 1;
	ESP_LOGI(INV_MSG_TAG, "Opening serial interface through SPI");

	/*
	 * Reset icm20789 driver states
	 */
	inv_icm20789_reset_states(&icm_device, &icm20789_serif);

	/*
	 * Setup the icm20789 device
	 */
	rc = icm20789_sensor_setup();
	if(rc < 0)
		return;
	rc = icm20789_sensor_configuration();
	if(rc < 0)
		return;

	/*
	 * Initializes the default sensor ODR in order to properly init the algorithms
	 */
	//sensor_configure_odr(period_us);

	/*
	 * At boot time, all sensors are turned on.
	 */
	//sensor_control(1);
}

void calc_gravity(sGravity *sGRV)
{
	/*
	 * Compute the gravity data
	 */
	/* x axis */
	sGRV->gravity_q16[0] = (2 * (int32_t)(((int64_t)sGRV->grv_quat_q30[1] * sGRV->grv_quat_q30[3]) >> 30)
			- 2 * (int32_t)(((int64_t)sGRV->grv_quat_q30[0] * sGRV->grv_quat_q30[2]) >> 30)) >> (30 - 16);
	/* y axis */
	sGRV->gravity_q16[1] = (2 * (int32_t)(((int64_t)sGRV->grv_quat_q30[2] * sGRV->grv_quat_q30[3]) >> 30)
			+ 2 * (int32_t)(((int64_t)sGRV->grv_quat_q30[0] * sGRV->grv_quat_q30[1]) >> 30)) >> (30 - 16);
	/* z axis */
	sGRV->gravity_q16[2] = ((1 << 30) - 2 * (int32_t)(((int64_t)sGRV->grv_quat_q30[1] * sGRV->grv_quat_q30[1]) >> 30)
			- 2 * (int32_t)(((int64_t)sGRV->grv_quat_q30[2] * sGRV->grv_quat_q30[2]) >> 30)) >> (30 - 16);
	/*
	 * Compute the linear acceleration data
	 */
	sGRV->linearacc_q16[0] = sGRV->acc_cal_q16[0] - sGRV->gravity_q16[0];
	sGRV->linearacc_q16[1] = sGRV->acc_cal_q16[1] - sGRV->gravity_q16[1];
	sGRV->linearacc_q16[2] = sGRV->acc_cal_q16[2] - sGRV->gravity_q16[2];

}

