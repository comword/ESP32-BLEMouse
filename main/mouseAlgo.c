#include "mouseAlgo.h"
#include "esp_hidd_prf_api.h"
#include "hidd_le_prf_int.h"
#include "esp_log.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef struct {
	float yaw, pitch, roll;
} sEular;

static uint8_t mouse_button = 0;
static int8_t mickeys_x=0, mickeys_y=0, wheel=0, acpan=0;
uint8_t mscale_x=6, mscale_y=6;

static void send_mouse_value(uint16_t);
static bool check_click();
static sEular quat_eular(float, float, float, float);

static void send_mouse_value(uint16_t conn_id)
{
    uint8_t buffer[5];

    buffer[0] = mouse_button;   // Buttons
    buffer[1] = mickeys_x;           // X
    buffer[2] = mickeys_y;           // Y
    buffer[3] = wheel;           // Wheel
    buffer[4] = acpan;           // AC Pan

    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, 5, buffer);
    return;
}

static bool check_click()
{
	return false;
}

void proc_lineacc(sGravity gv)
{
	float x = (float)gv.linearacc_q16[0] / (1 << 16);
	float y = (float)gv.linearacc_q16[1] / (1 << 16);
	float z = (float)gv.linearacc_q16[2] / (1 << 16);
	ESP_LOGW("SENSOR_LINE", "%f,%f,%f", x, y, z);
	//if(abs(x)>0.7)
		mickeys_x = x * mscale_x;
	//else
	//	mickeys_x = 0;
	//if(abs(event->data.acc.vect[1])>0.7)
		mickeys_y = y * mscale_y;
	//else
	//	mickeys_y = 0;
}

void proc_quat(const inv_sensor_event_t * event)
{
	if(check_click())
		mouse_button = 1; //click down
	else
		mouse_button = 0;
}

void hid_update(uint16_t conn_id)
{
	if(mickeys_x==0 && mickeys_y==0 && mouse_button == 0)
		return;
	ESP_LOGI("MOUSE", "Send mouse report: x:%d, y:%d", mickeys_x, mickeys_y);
	send_mouse_value(conn_id);
}

static sEular quat_eular(float x, float y, float z, float w)
{
	sEular result = {
		.yaw = atan2(2*(w*z+x*y), 1-2*(z*z+x*x)),
		.pitch = asin(2*(w*x-y*z)),
		.roll = atan2(2*(w*y+z*x), 1-2*(x*x+y*y))
	};
	return result;
}
