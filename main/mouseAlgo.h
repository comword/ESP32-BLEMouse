#ifndef APP_MOUSEALGO_H_
#define APP_MOUSEALGO_H_
#include "sensor.h"
void proc_lineacc(sGravity gv);
void proc_quat(const inv_sensor_event_t * event);
void hid_update(uint16_t conn_id);
#endif
