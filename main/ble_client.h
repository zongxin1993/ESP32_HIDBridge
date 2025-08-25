#ifndef __BLE_HOST_H__
#define __BLE_HOST_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "usb/hid_host.h"

extern bool is_send_info;

void ble_client_init(void);
void ble_send_keyword(uint8_t* data, size_t data_length);
extern hid_host_device_handle_t hid_device_handle;
#endif /* __BLE_CLIENT_H__ */
