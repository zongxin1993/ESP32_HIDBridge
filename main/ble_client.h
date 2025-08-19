#ifndef __BLE_HOST_H__
#define __BLE_HOST_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern bool is_send_info;

void ble_client_init(void);
void ble_send_char(uint8_t* data, size_t data_length);

#endif /* __BLE_CLIENT_H__ */
