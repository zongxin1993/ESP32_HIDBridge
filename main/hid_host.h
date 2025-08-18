#ifndef __HID_HOST_H__
#define __HID_HOST_H__

#include "driver/gpio.h"
#include "errno.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "usb/usb_host.h"

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to
 * distinguish the event by application event group. In this example we have two
 * event groups: APP_EVENT            - General event, which is APP_QUIT_PIN
 * press event (Generally, it is IO0). APP_EVENT_HID_HOST   - HID Host Driver
 * event, such as device connection/disconnection or input report.
 */
typedef enum {
    APP_EVENT = 0,
    APP_EVENT_HID_HOST
} app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
    app_event_group_t event_group;

    /* HID Host - Device related info */
    struct {
        hid_host_device_handle_t handle;
        hid_host_driver_event_t  event;
        void*                    arg;
    } hid_host_device;
} hid_event_queue_t;

void               hid_host_init( void );
void               hid_host_device_event( hid_event_queue_t* evt );
QueueHandle_t      get_queue_handle( void );
hid_event_queue_t* get_event_queue( void );

#endif /* __HID_HOST_H__ */