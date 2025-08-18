#include "ble_client.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "hid_host.h"

void app_main( void ) {

    hid_host_init( );
    ble_client_init( );

    while ( 1 ) {
        if ( xQueueReceive(
                 get_queue_handle( ), get_event_queue( ), portMAX_DELAY ) ) {
            if ( APP_EVENT_HID_HOST == get_event_queue( )->event_group ) {
                hid_host_device_event( get_event_queue( ) );
            }
        }
    }
}