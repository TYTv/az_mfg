/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/**
 * @file
 * This is the main file for the manufacturing test app
 *
 * To use the manufacturing test application, please
 * read the instructions in the WICED Manufacturing
 * Test User Guide provided in the <WICED-SDK>/Doc
 * directory: WICED-MFG2xx-R.pdf
 *
 */

#include "wiced_management.h"
#include "wl_tool.h"
#include "mfg_test.h"
#include "internal/wwd_internal.h"
#include "wwd_wifi.h"
#include "bt_mfg_test.h"

/******************************************************
 *                      Macros
 ******************************************************/
#ifdef AWCU359
#define SEL_BTN   WICED_GPIO_6
#elif AWCU427
#define SEL_BTN   WICED_GPIO_6
#else
#define SEL_BTN   WICED_GPIO_1
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
enum mfgselect
{
    mfg_wifi,
    mfg_blue,
    mfg_num,
};
/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct
{
    uint8_t dct_next_mfg_mode;
} dct_mfg_mode;
/******************************************************
 *                    Structures
 ******************************************************/
static const wiced_uart_config_t const stdio1_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
#ifdef WICED_BT_BRIDGE_MODE
    .flow_control = FLOW_CONTROL_CTS_RTS,
#else
    .flow_control = FLOW_CONTROL_DISABLED,
#endif /* WICED_BT_BRIDGE_SUPPORT */
};
/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
void reboot_now(void)
{
    WPRINT_APP_INFO(( "Rebooting...\n" ));
    wiced_rtos_delay_milliseconds( 1000 );
    wiced_framework_reboot();
}
uint8_t read_mfg_dct(void)
{
    dct_mfg_mode* app_dct = NULL;
    uint8_t result;

    wiced_dct_read_lock( (void**) &app_dct, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( *app_dct ) );

    result = app_dct->dct_next_mfg_mode;

    wiced_dct_read_unlock( app_dct, WICED_FALSE );

    return result;
}
void write_mfg_dct(uint8_t mode)
{
    dct_mfg_mode* app_dct = NULL;
    uint32_t time_before_writing;
    uint32_t time_after_writing;
    uint8_t before_mode;

    wiced_dct_read_lock( (void**) &app_dct, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( *app_dct ) );

    before_mode = app_dct->dct_next_mfg_mode;
    app_dct->dct_next_mfg_mode = mode;
    wiced_time_get_time(&time_before_writing);
    wiced_dct_write( (const void*) &(app_dct->dct_next_mfg_mode), DCT_APP_SECTION, OFFSETOF(dct_mfg_mode, dct_next_mfg_mode), sizeof(uint8_t) );
    wiced_time_get_time(&time_after_writing);

    wiced_dct_read_unlock( app_dct, WICED_TRUE );

    WPRINT_APP_INFO( ( "Change mfg mode %d -> %d (time=%ldms)\r\n", before_mode, mode, time_after_writing - time_before_writing) );
}
static void button_press_callback( void* arg )
{
    uint8_t mode = read_mfg_dct();

    mode++;
    if( mode >= mfg_num ){
        mode = mfg_wifi;
    }
    write_mfg_dct(mode);
    reboot_now();
}

void application_start( )
{
    wiced_gpio_init( SEL_BTN, INPUT_PULL_UP );
    wiced_rtos_delay_milliseconds( 1000 );

    uint8_t mode = read_mfg_dct();

    wiced_gpio_input_irq_enable( SEL_BTN, IRQ_TRIGGER_FALLING_EDGE, button_press_callback, NULL );

    switch( mode )
    {
        case mfg_wifi:
            wiced_init( );

            WPRINT_APP_INFO( ( "\r\n------ entry mfg wifi mode ------\r\n" ) );

            /* Disable WIFI sleeping */
            wwd_wifi_set_iovar_value( IOVAR_STR_MPC, 0, WWD_STA_INTERFACE );

#ifdef MFG_TEST_ENABLE_ETHERNET_SUPPORT
            wiced_wlu_server_eth_start( ETHERNET_PORT, 0);
#else
            wiced_wlu_server_serial_start( STDIO_UART );
#endif
            break;
        case mfg_blue:
            wiced_core_init( ); /* Initialise WICED */

            WPRINT_APP_INFO( ( "\r\n------ entry mfg bluetooth mode ------\r\n" ) );

            /* Enter BT manufacturing test mode */
            bt_mfgtest_start( &stdio1_config );
            break;
        default:
            write_mfg_dct(mfg_wifi);
            reboot_now();
            break;
    }

}

