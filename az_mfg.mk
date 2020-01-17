#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := App_mfg_test

$(NAME)_SOURCES := mfg_test.c

WIFI_FIRMWARE_BIN := firmware/$(WLAN_CHIP)/$(WLAN_CHIP)$(WLAN_CHIP_REVISION)-mfgtest.bin

$(NAME)_COMPONENTS += test/wl_tool \
                      utilities/wifi

$(NAME)_COMPONENTS += drivers/bluetooth/mfg_test
# wl commands which dump a lot of data require big buffers.
GLOBAL_DEFINES   += WICED_PAYLOAD_MTU=8320

#GLOBAL_DEFINES     := WICED_DISABLE_STDIO     # Disable default STDIO. Manufacturing Test uses raw UART

GLOBAL_DEFINES     += BT_BUS_RX_FIFO_SIZE=1024 # Set Bluetooth UART RX FIFO size to large enough number

#NO_WIFI_FIRMWARE   := YES                     # Don't need WLAN FW binary down in the resources
#GLOBAL_DEFINES     += NO_WIFI_FIRMWARE        # Don't need WiFi FW

PLATFORM_WITH_LARGE_POOL_SIZE := BCM943909WCD1_3*
$(eval PLATFORM_WITH_LARGE_POOL_SIZE := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_WITH_LARGE_POOL_SIZE)))
ifeq ($(PLATFORM),$(filter $(PLATFORM),$(PLATFORM_WITH_LARGE_POOL_SIZE)))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=10 \
                  RX_PACKET_POOL_SIZE=10 \
                  PBUF_POOL_TX_SIZE=8 \
                  PBUF_POOL_RX_SIZE=8 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=32+WICED_ETHERNET_DESCNUM_RX
else
ifneq ($(PLATFORM),$(filter $(PLATFORM), BCM943362WCD4_LPCX1769))
# Default values for all other platforms
# Values are higher for LwIP because it has sanity checks and will reject a config
# where the TCP window is larger then the total memory available for Rx buffers
GLOBAL_DEFINES   += TX_PACKET_POOL_SIZE=2 \
                    RX_PACKET_POOL_SIZE=2 \
                    PBUF_POOL_RX_SIZE=5
endif
endif

ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW9MCU7X9N364))
USE_PLATFORM_ALLOCATED_POOL := 0
endif

# ENABLE for ethernet support
#$(NAME)_DEFINES   += MFG_TEST_ENABLE_ETHERNET_SUPPORT

INVALID_PLATFORMS := BCM943362WCD4_LPCX1769 \
                     BCM943362WCDA \
                     BCM9WCD2WLREFAD.BCM94334WLAGB \
                     BCM943909QT

ifeq ($(PLATFORM),$(filter $(PLATFORM),BCM943362WCD4_LPCX1769 BCM94334WLAGB BCM943909QT))
ifneq (yes,$(strip $(TESTER)))
$(error Platform not supported for Manufacturing test)
endif
endif


#==============================================================================
# Flag for app specific
#==============================================================================
GLOBAL_DEFINES   += MFG_TEST_APP
GLOBAL_DEFINES   += WIFI_TEST

